#ifndef GROUND_REMOVAL_H
#define GROUND_REMOVAL_h

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace Eigen;
using namespace pcl;

/* Parameter */
const int numChannel = 80;
const int numBin = 120;
const int numMedianKernel = 1;
const float rMin = 0.5;
const float rMax = 10;
const float tHmin = -1.0;
const float tHmax = -0.7;

const float tHDiff = 0.3;
const float hSensor = 0.8;

class Cell{
    private:
        float smoothed;
        float height;
        float hDiff;
        float hGround;
        float minZ;
        bool isGround;
    public:
        Cell(){
            minZ = 1000;
            isGround = false;
        }
        void updateMinZ(float z){if(z<minZ)minZ=z;}
        void updataHeight(float h) {height = h;}
        void updateSmoothed(float s) {smoothed = s;}
        void updateHDiff(float hd){hDiff = hd;}
        void updateGround(){isGround = true; hGround = height;}

        bool getIsGround(){return isGround;}
        float getMinZ() {return minZ;}
        float getHeight(){return height;}
        float getHDiff(){ return hDiff;}
        float getSmoothed() {return smoothed;}
        float getHGround() {return hGround;}
};

void filterCloud(PointCloud<PointXYZ> cloud, PointCloud<PointXYZ> & filteredCloud){
    for (int i = 0; i < cloud.size(); i++) {
        float x = cloud.points[i].x;
        float y = cloud.points[i].y;
        float z = cloud.points[i].z;

        float distance = sqrt(x * x + y * y);
        if(distance <= rMin || distance >= rMax) {
            continue;
        }
        else{
            pcl::PointXYZ o;
            o.x = x;
            o.y = y;
            o.z = z;
            filteredCloud.push_back(o);
        }
    }
}

double gauss(double sigma, double x) {
    double expVal = -1 * (pow(x, 2) / pow(2 * sigma, 2));
    double divider = sqrt(2 * M_PI * pow(sigma, 2));
    return (1 / divider) * exp(expVal);
}


std::vector<double> gaussKernel(int samples, double sigma) {
    std::vector<double> kernel(samples);
    double mean = samples/2;
    double sum = 0.0;
    for (int x = 0; x < samples; ++x) {
        kernel[x] = exp( -0.5 * (pow((x-mean)/sigma, 2.0)))/(2 * M_PI * sigma * sigma);
        sum += kernel[x];
    }

    for (int x = 0; x < samples; ++x){
        kernel[x] /= sum;
    }

    assert(kernel.size() == samples);

    return kernel;
}

void gaussSmoothen(std::array<Cell, numBin>& values, double sigma, int samples) {
    auto kernel = gaussKernel(samples, sigma);
    int sampleSide = samples / 2;
    unsigned long ubound = values.size();
    for (long i = 0; i < ubound; i++) {
        double smoothed = 0;
        for (long j = i - sampleSide; j <= i + sampleSide; j++) {
            if (j >= 0 && j < ubound) {
                int sampleWeightIndex = sampleSide + (j - i);
                smoothed += kernel[sampleWeightIndex] * values[j].getHeight();
            }
        }
        values[i].updateSmoothed(smoothed);
    }
}

void getCellIndexFromPoints(float x, float y, int& chI, int& binI){
    float distance = sqrt(x * x + y * y);
    float chP = (atan2(y, x) + M_PI) / (2 * M_PI);
    float binP = (distance - rMin) / (rMax - rMin);
    chI = floor(chP*numChannel);
    binI = floor(binP*numBin);
}

void createAndMapPolarGrid(PointCloud<PointXYZ> cloud,
                           array<array<Cell, numBin>, numChannel>& polarData ){
    for (int i = 0; i < cloud.size(); i++) {
        float x = cloud.points[i].x;
        float y = cloud.points[i].y;
        float z = cloud.points[i].z;

        int chI, binI;
        getCellIndexFromPoints(x, y, chI, binI);
        if(chI < 0 || chI >=numChannel || binI < 0 || binI >= numBin) continue;
        polarData[chI][binI].updateMinZ(z);
    }
}

void computeHDiffAdjacentCell(array<Cell, numBin>& channelData){
    for(int i = 0; i < channelData.size(); i++){
        if(i == 0){
            float hD = channelData[i].getHeight() - channelData[i+1].getHeight();
            channelData[i].updateHDiff(hD);
        }
        else if(i == channelData.size()-1){
            float hD = channelData[i].getHeight() - channelData[i-1].getHeight();
            channelData[i].updateHDiff(hD);
        }
        else{
            float preHD  = channelData[i].getHeight() - channelData[i-1].getHeight();
            float postHD = channelData[i].getHeight() - channelData[i+1].getHeight();
            if(preHD > postHD) channelData[i].updateHDiff(preHD);
            else channelData[i].updateHDiff(postHD);
        }
    }
}

void applyMedianFilter(array<array<Cell, numBin>, numChannel>& polarData){
    for(int channel = 1; channel < polarData.size()-1; channel++){
        for(int bin = 1; bin < polarData[0].size()-1; bin++){
            if(!polarData[channel][bin].getIsGround()){
                if(polarData[channel][bin+1].getIsGround()&&
                   polarData[channel][bin-1].getIsGround()&&
                   polarData[channel+1][bin].getIsGround()&&
                   polarData[channel-1][bin].getIsGround()){
                    vector<float> sur{polarData[channel][bin+1].getHeight(),
                                      polarData[channel][bin-1].getHeight(),
                                      polarData[channel+1][bin].getHeight(),
                                      polarData[channel-1][bin].getHeight()};
                    sort(sur.begin(), sur.end());
                    float m1 = sur[1]; float m2 = sur[2];
                    float median = (m1+m2)/2;
                    polarData[channel][bin].updataHeight(median);
                    polarData[channel][bin].updateGround();
                }
            }
        }
    }
}

void outlierFilter(array<array<Cell, numBin>, numChannel>& polarData){
    for(int channel = 1; channel < polarData.size() - 1; channel++) {
        for (int bin = 1; bin < polarData[0].size() - 2; bin++) {
            if(polarData[channel][bin].getIsGround()&&
               polarData[channel][bin+1].getIsGround()&&
               polarData[channel][bin-1].getIsGround()&&
               polarData[channel][bin+2].getIsGround()){
                float height1 = polarData[channel][bin-1].getHeight();
                float height2 = polarData[channel][bin].getHeight();
                float height3 = polarData[channel][bin+1].getHeight();
                float height4 = polarData[channel][bin+2].getHeight();
                if(height1 != tHmin && height2 == tHmin && height3 != tHmin){
                    float newH = (height1 + height3)/2;
                    polarData[channel][bin].updataHeight(newH);
                    polarData[channel][bin].updateGround();
                }
                else if(height1 != tHmin && height2 == tHmin && height3 == tHmin && height4 != tHmin){
                    float newH = (height1 + height4)/2;
                    polarData[channel][bin].updataHeight(newH);
                    polarData[channel][bin].updateGround();
                }
            }
        }
    }
}

void groundRemove(PointCloud<pcl::PointXYZ>   cloud,
              PointCloud<pcl::PointXYZ>::Ptr  obstacleCloud,
              PointCloud<pcl::PointXYZ>::Ptr  groundCloud){

    PointCloud<pcl::PointXYZ> filteredCloud;

    filterCloud(cloud, filteredCloud);
    array<array<Cell, numBin>, numChannel> polarData;
    createAndMapPolarGrid(filteredCloud, polarData);

    for (int channel = 0; channel < polarData.size(); channel++){
        for (int bin = 0; bin < polarData[0].size(); bin ++){
            float zi = polarData[channel][bin].getMinZ();
            if(zi > tHmin && zi < tHmax){polarData[channel][bin].updataHeight(zi);}
            else if(zi > tHmax){polarData[channel][bin].updataHeight(hSensor);}
            else {polarData[channel][bin].updataHeight(tHmin);}
        }

        gaussSmoothen(polarData[channel], 1, 3);

        computeHDiffAdjacentCell(polarData[channel]);

        for (int bin = 0; bin < polarData[0].size(); bin ++){
            if(polarData[channel][bin].getSmoothed() < tHmax &&
                    polarData[channel][bin].getHDiff() < tHDiff){
                polarData[channel][bin].updateGround();
            }
            else if(polarData[channel][bin].getHeight() < tHmax &&
                    polarData[channel][bin].getHDiff() < tHDiff){
                polarData[channel][bin].updateGround();
            }
        }
    }

    applyMedianFilter(polarData);

    outlierFilter(polarData);

    for(int i = 0; i < filteredCloud.size(); i++) {
        float x = filteredCloud.points[i].x;
        float y = filteredCloud.points[i].y;
        float z = filteredCloud.points[i].z;

        pcl::PointXYZ o;
        o.x = x;
        o.y = y;
        o.z = z;
        int chI, binI;
        getCellIndexFromPoints(x, y, chI, binI);

        if(chI < 0 || chI >=numChannel || binI < 0 || binI >= numBin) continue;
        
        if (polarData[chI][binI].getIsGround()) {
            float hGround = polarData[chI][binI].getHGround();
            if (z < (hGround + 0)) {
                groundCloud->push_back(o);
            } else {
                obstacleCloud->push_back(o);
            }
        } else {
            obstacleCloud->push_back(o);
        }
    }
}

void groundRemove(PointCloud<pcl::PointXYZ>   cloud,
              PointCloud<pcl::PointXYZ>::Ptr  obstacleCloud){

    PointCloud<pcl::PointXYZ> filteredCloud;

    filterCloud(cloud, filteredCloud);
    array<array<Cell, numBin>, numChannel> polarData;
    createAndMapPolarGrid(filteredCloud, polarData);

    for (int channel = 0; channel < polarData.size(); channel++){
        for (int bin = 0; bin < polarData[0].size(); bin ++){
            float zi = polarData[channel][bin].getMinZ();
            if(zi > tHmin && zi < tHmax){polarData[channel][bin].updataHeight(zi);}
            else if(zi > tHmax){polarData[channel][bin].updataHeight(hSensor);}
            else {polarData[channel][bin].updataHeight(tHmin);}
        }

        gaussSmoothen(polarData[channel], 1, 3);

        computeHDiffAdjacentCell(polarData[channel]);

        for (int bin = 0; bin < polarData[0].size(); bin ++){
            if(polarData[channel][bin].getSmoothed() < tHmax &&
                    polarData[channel][bin].getHDiff() < tHDiff){
                polarData[channel][bin].updateGround();
            }
            else if(polarData[channel][bin].getHeight() < tHmax &&
                    polarData[channel][bin].getHDiff() < tHDiff){
                polarData[channel][bin].updateGround();
            }
        }
    }

    applyMedianFilter(polarData);

    outlierFilter(polarData);

    for(int i = 0; i < filteredCloud.size(); i++) {
        float x = filteredCloud.points[i].x;
        float y = filteredCloud.points[i].y;
        float z = filteredCloud.points[i].z;

        pcl::PointXYZ o;
        o.x = x;
        o.y = y;
        o.z = z;
        int chI, binI;
        getCellIndexFromPoints(x, y, chI, binI);

        if(chI < 0 || chI >=numChannel || binI < 0 || binI >= numBin) continue;
        
        if (polarData[chI][binI].getIsGround()) {
            float hGround = polarData[chI][binI].getHGround();
            if (z < (hGround + 0.25)) {
            } else {
                obstacleCloud->push_back(o);
            }
        } else {
            obstacleCloud->push_back(o);
        }
    }
}


#endif