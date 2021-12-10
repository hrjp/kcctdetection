#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <string>
#include <vector>
#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


//red and orange
const double minH = 0;
const double minS = 127;
const double minV = 0;
const double maxH = 25;
const double maxS = 255;
const double maxV = 255;

class colorTracking
{
ros::NodeHandle nh;
image_transport::ImageTransport it;
image_transport::Subscriber image_sub;
image_transport::Publisher proImgPub, maskImgPub;
ros::Publisher posPub = nh.advertise<std_msgs::Float32MultiArray>("result", 10);

private:
    int centerX = 0;
    int centerY = 0;
    int detectState = 0;
    cv::Mat image, image_g, mask;
    std::vector<float> vecData_row;
    std::vector<float> vecData;
    cv::Mat colorBinarization(const cv::Mat& frame);
    void maxContourAnalysis(const cv::Mat& mask);
    void image_callback(const sensor_msgs::ImageConstPtr& image_message);
public:
    colorTracking();
    ~colorTracking();
};

colorTracking::colorTracking():it(nh)
{
    proImgPub = it.advertise("processedImage", 10);
    maskImgPub = it.advertise("maskImage", 10);
    image_sub = it.subscribe("image", 10, &colorTracking::image_callback, this);
}

colorTracking::~colorTracking()
{
    //cv::destroyAllWindows();
}

cv::Mat colorTracking::colorBinarization(const cv::Mat& frame)
{
    cv::Mat hsv_frame, hsv_min, hsv_max;
    cv::cvtColor(frame, hsv_frame, CV_BGR2HSV);
    //binarization
	cv::inRange(hsv_frame, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), mask);
    //dilate
    cv::Mat kernel1(3,3,CV_8U, cv::Scalar(1));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel1, cv::Point(-1, -1), 5);
    //erode
    cv::Mat kernel2(5,5,CV_8U, cv::Scalar(1));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel2, cv::Point(-1, -1), 25);
    //dilate
    cv::Mat kernel3(3,3,CV_8U, cv::Scalar(1));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel1, cv::Point(-1, -1), 10);


    return mask;
}

void colorTracking::maxContourAnalysis(const cv::Mat& mask)
{
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int i=0;
    int maxConPos = 0;
    for(const auto& contour: contours){
        i++;
        //ignore tiny area
        if (contour.size() < 30) continue;
        //searh max area
        if (contour.max_size() <= contour.size()+1 & contour.max_size() >= contour.size()-1) maxConPos = i;
    }
    //輪郭凸包
    if(contours.size()>0){
        std::vector<cv::Point> approx;
        cv::convexHull(contours[maxConPos], approx);
        cv::Mat pointsf;
        //cv::Mat(contours[maxConPos]).convertTo(pointsf, CV_32F);
        cv::Mat(approx).convertTo(pointsf, CV_32F);
        // 楕円フィッティング
        if(pointsf.rows>4){
            cv::RotatedRect box = cv::fitEllipse(pointsf);
            // 楕円の描画
            cv::ellipse(image, box, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

            centerX = box.center.x - mask.cols/2;
            centerY = mask.rows/2 - box.center.y;
            detectState = 1;
        }
    }else{
        detectState = 0;
    }
}

void colorTracking::image_callback(const sensor_msgs::ImageConstPtr& image_message)
{
    try {
        image = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::BGR8)->image;
        //image_g = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::MONO8)->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //cv::cvtColor(image, image_g, CV_BGR2GRAY);
    mask = colorBinarization(image);
    maxContourAnalysis(mask);

    std::string dState;
    if(detectState){
        dState = "True";
    }else{
        dState = "False";
    }
    std::string cenPos = "X:"+std::to_string(centerX)+",Y:"+std::to_string(centerY)+","+dState;
    cv::putText(image, cenPos, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(255,0,0), 4);
    std_msgs::Float32MultiArray pos;
    pos.data.resize(3);
    pos.data[0] = centerX;
    pos.data[1] = centerY;
    pos.data[2] = detectState; //state
    posPub.publish(pos);

    sensor_msgs::ImagePtr image_pub = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    sensor_msgs::ImagePtr mask_pub = cv_bridge::CvImage(std_msgs::Header(), "mono8", mask).toImageMsg();
    proImgPub.publish(image_pub);
    maskImgPub.publish(mask_pub);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "colorTracking");
    colorTracking ct;

    ros::spin();
    return 0;
}