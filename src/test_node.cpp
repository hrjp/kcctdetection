#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/ground_plane_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

ros::Publisher pub;

pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
pcl::GroundPlaneComparator<pcl::PointXYZ, pcl::Normal>::Ptr road_comparator(new pcl::GroundPlaneComparator<pcl::PointXYZ, pcl::Normal>);
pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZ, pcl::Label> road_segmentation (road_comparator);

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input){
 
    // Create a container for the data. --------------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    // Compute the normals
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud (cloud);
    ne.compute (*normal_cloud);
 
    // Set up the groundplane comparator
    road_comparator->setInputCloud (cloud);
    road_comparator->setInputNormals (normal_cloud);
/*
    // Run segmentation
    pcl::PointCloud<pcl::Label> labels;
    std::vector<pcl::PointIndices> region_indices;
    road_segmentation.setInputCloud (cloud);
    road_segmentation.segment (labels, region_indices);
 
    // Draw the segmentation result
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
    Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero ();
    Eigen::Vector4f vp = Eigen::Vector4f::Zero ();
    Eigen::Matrix3f clust_cov;
    pcl::ModelCoefficients model;
    model.values.resize (4);
 
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > centroids;
    std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > covariances;
    std::vector<pcl::PointIndices> inlier_indices;
 
    for (const auto &region_index : region_indices){
 
        if (region_index.indices.size () > 1000){
 
            for (size_t j = 0; j < region_index.indices.size (); j++){
 
                pcl::PointXYZ ground_pt (cloud->points[region_index.indices[j]].x,
                                         cloud->points[region_index.indices[j]].y,
                                         cloud->points[region_index.indices[j]].z);
                ground_cloud->points.push_back (ground_pt);
            }
 
            // Compute plane info
            pcl::computeMeanAndCovarianceMatrix (*cloud, region_index.indices, clust_cov, clust_centroid);
            Eigen::Vector4f plane_params;
 
            EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
            EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
            pcl::eigen33 (clust_cov, eigen_value, eigen_vector);
            plane_params[0] = eigen_vector[0];
            plane_params[1] = eigen_vector[1];
            plane_params[2] = eigen_vector[2];
            plane_params[3] = 0;
            plane_params[3] = -1 * plane_params.dot (clust_centroid);
 
            vp -= clust_centroid;
            float cos_theta = vp.dot (plane_params);
            if (cos_theta < 0){
 
                plane_params *= -1;
                plane_params[3] = 0;
                plane_params[3] = -1 * plane_params.dot (clust_centroid);
            }
 
            model.values[0] = plane_params[0];
            model.values[1] = plane_params[1];
            model.values[2] = plane_params[2];
            model.values[3] = plane_params[3];
            model_coefficients.push_back (model);
            inlier_indices.push_back (region_index);
            centroids.push_back (clust_centroid);
            covariances.push_back (clust_cov);
        }
    }
    */
    pub.publish(input);
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "test_node");
    
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, cloud_callback);
    
    // Publishers
    pub = nh.advertise<sensor_msgs::PointCloud2>("test_cloud", 100);

    ros::spin();

    return 0;
} // main