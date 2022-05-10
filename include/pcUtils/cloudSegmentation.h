#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/normal_3d.h>

std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SacModel sm){

    pcl::PointCloud<pcl::PointXYZ>::Ptr road_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointIndices::Ptr inliers_seg{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::SACSegmentation<pcl::PointXYZ> segmenter;

    segmenter.setOptimizeCoefficients(true);
    segmenter.setModelType(sm);
    segmenter.setMethodType(pcl::SAC_RANSAC);

    segmenter.setDistanceThreshold(0.1);
    // segmenter.setMaxIterations(100);

    segmenter.setInputCloud(cloud);
    segmenter.segment (*inliers_seg, *coefficients);

	if (inliers_seg->indices.size () == 0)
	{
        /* Failure*/
	}

	pcl::ExtractIndices<pcl::PointXYZ> extract_seg;
	extract_seg.setInputCloud(cloud);
	extract_seg.setIndices(inliers_seg);
    
	extract_seg.setNegative(false);
	extract_seg.filter(*road_cloud);


    extract_seg.setNegative(true);
    extract_seg.filter(*obstacle_cloud);

    return std::make_tuple(road_cloud, obstacle_cloud);

} // std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudSegment()