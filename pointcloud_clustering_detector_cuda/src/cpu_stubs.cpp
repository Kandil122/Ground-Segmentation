#include "ransac.h"
#include "object_detector.h"
#include <iostream>
#include <pcl/segmentation/sac_segmentation.h>

// Ransac Implementation (CPU-based using PCL)
Ransac::Ransac(const YAML::Node &config)
{
    // Load parameters (replicating logic that would be in ransac.cu)
    if (config["ransac"]) {
        distance_threshold_ = config["ransac"]["distance_threshold"].as<float>(0.2f);
        max_iterations_ = config["ransac"]["max_iterations"].as<int>(100);
        min_points_ = config["ransac"]["min_points"].as<int>(50);
        verbose_ = config["ransac"]["verbose"].as<bool>(false); // Assuming key exists or default
        timer_ = config["ransac"]["timer"].as<bool>(false);
    } else {
         // Fallback/Default if passed directly or differently
        distance_threshold_ = 0.2f;
        max_iterations_ = 100;
        min_points_ = 50;
        verbose_ = false;
        timer_ = false;
    }
}

void Ransac::estimatePlane(const PointCloudPtr &cloud, std::vector<float> &plane_coeffs)
{
    if (cloud->size() < min_points_) {
        return;
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold_);
    seg.setMaxIterations(max_iterations_);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (coefficients->values.size() == 4) {
        plane_coeffs = coefficients->values;
    } else {
        // Failed
        plane_coeffs.clear();
    }
}


// ObjectDetector CUDA Stub
std::vector<DetectedObjectPtr> detectObjectsCUDA_impl(const PointCloudPtr& non_ground_cloud, const std::vector<float>& ground_plane)
{
    std::cerr << "[ObjectDetector] Error: CUDA method called but compiled with CPU stubs only." << std::endl;
    return {};
}
