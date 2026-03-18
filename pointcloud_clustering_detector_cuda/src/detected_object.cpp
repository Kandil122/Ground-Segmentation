/*
 * Pointcloud Clustering Detector Cuda
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#include "detected_object.h"
#include <sstream>
#include <iomanip>
#include <cmath>

DetectedObject DetectedObject::fromPointCloud(const PointCloudPtr& cloud, int object_id)
{
    DetectedObject obj;
    obj.cluster = cloud;
    obj.num_points = cloud->points.size();
    obj.id = object_id;
    
    if (cloud->points.empty())
    {
        return obj;
    }
    
    // Compute centroid
    Eigen::Vector4f centroid_vec;
    pcl::compute3DCentroid(*cloud, centroid_vec);
    obj.centroid.x = centroid_vec[0];
    obj.centroid.y = centroid_vec[1];
    obj.centroid.z = centroid_vec[2];
    obj.centroid.intensity = 0.0f;
    
    // Compute axis-aligned bounding box
    pcl::getMinMax3D(*cloud, obj.min_point, obj.max_point);
    obj.length = obj.max_point.x - obj.min_point.x;
    obj.width = obj.max_point.y - obj.min_point.y;
    obj.height = obj.max_point.z - obj.min_point.z;
    
    // Compute orientation using PCA (Principal Component Analysis)
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid_vec, covariance);
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();
    
    // Eigenvectors are sorted in increasing order of eigenvalues
    // Column 0: smallest variance (normal direction)
    // Column 1: medium variance (secondary direction)
    // Column 2: largest variance (main/principal direction)
    obj.normal_direction = eigenvectors.col(0);
    obj.secondary_direction = eigenvectors.col(1);
    obj.main_direction = eigenvectors.col(2);
    
    // Ensure right-handed coordinate system
    obj.normal_direction = obj.main_direction.cross(obj.secondary_direction);
    
    // Compute Euler angles (yaw, pitch, roll) from the principal direction
    // Yaw: rotation around Z-axis (heading in XY plane)
    obj.yaw = std::atan2(obj.main_direction(1), obj.main_direction(0));
    
    // Pitch: rotation around Y-axis
    float r = std::sqrt(obj.main_direction(0) * obj.main_direction(0) + 
                        obj.main_direction(1) * obj.main_direction(1));
    obj.pitch = std::atan2(-obj.main_direction(2), r);
    
    // Roll: rotation around X-axis (computed from secondary direction)
    obj.roll = std::atan2(obj.secondary_direction(2), obj.normal_direction(2));
    
    return obj;
}

std::string DetectedObject::toJSON() const
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    
    oss << "{\n";
    oss << "  \"id\": " << id << ",\n";
    oss << "  \"num_points\": " << num_points << ",\n";
    oss << "  \"centroid\": {\"x\": " << centroid.x << ", \"y\": " << centroid.y 
        << ", \"z\": " << centroid.z << "},\n";
    oss << "  \"orientation\": {\"yaw\": " << yaw << ", \"pitch\": " << pitch 
        << ", \"roll\": " << roll << "},\n";
    oss << "  \"bounding_box\": {\n";
    oss << "    \"min\": {\"x\": " << min_point.x << ", \"y\": " << min_point.y 
        << ", \"z\": " << min_point.z << "},\n";
    oss << "    \"max\": {\"x\": " << max_point.x << ", \"y\": " << max_point.y 
        << ", \"z\": " << max_point.z << "},\n";
    oss << "    \"dimensions\": {\"length\": " << length << ", \"width\": " << width 
        << ", \"height\": " << height << "}\n";
    oss << "  },\n";
    oss << "  \"principal_direction\": {\"x\": " << main_direction(0) 
        << ", \"y\": " << main_direction(1) << ", \"z\": " << main_direction(2) << "}\n";
    oss << "}";
    
    return oss.str();
}
