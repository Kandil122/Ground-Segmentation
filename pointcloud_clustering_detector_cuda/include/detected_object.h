/*
 * Pointcloud Clustering Detector Cuda
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#pragma once

#include "constant.h"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <Eigen/Dense>
#include <memory>

/**
 * @brief Structure representing a detected object with pose information.
 */
struct DetectedObject
{
    // Pose information
    pcl::PointXYZI centroid;      // 3D position (center of mass)
    float yaw;                     // Heading angle in radians (rotation around Z-axis)
    float pitch;                   // Pitch angle in radians (rotation around Y-axis)
    float roll;                    // Roll angle in radians (rotation around X-axis)
    
    // Bounding box
    pcl::PointXYZI min_point;     // Minimum corner of axis-aligned bounding box
    pcl::PointXYZI max_point;     // Maximum corner of axis-aligned bounding box
    float length;                  // Bounding box length (max dimension in principal direction)
    float width;                   // Bounding box width
    float height;                  // Bounding box height
    
    // Orientation vectors (from PCA)
    Eigen::Vector3f main_direction;      // Principal component (largest variance)
    Eigen::Vector3f secondary_direction; // Secondary component
    Eigen::Vector3f normal_direction;    // Normal component (smallest variance)
    
    // Point cloud data
    PointCloudPtr cluster;         // Original point cloud cluster
    int num_points;                // Number of points in cluster
    
    // Tracking ID (optional, for multi-frame tracking)
    int id;
    
    /**
     * @brief Computes pose information from a point cloud cluster.
     * @param cloud The input point cloud cluster.
     * @param object_id Optional ID for tracking.
     * @return A DetectedObject with computed pose information.
     */
    static DetectedObject fromPointCloud(const PointCloudPtr& cloud, int object_id = -1);
    
    /**
     * @brief Converts the detected object to a JSON-formatted string.
     * @return JSON string representation of the object.
     */
    std::string toJSON() const;
};

using DetectedObjectPtr = std::shared_ptr<DetectedObject>;
