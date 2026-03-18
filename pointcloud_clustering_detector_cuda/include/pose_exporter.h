/*
 * Pointcloud Clustering Detector Cuda
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#pragma once

#include "detected_object.h"
#include <string>
#include <vector>
#include <fstream>

/**
 * @brief Utility class for exporting detected objects to JSON files.
 */
class PoseExporter
{
public:
    /**
     * @brief Exports a frame of detected objects to a JSON file.
     * @param detected_objects Vector of detected objects.
     * @param frame_number Frame number for naming.
     * @param output_dir Directory to save the JSON file.
     * @return True if export was successful, false otherwise.
     */
    static bool exportFrame(const std::vector<DetectedObjectPtr>& detected_objects,
                           int frame_number,
                           const std::string& output_dir = "output");
    
    /**
     * @brief Exports all detected objects to a single JSON array string.
     * @param detected_objects Vector of detected objects.
     * @return JSON array string.
     */
    static std::string toJSONArray(const std::vector<DetectedObjectPtr>& detected_objects);
};
