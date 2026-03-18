/*
 * Pointcloud Clustering Detector Cuda
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#include "pose_exporter.h"
#include <sstream>
#include <iomanip>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

bool PoseExporter::exportFrame(const std::vector<DetectedObjectPtr>& detected_objects,
                                int frame_number,
                                const std::string& output_dir)
{
    // Create output directory if it doesn't exist
    struct stat st = {0};
    if (stat(output_dir.c_str(), &st) == -1)
    {
        mkdir(output_dir.c_str(), 0700);
    }
    
    // Create filename
    std::ostringstream filename;
    filename << output_dir << "/frame_" << std::setw(6) << std::setfill('0') << frame_number << ".json";
    
    // Open file for writing
    std::ofstream file(filename.str());
    if (!file.is_open())
    {
        std::cerr << "[PoseExporter] Error: Could not open file " << filename.str() << std::endl;
        return false;
    }
    
    // Write JSON content
    file << "{\n";
    file << "  \"frame_number\": " << frame_number << ",\n";
    file << "  \"num_objects\": " << detected_objects.size() << ",\n";
    file << "  \"objects\": [\n";
    
    for (size_t i = 0; i < detected_objects.size(); ++i)
    {
        file << "    " << detected_objects[i]->toJSON();
        if (i < detected_objects.size() - 1)
        {
            file << ",";
        }
        file << "\n";
    }
    
    file << "  ]\n";
    file << "}\n";
    
    file.close();
    return true;
}

std::string PoseExporter::toJSONArray(const std::vector<DetectedObjectPtr>& detected_objects)
{
    std::ostringstream oss;
    oss << "[\n";
    
    for (size_t i = 0; i < detected_objects.size(); ++i)
    {
        oss << "  " << detected_objects[i]->toJSON();
        if (i < detected_objects.size() - 1)
        {
            oss << ",";
        }
        oss << "\n";
    }
    
    oss << "]";
    return oss.str();
}
