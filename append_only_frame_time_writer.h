//
// Created by Rafayel Gardishyan on 04/06/2025.
//
#pragma once
#include <stdexcept>
#include <string>
#include <fstream>

#include "light_sampler.hpp"

class append_only_frame_time_writer {
public:
    static void write_frame_time(const std::string& filename, SamplingMode mode, float frame_time) {
        // Check if the file exists, if not create it
        std::ifstream file_check(filename);
        if (!file_check.is_open()) {
            // Create the file if it does not exist
            std::ofstream create_file(filename);
            if (!create_file.is_open()) {
                throw std::runtime_error("Could not create file: " + filename);
            }
            // Write the header to the file
            create_file << "mode,frame_time(ms)\n";
            create_file.close();
        } else {
            file_check.close();
        }

        // open the csv file in append mode
        std::ofstream file(filename, std::ios::app);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file: " + filename);
        }

        // write the frame time to the file
        file << mode << "," << frame_time << "\n";
    }
};



