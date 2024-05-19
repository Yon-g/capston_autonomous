/*
 * helper_functions.h
 * Some helper functions for the 2D particle filter.
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "map.h"

/*
 * Struct representing one landmark observation measurement.
 */
struct LandmarkObs {
	
	int id;				// Id of matching landmark in the map.
	int x;			// Local (vehicle coordinates) x position of landmark observation [m]
	int y;			// Local (vehicle coordinates) y position of landmark observation [m]
};

/*
 * Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
inline double dist(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline double * getError(double gt_x, double gt_y, double gt_theta, double pf_x, double pf_y, double pf_theta) {
	static double error[3];
	error[0] = fabs(pf_x - gt_x);
	error[1] = fabs(pf_y - gt_y);
	error[2] = fabs(pf_theta - gt_theta);
	error[2] = fmod(error[2], 2.0 * M_PI);
	if (error[2] > M_PI) {
		error[2] = 2.0 * M_PI - error[2];
	}
	return error;
}

/* Reads map data from a file.
 * @param filename Name of file containing map data.
 * @output True if opening and reading file was successful
 */
inline bool read_map_data(Map& map , int height, int width) { //2차원 배열을 인덱스값으로 x,y를 지정하고 순서대로 id부여하기
	std::string package_path = ament_index_cpp::get_package_share_directory("capstone");
	std::string file_path = package_path + "/map/map2.txt";
	std::cout << file_path << std::endl;
	// Get file of map:
	std::ifstream file(file_path);
	// Return if we can't open the file.
	if (!file.is_open()) {
		return false;
	}
	
	// Declare single line of map file:
	std::string line_map;
	int count = 0;
	for (int i = 0; i < height; ++i) {
        std::string line;
        std::getline(file, line);
        if (line.length() != width) {
            std::cout << "잘못된 줄 길이: " << i << std::endl;
            return 1;
        }
        for (int j = 0; j < width; ++j) {
            if (line[j] == '1') {
				Map::single_landmark_s single_landmark_temp;

				// Set values
				single_landmark_temp.id_i = count++;
				single_landmark_temp.x_f  = i;
				single_landmark_temp.y_f  = j;
				map.landmark_list.push_back(single_landmark_temp);
            }
        }
    }
	return true;
}

#endif /* HELPER_FUNCTIONS_H_ */
