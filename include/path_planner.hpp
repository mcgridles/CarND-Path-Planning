//
// Created by Henry Gridley on 2/27/19.
//

#ifndef PATH_PLANNING_PATH_PLANNER_HPP
#define PATH_PLANNING_PATH_PLANNER_HPP

#include "helpers.h"

class PathPlanner {
public:
    std::vector<std::vector<double>> generate_path(double car_x, double car_y, double car_s, double car_d,
            double car_yaw, double car_speed);

    // Test functions provided by Udacity
    std::vector<std::vector<double>> straight_path(double car_x, double car_y, double car_yaw);
    std::vector<std::vector<double>> curve_path(double car_x, double car_y, double car_yaw);
};

#endif //PATH_PLANNING_PATH_PLANNER_HPP
