#ifndef PATH_PLANNING_PATH_PLANNER_HPP
#define PATH_PLANNING_PATH_PLANNER_HPP

#include "helpers.h"

class PathPlanner {
public:
    // Constructor
    PathPlanner(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);
    ~PathPlanner() = default;

    // Path generation
    std::vector<std::vector<double>> generate_path(double car_x, double car_y, double car_s, double car_d,
                                                   double car_yaw, double car_speed);

private:
    // Test functions provided by Udacity
    std::vector<std::vector<double>> straight_path(double car_x, double car_y, double car_yaw);
    std::vector<std::vector<double>> curve_path(double car_x, double car_y, double car_yaw);
    std::vector<std::vector<double>> follow_lane(double car_s, double car_d);

    // Map waypoints
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Number of points in generated path
    int path_length = 50;

    // Speed
    double dist_inc = 0.3;
    double max_dist_inc = 0.3;
};

#endif //PATH_PLANNING_PATH_PLANNER_HPP
