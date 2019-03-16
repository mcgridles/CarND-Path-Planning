#ifndef PATH_PLANNING_PATH_PLANNER_HPP
#define PATH_PLANNING_PATH_PLANNER_HPP

#include "helpers.h"
#include "spline.h"

class PathPlanner {
public:
    // Constructor
    PathPlanner(vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, vector<double>& map_waypoints_s);
    ~PathPlanner() = default;

    // Path generation and behavior function
    std::vector<std::vector<double>> generate_path(double car_x, double car_y, double car_s, double car_d,
                                                   double car_yaw, std::vector<std::vector<double>> prev_path,
                                                   std::vector<std::vector<double>> cars);

private:
    // Trajectory creation
    std::vector<std::vector<double>> path(double car_x, double car_y, double car_yaw, double car_s,
                                          double car_d, int goal_lane, double speed_inc,
                                          std::vector<std::vector<double>>& prev_path);

    // Spline calculation
    tk::spline calculate_spline(double car_x, double car_y, double car_yaw, double& ref_x, double& ref_y,
                                double& yaw, double car_s, int goal_lane,
                                std::vector<std::vector<double>>& prev_path);
    std::vector<bool> detect_cars(std::vector<std::vector<double>>& cars, double car_s, int lane);
    double get_follow_speed(std::vector<std::vector<double>>& cars, double car_s, int lane);
    double decelerate(std::vector<std::vector<double>>& cars, double car_s, int lane);
    int check_lane(double d);

    // Map waypoints
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;

    // Number of points in generated path
    const int path_length = 50;

    // Reference velocity
    double speed = 0.0;

    // Velocity/acceleration limits
    const double MAX_SPEED = 49.5;
    const double MAX_ACC = 0.2;

    // Following distances for cars in the same lane or adjacent lanes
    double follow_distance = 45;
    double adjacent_lane_forward = 50;
    double adjacent_lane_backward = 35;
};

#endif //PATH_PLANNING_PATH_PLANNER_HPP
