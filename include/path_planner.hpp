#ifndef PATH_PLANNING_PATH_PLANNER_HPP
#define PATH_PLANNING_PATH_PLANNER_HPP

#include "helpers.h"
#include "spline.h"

class PathPlanner {
public:
    // Constructor
    PathPlanner(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);
    ~PathPlanner() = default;

    // Path generation and behavior function
    std::vector<std::vector<double>> generate_path(double car_x, double car_y,
                                                   double car_s, double car_d,
                                                   double car_yaw, double car_speed,
                                                   std::vector<std::vector<double>> prev_path,
                                                   std::vector<std::vector<double>> cars);

private:
    // Trajectory creation
    std::vector<std::vector<double>> path(double car_s, double car_d, int goal_lane,
                                          std::vector<std::vector<double>>& prev_path);

    // Spline calculation
    std::vector<tk::spline> calculate_spline(double car_s, double car_d, int goal_lane);
    std::vector<bool> detect_cars(std::vector<std::vector<double>>& cars, double car_s, int lane);
    int check_lane(double d);

    // Map waypoints
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Number of points in generated path
    const int path_length = 50;

    // Reference velocity
    double speed = 0.0;

    // Velocity/acceleration limits
    const double MAX_SPEED = 49.5;
    const double MAX_ACC = 0.2;
};

#endif //PATH_PLANNING_PATH_PLANNER_HPP
