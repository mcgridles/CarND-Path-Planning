#include <iostream>
#include <math.h>

#include "path_planner.hpp"

PathPlanner::PathPlanner(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                         vector<double> map_waypoints_dx, vector<double> map_waypoints_dy) {
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
    this->map_waypoints_dx = map_waypoints_dx;
    this->map_waypoints_dy = map_waypoints_dy;
}

std::vector<std::vector<double>> PathPlanner::generate_path(double car_x, double car_y,
                                                            double car_s, double car_d,
                                                            double car_yaw, double car_speed,
                                                            std::vector<std::vector<double>> prev_path,
                                                            std::vector<std::vector<double>> cars) {
    /*
     * detect_cars
     *
     * if no car ahead:
     *  speed += MAX_ACC
     *  if (speed > MAX_SPEED) {
     *      speed = MAX_SPEED;
     *  } else if (speed < MAX_ACC) {
     *      speed = MAX_ACC;
     *  }
     *  follow_lane
     * else:
     *  if no car right:
     *      change_right
     *  elif no car left:
     *      change_left
     *  else:
     *      speed -= MAX_ACC
     *      follow_lane
     */

    auto path = follow_lane(car_s, round(car_d));

    return path;
}

std::vector<std::vector<double>> PathPlanner::follow_lane(double car_s, double car_d) {
    return path(car_s, car_d, car_d);
}

std::vector<std::vector<double>> PathPlanner::change_right(double car_s, double car_d) {
    double goal_d = car_d + 4;
    if (goal_d > 10) {
        goal_d = 10;
    }

    return path(car_s, car_d, goal_d);
}

std::vector<std::vector<double>> PathPlanner::change_left(double car_s, double car_d) {
    double goal_d = car_d + 4;
    if (goal_d < 2) {
        goal_d = 2;
    }

    return path(car_s, car_d, goal_d);
}

std::vector<std::vector<double>> PathPlanner::path(double car_s, double car_d, double goal_d) {

    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    std::vector<tk::spline> spline = calculate_spline(car_s, car_d, goal_d);

    double target_distance = 50;
    double inc = target_distance / (0.02 * speed / 2);

    for (int i=0; i<path_length; i++) {
        next_x_vals.push_back(spline[0](inc*i));
        next_y_vals.push_back(spline[1](inc*i));
    }

    return {next_x_vals, next_y_vals};
}

std::vector<tk::spline> PathPlanner::calculate_spline(double car_s, double car_d, double goal_d) {
    std::vector<double> spline_idx;
    std::vector<double> spline_x_vals;
    std::vector<double> spline_y_vals;

    for (int i=30; i<=90; i+=30) {
        double d;
        if (i == 0) {
            d = car_d;
        } else {
            d = goal_d;
        }

        double s = car_s + i;

        auto next_pos = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

        spline_idx.push_back(i);
        spline_x_vals.push_back(next_pos[0]);
        spline_y_vals.push_back(next_pos[1]);
    }

    tk::spline sx;
    tk::spline sy;

    sx.set_points(spline_idx, spline_x_vals);
    sy.set_points(spline_idx, spline_y_vals);

    return {sx, sy};
}
