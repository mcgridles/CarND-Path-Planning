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
    int goal_lane = 1;
    int lane = check_lane(car_d);

    // Behavior logic
    auto found_cars = detect_cars(cars, car_s, lane);
    if (!found_cars[1]) {
        if (lane != 1) {
            if ((lane == 0 && !found_cars[2]) || (lane == 2 && !found_cars[0])) {
                goal_lane = 1;
            } else {
                goal_lane = lane;
            }
        }

        if (speed < MAX_SPEED) {
            speed += MAX_ACC;
        }
    } else {
        if (!found_cars[0]) {
            goal_lane = lane - 1;
        } else if (!found_cars[2]) {
            goal_lane = lane + 1;
        } else {
            speed -= MAX_ACC;
            goal_lane = lane;
        }
    }

    if (goal_lane < 0) {
        goal_lane = 0;
    } else if (goal_lane > 2) {
        goal_lane = 2;
    }

    // Execute decision
    auto next_vals = path(car_s, car_d, goal_lane, prev_path);

    return next_vals;
}

std::vector<std::vector<double>> PathPlanner::path(double car_s, double car_d, int goal_lane,
                                                   std::vector<std::vector<double>>& prev_path) {

    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    std::vector<tk::spline> spline = calculate_spline(car_s, car_d, goal_lane);

    double target_distance = 50;
    double inc = target_distance / (0.02 * speed / 2);

    for (int i=0; i<path_length; i++) {
        next_x_vals.push_back(spline[0](inc*i));
        next_y_vals.push_back(spline[1](inc*i));
    }

    return {next_x_vals, next_y_vals};
}

std::vector<tk::spline> PathPlanner::calculate_spline(double car_s, double car_d, int goal_lane) {
    std::vector<double> spline_idx;
    std::vector<double> spline_x_vals;
    std::vector<double> spline_y_vals;

    int lane = check_lane(car_d);
    for (int i=30; i<=90; i+=30) {
        double d;
        if (i == 0) {
            d = lane;
        } else {
            d = goal_lane;
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

std::vector<bool> PathPlanner::detect_cars(std::vector<std::vector<double>>& cars, double car_s, int lane) {
    double safe_distance = 30;

    bool car_front = false;
    bool car_left = false;
    bool car_right = false;
    for (auto car : cars) {
        double vx = car[0];
        double vy = car[1];
        double s = car[2];
        double d = car[3];

        int car_lane = check_lane(d);

        if (car_lane == -1) {
            continue;
        }

        double v = sqrt(vx*vx + vy*vy);
        s += v * path_length * 0.02;

        if (car_lane == lane) {
            car_front |= s > car_s && s - car_s < safe_distance;
        } else if (lane - car_lane == 1) {
            car_left |= s > car_s - safe_distance && s < car_s + safe_distance;
        } else if (lane - car_lane == -1) {
            car_right |= s > car_s - safe_distance && s < car_s + safe_distance;
        }
    }

    return {car_left, car_front, car_right};
}

int PathPlanner::check_lane(double d) {
    if (d > 0 && d < 4) {
        return 0;
    } else if (d > 4 && d < 8) {
        return 1;
    } else if (d > 8 && d < 12) {
        return 2;
    } else {
        return -1;
    }
}
