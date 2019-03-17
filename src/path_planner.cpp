#include <iostream>
#include <math.h>

#include "path_planner.hpp"

PathPlanner::PathPlanner(vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, vector<double>& map_waypoints_s) {
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
}

std::vector<std::vector<double>> PathPlanner::generate_path(double car_x, double car_y, double car_s, double car_d,
                                                            double car_yaw, std::vector<std::vector<double>> prev_path,
                                                            std::vector<std::vector<double>> cars) {
    int lane = check_lane(car_d);
    int goal_lane = 1;
    double speed_inc = 0;

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

        if (speed_inc < MAX_SPEED) {
            speed_inc += MAX_ACC;
        }
    } else {
        if (!found_cars[0] && lane != 0) {
            goal_lane = lane - 1;
        } else if (!found_cars[2] && lane != 2) {
            goal_lane = lane + 1;
        } else {
            goal_lane = lane;
        }

        double follow_speed = get_follow_speed(cars, car_s, lane);
        if (speed > follow_speed) {
            speed_inc -= MAX_ACC;
        }
    }

    if (goal_lane < 0) {
        goal_lane = 0;
    } else if (goal_lane > 2) {
        goal_lane = 2;
    }

    // Execute decision
    auto next_vals = path(car_x, car_y, car_yaw, car_s, car_d, goal_lane, speed_inc, prev_path);

    return next_vals;
}

std::vector<std::vector<double>> PathPlanner::path(double car_x, double car_y, double car_yaw, double car_s,
                                                   double car_d, int goal_lane, double speed_inc,
                                                   std::vector<std::vector<double>>& prev_path) {
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    long prev_path_size = 0;
    if (!prev_path.empty()) {
        std::vector<double> prev_x_vals = prev_path[0];
        std::vector<double> prev_y_vals = prev_path[1];
        prev_path_size = prev_x_vals.size();

        for (int i=0; i<prev_path_size; i++) {
            next_x_vals.push_back(prev_x_vals[i]);
            next_y_vals.push_back(prev_y_vals[i]);
        }
    }

    tk::spline spline = calculate_spline(car_x, car_y, car_yaw, ref_x, ref_y, ref_yaw, car_s, goal_lane, prev_path);

    double goal_x = 30.0;
    double goal_y = spline(goal_x);
    double goal_distance = sqrt(pow(goal_x, 2) + pow(goal_y, 2));
    double x_inc = 0;

    for (int i=0; i<path_length - prev_path_size; i++) {
        // Make sure speed is within set bounds
        speed += speed_inc;

        if ( speed > MAX_SPEED ) {
            speed = MAX_SPEED;
        }

        double num_steps = goal_distance / (0.02 * speed / 2.24);
        double next_x = x_inc + goal_x / num_steps;
        double next_y = spline(next_x);

        x_inc = next_x;

        double x = next_x;
        double y = next_y;

        next_x = ref_x + (x * cos(ref_yaw) - y * sin(ref_yaw));
        next_y = ref_y + (x * sin(ref_yaw) + y * cos(ref_yaw));

        next_x_vals.push_back(next_x);
        next_y_vals.push_back(next_y);
    }

    return {next_x_vals, next_y_vals};
}

tk::spline PathPlanner::calculate_spline(double car_x, double car_y, double car_yaw, double& ref_x, double& ref_y,
                                         double& ref_yaw, double car_s, int goal_lane,
                                         std::vector<std::vector<double>>& prev_path) {
    std::vector<double> spline_x_vals;
    std::vector<double> spline_y_vals;

    std::vector<double> prev_x_vals;
    std::vector<double> prev_y_vals;
    long prev_path_size = 0;
    if (!prev_path.empty()) {
        prev_x_vals = prev_path[0];
        prev_y_vals = prev_path[1];
        prev_path_size = prev_x_vals.size();
    }

    // If no previous path
    if (prev_path_size < 2) {
        spline_x_vals.push_back(car_x - cos(car_yaw));
        spline_x_vals.push_back(car_x);

        spline_y_vals.push_back(car_y - sin(car_yaw));
        spline_y_vals.push_back(car_y);
    } else {
        // Use last 2 values of previous path
        ref_x = prev_x_vals[prev_path_size - 1];
        double x_prev = prev_x_vals[prev_path_size - 2];
        ref_y = prev_y_vals[prev_path_size - 1];
        double y_prev = prev_y_vals[prev_path_size - 2];

        ref_yaw = atan2(ref_y-y_prev, ref_x-x_prev);

        spline_x_vals.push_back(x_prev);
        spline_x_vals.push_back(ref_x);

        spline_y_vals.push_back(y_prev);
        spline_y_vals.push_back(ref_y);
    }

    // Get waypoints from 30, 60, and 90 m ahead
    std::vector<std::vector<double>> next_waypoints = {
            getXY(car_s + 50, 2 + 4*goal_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y),
            getXY(car_s + 60, 2 + 4*goal_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y),
            getXY(car_s + 90, 2 + 4*goal_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)
    };

    for (auto waypoint : next_waypoints) {
        spline_x_vals.push_back(waypoint[0]);
        spline_y_vals.push_back(waypoint[1]);
    }

    // Convert from world to car coordinates
    for (int i = 0; i < spline_x_vals.size(); i++) {
        double shift_x = spline_x_vals[i] - ref_x;
        double shift_y = spline_y_vals[i] - ref_y;

        spline_x_vals[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
        spline_y_vals[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
    }

    tk::spline s;
    s.set_points(spline_x_vals, spline_y_vals);

    return s;
}

std::vector<bool> PathPlanner::detect_cars(std::vector<std::vector<double>>& cars, double car_s, int lane) {
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
            car_front |= s > car_s && s - car_s < follow_distance;
        } else if (car_lane == lane - 1) {
            car_left |= s > car_s - adjacent_lane_backward && s < car_s + adjacent_lane_forward;
        } else if (car_lane == lane + 1) {
            car_right |= s > car_s - adjacent_lane_backward && s < car_s + adjacent_lane_forward;
        }
    }

    return {car_left, car_front, car_right};
}

double PathPlanner::get_follow_speed(std::vector<std::vector<double>>& cars, double car_s, int lane) {
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

        if (car_lane == lane && s > car_s) {
            return v * 2.237; // Convert from m/s to mph
        }
    }

    return -1;
}

double PathPlanner::decelerate(std::vector<std::vector<double>>& cars, double car_s, int lane) {
    double follow_speed = -1;
    double follow_dist = -1;
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

        if (car_lane == lane && s > car_s) {
            follow_speed = v * 2.237; // Convert from m/s to mph
            follow_dist = s;
            break;
        }
    }

    double acc = 0;
    if (speed > follow_speed) {
        if (follow_dist > 10) {
            acc = MAX_ACC;
        } else {
            acc = MAX_ACC + (MAX_ACC * log(10 - follow_dist));
        }
    }

    return acc;
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
