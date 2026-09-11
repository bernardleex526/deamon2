// Offline scan replay through the production tracker. No ROS nodes or sockets.
#include "lidar_tracker.hpp"
#include <iomanip>
#include <iostream>
#include <limits>

static auto scanFor(double x, double y, bool person, bool obstacle, bool empty) {
    auto s = std::make_shared<sensor_msgs::msg::LaserScan>();
    s->angle_min = -M_PI;
    s->angle_increment = M_PI / 360;
    s->range_min = .1;
    s->range_max = 12;
    if (empty) return s;
    s->ranges.assign(720, std::numeric_limits<float>::infinity());
    for (size_t i = 0; i < s->ranges.size(); ++i) {
        double a = s->angle_min + i * s->angle_increment;
        double ux = cos(a), uy = sin(a);
        // Static square room at +/-5 m, plus a circular target of radius 0.12 m.
        double r = 5.0 / std::max(std::abs(ux), std::abs(uy));
        if (person) {
            double projection = x * ux + y * uy;
            double discriminant = .12 * .12 - (x*x + y*y - projection*projection);
            if (projection > 0 && discriminant >= 0)
                r = std::min(r, projection - sqrt(discriminant));
        }
        s->ranges[i] = r;
    }
    if (obstacle) s->ranges[360] = .5;
    return s;
}

int main(int argc, char** argv) {
    double follow = argc > 1 ? std::stod(argv[1]) : 2.0;
    SharedState state;
    state.active = true;
    state.is_moving_enabled = true;
    state.setTarget(2, 0);  // Seed once; all later association is the real tracker.
    LidarTracker tracker(state);
    LidarTracker::Config c;
    c.scan_yaw = 0; c.follow_distance = follow;
    c.corridor_width = .8;
    c.frame_front = c.frame_back = .41;
    c.frame_left = c.frame_right = .253;
    tracker.configure(c);
    geometry_msgs::msg::Twist raw;
    tracker.setVelocityCallback([&](const auto& v) { raw = v; });
    std::cout << std::setprecision(9);
    std::cout << "time,stage,truth_x,truth_y,target_x,target_y,target_points,corridor,repulse_x,repulse_y,pre_x,pre_y,pre_yaw,raw_x,raw_y,raw_yaw,ranges\n";
    for (int i = 0; i < 800; ++i) {
        double t = i * .1, r = 3, a = 0;
        std::string stage;
        if (t < 5) { stage="hold_2m"; r=2; }
        else if (t < 10) { stage="walk_to_3m"; r=2+(t-5)/5; }
        else if (t < 15) stage="hold_3m";
        else if (t < 25) { stage="move_right"; a=-(t-15)/10*M_PI/3; }
        else if (t < 30) { stage="two_oclock"; a=-M_PI/3; }
        else if (t < 40) { stage="return_center"; a=-(40-t)/10*M_PI/3; }
        else if (t < 50) { stage="move_left"; a=(t-40)/10*M_PI/3; }
        else if (t < 55) { stage="ten_oclock"; a=M_PI/3; }
        else if (t < 65) { stage="return_again"; a=(65-t)/10*M_PI/3; }
        else if (t < 70) stage="target_lost";
        else if (t < 75) stage="near_obstacle";
        else stage="empty_scan";
        double x=r*cos(a), y=r*sin(a);
        auto s=scanFor(x,y,stage!="target_lost",stage=="near_obstacle",stage=="empty_scan");
        tracker.processScan(s);
        const auto& d=tracker.diagnostics();
        std::cout << t << ',' << stage << ',' << x << ',' << y << ','
                  << d.target_x << ',' << d.target_y << ',' << d.target_points << ','
                  << d.corridor << ',' << d.repulse_x << ',' << d.repulse_y << ','
                  << d.before_limit.linear.x << ',' << d.before_limit.linear.y << ','
                  << d.before_limit.angular.z << ',' << raw.linear.x << ','
                  << raw.linear.y << ',' << raw.angular.z << ",\"[";
        for (size_t j=0; j<s->ranges.size(); ++j) {
            if (j) std::cout << ',';
            if (std::isfinite(s->ranges[j])) std::cout << s->ranges[j];
            else std::cout << "null";
        }
        std::cout << "]\"\n";
    }
}
