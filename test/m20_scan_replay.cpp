// Offline stdin replay: no ROS initialization and no velocity publisher.
#include "lidar_tracker.hpp"
#include <iostream>
#include <iomanip>
#include <limits>
#include <sstream>

int main(int argc, char** argv) {
    if (argc != 3 && argc != 4) return 2;
    if (argc == 4 && std::string(argv[3]) != "--m20") return 2;
    SharedState state;
    state.active = true;
    state.is_moving_enabled = true;
    state.setTarget(std::stod(argv[1]), std::stod(argv[2]));
    LidarTracker tracker(state);
#ifndef UPSTREAM_BASELINE
    LidarTracker::Config c;
    c.scan_yaw = 0; c.follow_distance = 1.2; c.corridor_width = .8;
    c.frame_front = c.frame_back = .41;
    c.frame_left = c.frame_right = .253;
    c.gait_aware = argc == 4;
    c.minimum_target_points = c.gait_aware ? 3 : 1;
    tracker.configure(c);
#endif
    geometry_msgs::msg::Twist raw;
    tracker.setVelocityCallback([&](const auto& v) { raw = v; });
    std::cout << std::setprecision(12);
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream in(line);
        auto s = std::make_shared<sensor_msgs::msg::LaserScan>();
        double t; size_t count;
        in >> t >> s->angle_min >> s->angle_increment >> s->range_min >> s->range_max >> count;
        s->header.stamp.sec = static_cast<int>(t);
        s->header.stamp.nanosec = static_cast<unsigned>((t-std::floor(t))*1e9);
        for (size_t j=0; j<count; ++j) {
            double r; in >> r;
            s->ranges.push_back(r < 0 ? std::numeric_limits<float>::infinity() : r);
        }
        if (!in) return 3;
#ifdef UPSTREAM_BASELINE
        // Upstream hardcodes negated XY. Rotate the input to preserve body axes.
        s->angle_min += 3.141592653589793;
#endif
        tracker.processScan(s);
        double x,y; state.getTarget(x,y);
        std::cout << t << ',' << x << ',' << y << ','
                  << raw.linear.x << ',' << raw.linear.y << ',' << raw.angular.z;
#ifndef UPSTREAM_BASELINE
        const auto& d=tracker.diagnostics();
        std::cout << ',' << d.target_points << ',' << d.corridor << ','
                  << d.repulse_x << ',' << d.repulse_y;
#endif
        std::cout << '\n';
    }
}
