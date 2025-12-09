#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "yahboomcar_msgs/TargetArray.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <thread>
#include <atomic>
#include <string>
#include <deque>
#include <map>

// Define pi
#define PI 3.141592653589793

using namespace std;

// Historical cone data structure
struct HistoricalCone {
    double x, y;  // Coordinates in odom frame
    ros::Time timestamp;
    string type;  // "left" or "right"
    int id;       // Unique identifier
};

// Global variables
vector<double> global_left_cone_x, global_left_cone_y;
vector<double> global_right_cone_x, global_right_cone_y;
atomic<bool> new_data_arrived(false);
mutex data_mutex;
mutex path_mutex;
mutex history_mutex;

// Historical cones storage
deque<HistoricalCone> historical_cones;
double current_vehicle_x = 0.0, current_vehicle_y = 0.0;
double current_vehicle_yaw = 0.0;  // Add current yaw

// Parameters
double MAX_LINEAR_SPEED = 1.5;  // Maximum linear speed 1.5 m/s
double CONE_MERGE_DISTANCE = 0.5;  // Cone merge distance threshold 0.5 meters

// Odom callback to get current vehicle state
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Update vehicle position from odometry
    current_vehicle_x = msg->pose.pose.position.x;
    current_vehicle_y = msg->pose.pose.position.y;
    
    // Extract yaw from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_vehicle_yaw = yaw;
}

// New ROS callback function
void coneDataCallback(const yahboomcar_msgs::TargetArray::ConstPtr& msg)
{
    lock_guard<mutex> lock(data_mutex);
    
    // Clear old data
    global_left_cone_x.clear();
    global_left_cone_y.clear();
    global_right_cone_x.clear();
    global_right_cone_y.clear();
    
    // Parse TargetArray message
    int left_count = 0;
    int right_count = 0;
    
    for (const auto& target : msg->data) {
        string frame_id = target.frame_id;
        
        if (frame_id.find("left") != string::npos || frame_id.find("Left") != string::npos) {
            global_left_cone_x.push_back(target.distance_x);
            global_left_cone_y.push_back(target.distance_y);
            left_count++;
        } 
        else if (frame_id.find("right") != string::npos || frame_id.find("Right") != string::npos) {
            global_right_cone_x.push_back(target.distance_x);
            global_right_cone_y.push_back(target.distance_y);
            right_count++;
        }
        else {
            if (target.centerx < 320) {
                global_left_cone_x.push_back(target.distance_x);
                global_left_cone_y.push_back(target.distance_y);
                left_count++;
            } else {
                global_right_cone_x.push_back(target.distance_x);
                global_right_cone_y.push_back(target.distance_y);
                right_count++;
            }
        }
    }
    
    new_data_arrived = true;
    ROS_INFO("Received cone data: %d left cones, %d right cones", left_count, right_count);
}

// Update historical cones data
void updateHistoricalCones(const vector<double>& left_x, const vector<double>& left_y,
                          const vector<double>& right_x, const vector<double>& right_y) {
    lock_guard<mutex> lock(history_mutex);
    
    // Add current left cones to historical data
    for (size_t i = 0; i < left_x.size(); ++i) {
        bool is_duplicate = false;
        
        // Check if duplicate with existing historical cones
        for (const auto& hist_cone : historical_cones) {
            if (hist_cone.type == "left") {
                double dist = hypot(left_x[i] - hist_cone.x, left_y[i] - hist_cone.y);
                if (dist < CONE_MERGE_DISTANCE) {
                    is_duplicate = true;
                    break;
                }
            }
        }
        
        if (!is_duplicate) {
            HistoricalCone new_cone;
            new_cone.x = left_x[i];
            new_cone.y = left_y[i];
            new_cone.timestamp = ros::Time::now();
            new_cone.type = "left";
            new_cone.id = historical_cones.size();
            historical_cones.push_back(new_cone);
        }
    }
    
    // Add current right cones to historical data
    for (size_t i = 0; i < right_x.size(); ++i) {
        bool is_duplicate = false;
        
        // Check if duplicate with existing historical cones
        for (const auto& hist_cone : historical_cones) {
            if (hist_cone.type == "right") {
                double dist = hypot(right_x[i] - hist_cone.x, right_y[i] - hist_cone.y);
                if (dist < CONE_MERGE_DISTANCE) {
                    is_duplicate = true;
                    break;
                }
            }
        }
        
        if (!is_duplicate) {
            HistoricalCone new_cone;
            new_cone.x = right_x[i];
            new_cone.y = right_y[i];
            new_cone.timestamp = ros::Time::now();
            new_cone.type = "right";
            new_cone.id = historical_cones.size();
            historical_cones.push_back(new_cone);
        }
    }
    
    // Clean up outdated historical cones (too far from vehicle)
    double max_history_distance = 10.0;  // Keep cones within 10 meters in front of vehicle
    auto it = historical_cones.begin();
    while (it != historical_cones.end()) {
        double dist_to_vehicle = hypot(it->x - current_vehicle_x, it->y - current_vehicle_y);
        if (dist_to_vehicle > max_history_distance) {
            it = historical_cones.erase(it);
        } else {
            ++it;
        }
    }
    
    ROS_INFO("Historical cones count: %zu", historical_cones.size());
}

// Get merged cone data (current frame + historical data)
void getMergedConeData(vector<double>& left_x, vector<double>& left_y,
                      vector<double>& right_x, vector<double>& right_y) {
    lock_guard<mutex> lock1(data_mutex);
    lock_guard<mutex> lock2(history_mutex);
    
    // Add current frame data
    left_x = global_left_cone_x;
    left_y = global_left_cone_y;
    right_x = global_right_cone_x;
    right_y = global_right_cone_y;
    
    // Add historical data
    for (const auto& hist_cone : historical_cones) {
        if (hist_cone.type == "left") {
            left_x.push_back(hist_cone.x);
            left_y.push_back(hist_cone.y);
        } else if (hist_cone.type == "right") {
            right_x.push_back(hist_cone.x);
            right_y.push_back(hist_cone.y);
        }
    }
}

// Cubic spline interpolation class
class Spline {
private:
    vector<double> x, y;
    vector<double> a, b, c, d;
    int n;

public:
    Spline(const vector<double>& x_, const vector<double>& y_) : x(x_), y(y_) {
        if (x_.empty() || y_.empty()) {
            n = 0;
            return;
        }
        
        n = x.size();
        a.resize(n);
        b.resize(n);
        c.resize(n);
        d.resize(n);

        vector<double> h(n - 1);
        for (int i = 0; i < n - 1; ++i) {
            h[i] = x[i + 1] - x[i];
        }

        vector<double> alpha(n - 1);
        for (int i = 1; i < n - 1; ++i) {
            alpha[i] = 3.0 / h[i] * (y[i + 1] - y[i]) - 3.0 / h[i - 1] * (y[i] - y[i - 1]);
        }

        vector<double> l(n), mu(n), z(n);
        l[0] = 1.0;
        mu[0] = 0.0;
        z[0] = 0.0;

        for (int i = 1; i < n - 1; ++i) {
            l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[n - 1] = 1.0;
        z[n - 1] = 0.0;
        c[n - 1] = 0.0;

        for (int i = n - 2; i >= 0; --i) {
            c[i] = z[i] - mu[i] * c[i + 1];
            b[i] = (y[i + 1] - y[i]) / h[i] - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0;
            d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);
            a[i] = y[i];
        }
    }

    double operator()(double t) const {
        if (n == 0) return 0.0;
        if (t <= x[0]) return y[0];
        if (t >= x.back()) return y.back();

        int i = 0;
        while (i < n - 1 && x[i + 1] < t) ++i;

        double dt = t - x[i];
        return a[i] + b[i] * dt + c[i] * dt * dt + d[i] * dt * dt * dt;
    }
};

// Calculate target index
int calc_target_index(const vector<double>& cx, const vector<double>& cy, double Lf) {
    if (cx.empty() || cy.empty()) return 0;
    
    int n = cx.size();
    vector<double> distances(n);
    for (int i = 0; i < n; ++i) {
        distances[i] = hypot(cx[i] - current_vehicle_x, cy[i] - current_vehicle_y);
    }

    int location = 0;
    double min_dist = distances[0];
    for (int i = 1; i < n; ++i) {
        if (distances[i] < min_dist) {
            min_dist = distances[i];
            location = i;
        }
    }

    int ind = location;
    while (ind < n - 1 && hypot(cx[ind] - current_vehicle_x, cy[ind] - current_vehicle_y) < Lf) {
        ind++;
    }

    return ind;
}

// Pure pursuit control
double pure_pursuit_control(const vector<double>& cx, const vector<double>& cy,
    int ind, double k, double Lfc, double L, double& Lf) {
    if (cx.empty() || cy.empty()) return 0.0;
    
    double tx = cx[ind];
    double ty = cy[ind];

    double alpha = atan2(ty - current_vehicle_y, tx - current_vehicle_x) - current_vehicle_yaw;
    Lf = k * 0 + Lfc;  // Use constant lookahead since we don't have velocity
    return atan2(2.0 * L * sin(alpha), Lf);
}

// Data preprocessing function
void preprocess_cone_data(vector<double>& cone_x, vector<double>& cone_y) {
    if (cone_x.empty()) return;
    
    vector<pair<double, double>> points;
    for (size_t i = 0; i < cone_x.size(); ++i) {
        points.emplace_back(cone_x[i], cone_y[i]);
    }
    
    sort(points.begin(), points.end(), 
         [](const pair<double, double>& a, const pair<double, double>& b) {
             return a.first < b.first;
         });
    
    vector<pair<double, double>> filtered_points;
    for (const auto& p : points) {
        if (p.first > 0.5 && p.first < 15.0) {
            filtered_points.push_back(p);
        }
    }
    
    cone_x.clear();
    cone_y.clear();
    for (const auto& p : filtered_points) {
        cone_x.push_back(p.first);
        cone_y.push_back(p.second);
    }
}

// Generate reference path function
void generate_reference_path(
    const vector<double>& left_cone_x, const vector<double>& left_cone_y,
    const vector<double>& right_cone_x, const vector<double>& right_cone_y,
    vector<double>& discrete_path_x, vector<double>& discrete_path_y) 
{
    discrete_path_x.clear();
    discrete_path_y.clear();
    
    vector<double> processed_left_x = left_cone_x;
    vector<double> processed_left_y = left_cone_y;
    vector<double> processed_right_x = right_cone_x;
    vector<double> processed_right_y = right_cone_y;
    
    preprocess_cone_data(processed_left_x, processed_left_y);
    preprocess_cone_data(processed_right_x, processed_right_y);
    
    int N_left = processed_left_x.size();
    int N_right = processed_right_x.size();
    
    if (N_left == 0 && N_right == 0) {
        ROS_WARN("No valid cone data for path generation");
        return;
    }
    
    if (N_left == 0 || N_right == 0) {
        ROS_WARN("Missing one side cones, using single side path generation");
        
        const vector<double>& available_x = (N_left > 0) ? processed_left_x : processed_right_x;
        const vector<double>& available_y = (N_left > 0) ? processed_left_y : processed_right_y;
        int N = available_x.size();
        
        if (N < 2) {
            ROS_WARN("Not enough cones to generate path");
            return;
        }
        
        double lane_width = 2.0;
        for (int i = 0; i < N; ++i) {
            double offset = (N_left > 0) ? -lane_width/2 : lane_width/2;
            discrete_path_x.push_back(available_x[i]);
            discrete_path_y.push_back(available_y[i] + offset);
        }
        
        ROS_INFO("Generated single side path with %d points", N);
        return;
    }

    vector<double> midpoint_x, midpoint_y;
    
    for (size_t i = 0; i < processed_left_x.size(); ++i) {
        double min_dist = 1e9;
        int best_match = -1;
        
        for (size_t j = 0; j < processed_right_x.size(); ++j) {
            double dist = hypot(processed_right_x[j] - processed_left_x[i], 
                               processed_right_y[j] - processed_left_y[i]);
            if (dist < min_dist && dist < 5.0) {
                min_dist = dist;
                best_match = j;
            }
        }
        
        if (best_match != -1) {
            midpoint_x.push_back((processed_left_x[i] + processed_right_x[best_match]) / 2.0);
            midpoint_y.push_back((processed_left_y[i] + processed_right_y[best_match]) / 2.0);
        }
    }
    
    if (midpoint_x.size() < 2) {
        ROS_WARN("Not enough valid cone pairs to generate path");
        return;
    }

    vector<double> distances;
    for (size_t i = 0; i < midpoint_x.size() - 1; ++i) {
        distances.push_back(hypot(midpoint_x[i + 1] - midpoint_x[i], midpoint_y[i + 1] - midpoint_y[i]));
    }

    vector<double> t;
    t.push_back(0.0);
    double sum = 0.0;
    for (double d : distances) {
        sum += d;
        t.push_back(sum);
    }

    Spline spline_x(t, midpoint_x);
    Spline spline_y(t, midpoint_y);

    double t_max = t.back();
    if (t_max < 1e-3) {
        ROS_WARN("Path length too short for discretization");
        return;
    }
    
    int num_points = max(10, static_cast<int>(ceil(t_max / 0.1)));
    discrete_path_x.resize(num_points);
    discrete_path_y.resize(num_points);
    
    for (int i = 0; i < num_points; ++i) {
        double t_val = t_max * i / (num_points - 1);
        discrete_path_x[i] = spline_x(t_val);
        discrete_path_y[i] = spline_y(t_val);
    }
    
    ROS_INFO("Generated reference path: %d points, based on %d cone pairs", 
             num_points, (int)midpoint_x.size());
}

// Create direction arrow marker - simplified version
visualization_msgs::Marker createDirectionArrow(double steering_angle, 
                                               int id, string frame_id = "odom") {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = frame_id;
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "steering_direction";
    arrow.id = id;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    
    // Use current vehicle position
    arrow.pose.position.x = current_vehicle_x;
    arrow.pose.position.y = current_vehicle_y;
    arrow.pose.position.z = 0.2;
    
    // Calculate arrow orientation based on current yaw + steering angle
    double arrow_yaw = current_vehicle_yaw + steering_angle;
    
    arrow.pose.orientation.x = 0.0;
    arrow.pose.orientation.y = 0.0;
    arrow.pose.orientation.z = sin(arrow_yaw / 2.0);
    arrow.pose.orientation.w = cos(arrow_yaw / 2.0);
    
    arrow.scale.x = 1.0;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    
    arrow.color.r = 1.0;
    arrow.color.g = 0.0;
    arrow.color.b = 0.0;
    arrow.color.a = 0.8;
    
    arrow.lifetime = ros::Duration(0.2);
    
    return arrow;
}

// Create path visualization marker
visualization_msgs::Marker createPathMarker(const vector<double>& path_x, const vector<double>& path_y, 
                                           int id, string frame_id = "odom") {
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = frame_id;
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "reference_path";
    path_marker.id = id;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    
    for (size_t i = 0; i < path_x.size(); ++i) {
        geometry_msgs::Point p;
        p.x = path_x[i];
        p.y = path_y[i];
        p.z = 0.1;
        path_marker.points.push_back(p);
    }
    
    path_marker.scale.x = 0.05;
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 0.7;
    
    path_marker.lifetime = ros::Duration(0.5);
    
    return path_marker;
}

// Create historical cones visualization marker
visualization_msgs::Marker createHistoricalConesMarker(int id, string frame_id = "odom") {
    visualization_msgs::Marker cones_marker;
    cones_marker.header.frame_id = frame_id;
    cones_marker.header.stamp = ros::Time::now();
    cones_marker.ns = "historical_cones";
    cones_marker.id = id;
    cones_marker.type = visualization_msgs::Marker::POINTS;
    cones_marker.action = visualization_msgs::Marker::ADD;
    
    lock_guard<mutex> lock(history_mutex);
    for (const auto& cone : historical_cones) {
        geometry_msgs::Point p;
        p.x = cone.x;
        p.y = cone.y;
        p.z = 0.0;
        cones_marker.points.push_back(p);
    }
    
    cones_marker.scale.x = 0.2;
    cones_marker.scale.y = 0.2;
    cones_marker.color.r = 1.0;
    cones_marker.color.g = 0.5;
    cones_marker.color.b = 0.0;
    cones_marker.color.a = 0.6;
    
    cones_marker.lifetime = ros::Duration(0.5);
    
    return cones_marker;
}

// Path tracking main function
void tracking_main() {
    // Create independent node handle in tracking thread
    ros::NodeHandle nh;
    
    // Initialize ROS components
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    ros::Publisher state_pub = nh.advertise<std_msgs::Float32MultiArray>("vehicle_state", 10);
    
    // Initialize subscribers
    ros::Subscriber cone_sub = nh.subscribe("DetectMsg", 10, coneDataCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);  // Subscribe to odometry
    
    ROS_INFO("Path tracking node started");

    // Control parameters
    double k = 0.1;
    double Lfc = 1.0;
    double dt = 0.1;
    double L = 1.2;
    double target_speed = 1.0;  // Target speed 1.0 m/s

    // Wait until cone data is received
    while (ros::ok() && !new_data_arrived) {
        ROS_INFO_THROTTLE(1.0, "Waiting for cone data...");
        ros::spinOnce();
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    double time = 0.0;
    double Lf = Lfc;  // Constant lookahead distance
    int marker_id = 0;

    ROS_INFO("Starting path tracking");

    // Main control loop
    ros::Rate rate(1.0/dt);
    while (ros::ok()) {
        // Process ROS callbacks
        ros::spinOnce();
        
        // Update historical cones data
        {
            lock_guard<mutex> lock(data_mutex);
            updateHistoricalCones(global_left_cone_x, global_left_cone_y, 
                                global_right_cone_x, global_right_cone_y);
        }
        
        // Get merged cone data (current + historical)
        vector<double> merged_left_x, merged_left_y, merged_right_x, merged_right_y;
        getMergedConeData(merged_left_x, merged_left_y, merged_right_x, merged_right_y);
        
        // Generate reference path
        vector<double> discrete_path_x, discrete_path_y;
        generate_reference_path(merged_left_x, merged_left_y, merged_right_x, merged_right_y, 
                               discrete_path_x, discrete_path_y);

        // Copy current path data
        vector<double> current_path_x, current_path_y;
        {
            lock_guard<mutex> lock(path_mutex);
            current_path_x = discrete_path_x;
            current_path_y = discrete_path_y;
        }

        // Check if path is valid
        if (current_path_x.empty() || current_path_y.empty()) {
            ROS_WARN_THROTTLE(1.0, "Path is empty, skipping control cycle");
            rate.sleep();
            continue;
        }

        // Calculate control outputs
        int target_ind = calc_target_index(current_path_x, current_path_y, Lf);
        double target_delta = pure_pursuit_control(current_path_x, current_path_y,
            target_ind, k, Lfc, L, Lf);

        // Limit steering angle
        double max_steering = 30.0 * PI / 180.0;
        target_delta = max(-max_steering, min(max_steering, target_delta));

        // Publish velocity control command
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = target_speed;  // Use constant target speed
        cmd_vel.angular.z = target_delta;  // Steering angle (radians)
        cmd_vel_pub.publish(cmd_vel);

        // Publish visualization markers
        visualization_msgs::Marker direction_arrow = createDirectionArrow(target_delta, marker_id++);
        marker_pub.publish(direction_arrow);
        
        if (marker_id % 10 == 0) {
            visualization_msgs::Marker path_marker = createPathMarker(current_path_x, current_path_y, 1000);
            marker_pub.publish(path_marker);
            
            visualization_msgs::Marker historical_marker = createHistoricalConesMarker(2000);
            marker_pub.publish(historical_marker);
        }

        // Publish debug state
        std_msgs::Float32MultiArray state_msg;
        state_msg.data.clear();
        state_msg.data.push_back(static_cast<float>(target_delta));
        state_msg.data.push_back(static_cast<float>(target_speed));
        state_pub.publish(state_msg);

        // Output information
        double target_delta_deg = target_delta * 180.0 / PI;
        ROS_INFO_THROTTLE(0.5, "Time: %.1fs | Position: (%.2f, %.2f) | Speed: %.2fm/s | Steering: %.1fdeg | Historical cones: %zu", 
                         time, current_vehicle_x, current_vehicle_y, target_speed, target_delta_deg, historical_cones.size());

        time += dt;
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "path_tracking_node");
    
    ROS_INFO("Path tracking node starting...");
    
    // Run all functionality directly in tracking thread
    thread tracking_thread(tracking_main);
    
    // Wait for tracking thread to finish
    if (tracking_thread.joinable()) {
        tracking_thread.join();
    }
    
    return 0;
}