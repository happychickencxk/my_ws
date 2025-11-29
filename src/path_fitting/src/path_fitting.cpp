#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <thread>
#include <atomic>

// 定义圆周率π
#define PI 3.141592653589793

using namespace std;

// 全局变量存储锥桶数据
vector<double> global_left_cone_x, global_left_cone_y;
vector<double> global_right_cone_x, global_right_cone_y;
atomic<bool> new_data_arrived(false);
mutex data_mutex;

// 添加路径数据互斥锁
mutex path_mutex;

// ROS回调函数
void coneDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  // 检查数据大小是否符合预期（4个数组）
  int total_size = msg->data.size();
  if(total_size % 4 != 0) {
    ROS_WARN("Received invalid array size: %d (expected multiple of 4)", total_size);
    return;
  }
  
  int n = total_size / 4;
  
  lock_guard<mutex> lock(data_mutex);
  
  // 清空旧数据
  global_left_cone_x.clear();
  global_left_cone_y.clear();
  global_right_cone_x.clear();
  global_right_cone_y.clear();
  
  // 解析四个数组的数据
  for(int i = 0; i < n; i++) {
    global_left_cone_x.push_back(msg->data[i]);
  }
  for(int i = n; i < 2*n; i++) {
    global_left_cone_y.push_back(msg->data[i]);
  }
  for(int i = 2*n; i < 3*n; i++) {
    global_right_cone_x.push_back(msg->data[i]);
  }
  for(int i = 3*n; i < 4*n; i++) {
    global_right_cone_y.push_back(msg->data[i]);
  }
  
  new_data_arrived = true;
  ROS_INFO("Received new cone data: %d points per array", n);
}

// 三次样条插值类
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

// 更新状态量
void update(double& x, double& y, double& yaw, double& v,
    double a, double delta, double dt, double L) {
    x += v * cos(yaw) * dt;
    y += v * sin(yaw) * dt;
    yaw += v / L * tan(delta) * dt;
    v += a * dt;
}

// PID速度控制
double PIDcontrol(double target_v, double current_v, double Kp) {
    return Kp * (target_v - current_v);
}

// 计算目标点索引
int calc_target_index(double x, double y, const vector<double>& cx, const vector<double>& cy, double Lf) {
    if (cx.empty() || cy.empty()) return 0;
    
    int n = cx.size();
    vector<double> distances(n);
    for (int i = 0; i < n; ++i) {
        distances[i] = hypot(cx[i] - x, cy[i] - y);
    }

    // 找到最近点
    int location = 0;
    double min_dist = distances[0];
    for (int i = 1; i < n; ++i) {
        if (distances[i] < min_dist) {
            min_dist = distances[i];
            location = i;
        }
    }

    // 找出前瞻距离的目标点
    int ind = location;
    while (ind < n - 1 && hypot(cx[ind] - x, cy[ind] - y) < Lf) {
        ind++;
    }

    return ind;
}

// 纯追踪控制
double pure_pursuit_control(double x, double y, double yaw, double v,
    const vector<double>& cx, const vector<double>& cy,
    int ind, double k, double Lfc, double L, double& Lf) {
    if (cx.empty() || cy.empty()) return 0.0;
    
    double tx = cx[ind];
    double ty = cy[ind];

    double alpha = atan2(ty - y, tx - x) - yaw;
    Lf = k * v + Lfc;
    return atan2(2.0 * L * sin(alpha), Lf);
}

// 生成参考路径函数
void generate_reference_path(
    const vector<double>& left_cone_x, const vector<double>& left_cone_y,
    const vector<double>& right_cone_x, const vector<double>& right_cone_y,
    vector<double>& discrete_path_x, vector<double>& discrete_path_y) 
{
    discrete_path_x.clear();
    discrete_path_y.clear();
    
    int N = left_cone_x.size();
    if (N == 0) {
        ROS_WARN("No cone data available for path generation");
        return;
    }
    
    if (right_cone_x.size() != N || right_cone_y.size() != N || left_cone_y.size() != N) {
        ROS_WARN("左右锥桶数组长度不一致！左: %d, 右: %d", (int)left_cone_y.size(), (int)right_cone_x.size());
        return;
    }

    // 1. 找到最近的右锥桶索引
    vector<int> right_closest_indices(N);
    for (int i = 0; i < N; ++i) {
        double min_dist = 1e9;
        int min_idx = 0;
        for (int j = 0; j < N; ++j) {
            double dist = hypot(right_cone_x[j] - left_cone_x[i], right_cone_y[j] - left_cone_y[i]);
            if (dist < min_dist) {
                min_dist = dist;
                min_idx = j;
            }
        }
        right_closest_indices[i] = min_idx;
    }

    // 2. 计算车道中心点
    vector<double> midpoint_x(N), midpoint_y(N);
    for (int i = 0; i < N; ++i) {
        midpoint_x[i] = (right_cone_x[right_closest_indices[i]] + left_cone_x[i]) / 2.0;
        midpoint_y[i] = (right_cone_y[right_closest_indices[i]] + left_cone_y[i]) / 2.0;
    }

    // 3. 计算路径点间距离
    vector<double> distances;
    for (int i = 0; i < N - 1; ++i) {
        distances.push_back(hypot(midpoint_x[i + 1] - midpoint_x[i], midpoint_y[i + 1] - midpoint_y[i]));
    }

    // 4. 创建路径参数
    vector<double> t;
    t.push_back(0.0);
    double sum = 0.0;
    for (double d : distances) {
        sum += d;
        t.push_back(sum);
    }

    // 5. 创建样条曲线
    Spline spline_x(t, midpoint_x);
    Spline spline_y(t, midpoint_y);

    // 6. 离散化路径
    double t_max = t.back();
    if (t_max < 1e-3) {
        ROS_WARN("Path length too short for discretization");
        return;
    }
    
    int num_points = ceil(t_max / 0.1);
    discrete_path_x.resize(num_points);
    discrete_path_y.resize(num_points);
    
    for (int i = 0; i < num_points; ++i) {
        double t_val = t_max * i / (num_points - 1);
        discrete_path_x[i] = spline_x(t_val);
        discrete_path_y[i] = spline_y(t_val);
    }
}

// 路径跟踪主函数
void tracking_main(ros::NodeHandle& nh) {
    // 控制参数
    double k = 0.1;
    double Lfc = 1.0;
    double Kp = 1.0;
    double dt = 0.1; // 对应10Hz频率
    double L = 1.2;
    double target_speed = 10.0 / 3.6; // 10 km/h -> m/s

    // 创建状态发布器
    ros::Publisher state_pub = nh.advertise<std_msgs::Float32MultiArray>("vehicle_state", 10);
    // 创建位置发布器
    ros::Publisher position_pub = nh.advertise<std_msgs::Float32MultiArray>("vehicle_position", 10);

    // 等待直到接收到锥桶数据
    while (ros::ok() && !new_data_arrived) {
        ROS_INFO_THROTTLE(1.0, "Waiting for cone data...");
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    // 局部变量存储锥桶数据
    vector<double> left_cone_x, left_cone_y;
    vector<double> right_cone_x, right_cone_y;
    
    {
        lock_guard<mutex> lock(data_mutex);
        left_cone_x = global_left_cone_x;
        left_cone_y = global_left_cone_y;
        right_cone_x = global_right_cone_x;
        right_cone_y = global_right_cone_y;
        new_data_arrived = false;
    }

    // 生成参考路径
    vector<double> discrete_path_x, discrete_path_y;
    generate_reference_path(left_cone_x, left_cone_y, right_cone_x, right_cone_y, 
                           discrete_path_x, discrete_path_y);
    
    if (discrete_path_x.empty()) {
        ROS_ERROR("Failed to generate reference path!");
        return;
    }

    // 初始化车辆状态
    double x = discrete_path_x[0];
    double y = discrete_path_y[0];
    double yaw = 0.0;
    double v = 0.0;
    double time = 0.0;
    double Lf = k * v + Lfc;

    ROS_INFO("Starting path tracking with %d path points", (int)discrete_path_x.size());
    ROS_INFO("Initial position: (%.2f, %.2f)", x, y);

    // 主控制循环
    ros::Rate rate(1.0/dt); // 10Hz频率
    while (ros::ok()) {
        // 复制当前路径数据（线程安全）
        vector<double> current_path_x, current_path_y;
        {
            lock_guard<mutex> lock(path_mutex);
            current_path_x = discrete_path_x;
            current_path_y = discrete_path_y;
        }

        // 检查是否有新数据到达
        if (new_data_arrived) {
            vector<double> new_left_x, new_left_y, new_right_x, new_right_y;
            {
                lock_guard<mutex> lock(data_mutex);
                new_left_x = global_left_cone_x;
                new_left_y = global_left_cone_y;
                new_right_x = global_right_cone_x;
                new_right_y = global_right_cone_y;
                new_data_arrived = false;
            }
            
            // 生成新路径
            vector<double> new_path_x, new_path_y;
            generate_reference_path(new_left_x, new_left_y, new_right_x, new_right_y, 
                                   new_path_x, new_path_y);
            
            // 安全更新路径
            if (!new_path_x.empty()) {
                lock_guard<mutex> lock(path_mutex);
                discrete_path_x = new_path_x;
                discrete_path_y = new_path_y;
                current_path_x = discrete_path_x;
                current_path_y = discrete_path_y;
                ROS_INFO("Updated reference path with new cone data");
            } else {
                ROS_WARN("New path generation failed, keeping old path");
            }
        }

        // 检查路径是否有效
        if (current_path_x.empty() || current_path_y.empty()) {
            ROS_WARN_THROTTLE(1.0, "Empty path, skipping control cycle");
            rate.sleep();
            continue;
        }

        // 计算控制量
        double ai = PIDcontrol(target_speed, v, Kp);
        int target_ind = calc_target_index(x, y, current_path_x, current_path_y, Lf);
        double target_delta = pure_pursuit_control(x, y, yaw, v, current_path_x, current_path_y,
            target_ind, k, Lfc, L, Lf);

        // 更新车辆状态
        update(x, y, yaw, v, ai, target_delta, dt, L);

        // 发布车辆状态（目标转向角和速度）
        std_msgs::Float32MultiArray state_msg;
        state_msg.data.clear();
        state_msg.data.push_back(static_cast<float>(target_delta)); // 目标转向角（弧度）
        state_msg.data.push_back(static_cast<float>(v));            // 速度（m/s）
        state_pub.publish(state_msg);

        // 发布车辆位置（x和y）
        std_msgs::Float32MultiArray position_msg;
        position_msg.data.clear();
        position_msg.data.push_back(static_cast<float>(x)); // X坐标
        position_msg.data.push_back(static_cast<float>(y)); // Y坐标
        position_pub.publish(position_msg);

        // 输出信息
        double target_delta_deg = target_delta * 180.0 / PI;
        ROS_INFO_THROTTLE(0.5, "Time: %.1fs | Position: (%.2f, %.2f) | Speed: %.2fm/s | Yaw: %.2frad | Steering: %.1fdeg", 
                         time, x, y, v, yaw, target_delta_deg);

        time += dt;
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "path_tracking_node");
    ros::NodeHandle nh;
    
    // 创建订阅器
    ros::Subscriber sub = nh.subscribe("cone_data", 10, coneDataCallback);
    
    // 在单独线程中运行路径跟踪
    thread tracking_thread(tracking_main, ref(nh));
    
    // 主线程处理ROS回调
    ros::spin();
    
    // 等待跟踪线程结束
    if (tracking_thread.joinable()) {
        tracking_thread.join();
    }
    
    return 0;
}
