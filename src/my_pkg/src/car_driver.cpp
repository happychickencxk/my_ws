#include "car_driver.h"
// ++ sudo chmod 666 /dev/ttyTHS1
// 构造函数实现
car_driver::car_driver() 
    : running_(false), serial_fd_(-1), first_init_(false), last_valid_packet_(ros::Time(0)),
      // 添加odom相关成员的初始化
      x_pos_(0.0), y_pos_(0.0), theta_(0.0), first_theta_(0.0), first_odom_(true),
      odom_broadcaster_(),  // tf广播器初始化
      connect_state(false),  // 添加缺失的成员初始化
      running(false)        // 添加缺失的成员初始化
{}

car_driver::~car_driver() {
    // 1. 停止接收线程
    running_ = false;

    // 2. 等待线程结束
    if(read_thread_.joinable()) {
        try {
            read_thread_.join();
        } 
        catch(const std::system_error& e) {
            ROS_ERROR("Thread join failed: %s", e.what());
        }
    }

    // 3. 关闭串口
    if(serial_fd_ != -1) {
        if(close(serial_fd_) < 0) {
            ROS_ERROR("Close serial failed: %s", strerror(errno));
        } else {
            ROS_DEBUG("Serial port closed");
        }
        serial_fd_ = -1;
    }

    // 4. 打印析构日志
    ROS_DEBUG("car_driver destroyed");
}

bool car_driver::init(){
    // 清空数据结构
    memset(&imuData, 0, sizeof(imu_data));
    memset(&gpsData, 0, sizeof(gps_data));
    
    // 打开串口
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        ROS_ERROR("Open Serial failed: %s", strerror(errno));
        return false;
    }

    // 获取当前配置
    if(tcgetattr(serial_fd_, &options) < 0){
        ROS_ERROR("tcgetattr failed: %s", strerror(errno));
        close(serial_fd_);
        return false;
    }

    // 配置波特率
    const speed_t baud = [&](){
        switch(baudrate){
            case 921600: return B921600;
            case 115200: return B115200;
            default:     return B115200;
        }
    }();
    
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    // 8N1模式
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;

    // 禁用流控和回显
    options.c_cflag &= ~CRTSCTS;
    options.c_lflag &= ~(ICANON | ECHO | ECHONL | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
    options.c_oflag &= ~OPOST;

    // 立即应用配置
    if(tcsetattr(serial_fd_, TCSANOW, &options) < 0){
        ROS_FATAL("tcsetattr failed: %s", strerror(errno));
        close(serial_fd_);
        return false;
    }

    ROS_INFO("Serial port initialized at %d baud", baudrate);
    return true;
}

void car_driver::loop(){
    ros::NodeHandle nh;
    ros::NodeHandle nh_p_("~");

    nh_p_.param<std::string>("odom_frame", odom_frame_, "odom");
    nh_p_.param<std::string>("base_frame", base_frame_, "base_link");
    nh_p_.param<std::string>("camera_frame", camera_frame_, "camera_link");
    nh_p_.param<std::string>("imu_frame", imu_frame_, "imu_link");
    nh_p_.param<std::string>("port_name_", port_name_, "/dev/ttyTHS1");
    nh_p_.param<int>("baud_rate", baudrate, 115200);

    nh_p_.param<float>("Kp",pidData.p,0.017);
    nh_p_.param<float>("Ki",pidData.i,0);
    nh_p_.param<float>("Kd",pidData.d,0.2);
    nh_p_.param<int>("servo_bias",servo_bias_,0);

    velData.p_ang=0;velData.p_vel=0;set_vel(&velData);
    if(!init()) {
        ROS_ERROR("Driver initialization failed!");
        return;
    }
    // 初始化发布者以及odom坐标广播者
    odom_msg_.header.frame_id = odom_frame_;
    odom_msg_.child_frame_id = base_frame_;
    // 初始化协方差矩阵
    for(int i = 0; i < 36; i++) {
        if(i == 0 || i == 7 || i == 14) // x, y, z 位置方差
            odom_msg_.pose.covariance[i] = 0.1;
        else if(i == 21 || i == 28 || i == 35) // 旋转方差
            odom_msg_.pose.covariance[i] = 0.05;
        else
            odom_msg_.pose.covariance[i] = 0.0;
            
        if(i == 0 || i == 7 || i == 14) // 线速度方差
            odom_msg_.twist.covariance[i] = 0.1;
        else if(i == 21 || i == 28 || i == 35) // 角速度方差  
            odom_msg_.twist.covariance[i] = 0.05;
        else
            odom_msg_.twist.covariance[i] = 0.0;
    }

    imu_pub_ = nh.advertise<sensor_msgs::Imu>("Imu", 10);
    gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("Gps", 5);
    imu_battery_pub_ = nh.advertise<std_msgs::Float32>("Imu_adc", 1);
    pose_pub_ = nh.advertise<geometry_msgs::Pose>("robot_pose", 10);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
    act_vel_pub_ = nh.advertise<geometry_msgs::Twist>("raw_act_vel",10);
    pur_vel_pub_ = nh.advertise<geometry_msgs::Twist>("pur_act_vel",10);

    check_connent();

    //向单片机发送配置参数(PID)
    set_pid(&pidData);
    ros::Duration(0.02).sleep();

    // 启动接收线程
    running_ = true;
    read_thread_ = std::thread(&car_driver::read_thread_func, this);

    // 订阅控制指令
    cmd_sub_ = nh.subscribe("cmd_vel", 1, &car_driver::cmd_vel_cb, this);

    dynamic_reconfigure::Server<my_pkg::PID_reconfigConfig> reconfig_server;
    dynamic_reconfigure::Server<my_pkg::PID_reconfigConfig>::CallbackType f;

    f = boost::bind(&car_driver::dynamic_reconfig_callback,this,_1,_2);
    reconfig_server.setCallback(f);

    // 主循环仅处理ROS回调
    ros::Rate rate(30);
    ROS_INFO("Robot Running!");
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    // 清理
    running_ = false;
    if(read_thread_.joinable()) read_thread_.join();
    close(serial_fd_);
}

void car_driver::read_thread_func() {
    uint8_t buf[BUFFER_SIZE];          // 创建数据缓冲区
    while(running_) {                  // 循环检查运行标志
        // 1. 设置文件描述符监控
        fd_set set;                    // 定义文件描述符集合
        struct timeval timeout {       // 设置1秒超时
            .tv_sec = 0,               // 秒
            .tv_usec = 40000               // 微秒
        }; 
        
        FD_ZERO(&set);                 // 清空集合
        FD_SET(serial_fd_, &set);      // 添加串口文件描述符到集合
        
        // 2. 等待数据到达（带超时）
        int ret = select(
            serial_fd_ + 1,            // 监控的最大fd+1
            &set,                      // 读集合
            nullptr,                   // 写集合（不监控）
            nullptr,                   // 异常集合（不监控）
            &timeout                   // 超时时间
        );
        
        // 3. 处理select结果
        if(ret < 0) {                  // 错误处理
            ROS_ERROR("Select error: %s", strerror(errno));
            break;
        }
        if(ret == 0) {                 // 超时处理
            continue;                  // 重新检查running_状态
        }
        
        // 4. 读取数据
        ssize_t n = read(serial_fd_, buf, sizeof(buf));
        if(n > 0) {
            // std::stringstream ss;
            // ss<<"n:"<<static_cast<int>(n);
            // ROS_INFO("%s",ss.str().c_str());
            process_raw_data(buf, n);  // 处理数据（你的业务逻辑）
        }
    }
}

void car_driver::monitor_thread() {
    while(running_) {
        if((ros::Time::now() - last_valid_packet_).toSec() > 2.0) {
            ROS_ERROR("No valid packet in 2 seconds!");
            // 执行复位操作...
        }
        sleep(1);
    }
}

void car_driver::process_raw_data(const uint8_t* data, size_t len) {
    static std::vector<uint8_t> pkt_buffer;
    static bool in_packet = false;

    for(size_t i=0; i<len; ++i){
        if(!in_packet && data[i] == pack_begin){
            pkt_buffer.clear();
            pkt_buffer.push_back(data[i]);
            in_packet = true;
        }
        else if(in_packet){
            pkt_buffer.push_back(data[i]);

            // 包完整性检查
            if(pkt_buffer.size() >= 5 && pkt_buffer.back() == pack_end)
            {
                if(validate_packet(pkt_buffer)) {
                    in_packet = false;//验证成功后清除该数据
                    parse_packet(pkt_buffer);
                } 
                // else {
                //     // 将无效包转为HEX字符串
                //     std::stringstream ss;
                //     ss << "Invalid packet: ";
                //     for(uint8_t b : pkt_buffer) {
                //         ss << std::hex << std::setw(2) << std::setfill('0') 
                //            << static_cast<int>(b) << " ";
                //     }
                //     ROS_WARN("%s", ss.str().c_str());
                // }
            }
            // 防止缓冲区溢出
            else if(pkt_buffer.size() > BUFFER_SIZE){
                std::stringstream ss;
                ss << "Buffer overflow before packet end. Current: ";
                for(uint8_t b : pkt_buffer) {
                    ss << std::hex << std::setw(2) << std::setfill('0') 
                       << static_cast<int>(b) << " ";
                }
                ROS_WARN("%s", ss.str().c_str());
                in_packet = false;
            }
        }
    }
}

bool car_driver::validate_packet(const std::vector<uint8_t>& pkt) {
    if(pkt.size() < 5) return false;
    
    // 校验和验证
    const uint8_t expect_sum = std::accumulate(
        pkt.begin()+1, pkt.end()-2, 0);

        // if(expect_sum != pkt[pkt.size()-2]) {
        //     // 将数值转换为int防止被当作char输出
        //     const int calc = static_cast<int>(expect_sum);
        //     const int pkt_sum = static_cast<int>(pkt[pkt.size()-2]);
            
        //     ROS_WARN_STREAM("validate error! The calculate number is 0x"
        //         << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << calc
        //         << ", but the packet number is 0x"
        //         << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << pkt_sum);
        // }
    
    return (expect_sum == pkt[pkt.size()-2]) && 
           (pkt[0] == pack_begin) && 
           (pkt.back() == pack_end);
}

void car_driver::parse_packet(const std::vector<uint8_t>& pkt) {
    // 基础校验（已在 validate_packet 中完成）
    last_valid_packet_ = ros::Time::now();
    const uint8_t func_code = pkt[3]; // 协议结构：0x0F(1B) + 地址(1B) + 长度(1B) + 功能码(1B) + 数据(nB) + 校验(1B) + 0x4D(1B)
    const uint8_t data_length = pkt[2]; // 长度码 = 功能码(1B) + 数据长度

    switch(func_code) {
    case ID_CONNECT:
        ROS_INFO("connect to mcu!");
        connect_state = true;
        break;

    case ID_IMU:
        ROS_DEBUG("Get Imu data!");
        parse_IMU(pkt.data() + 4, data_length-1); // 数据起始位置
        break;

    case ID_GPS: {
        ROS_DEBUG("Get GPS data!");
        if(data_length < 24+1) { // 3个double(8*3=24字节)
            ROS_WARN("Invalid GPS packet length");
            return;
        }
        sensor_msgs::NavSatFix gps_msg;
        gps_msg.header.stamp = ros::Time::now();
        gps_msg.header.frame_id = gps_frame_;

        // 直接通过内存映射解析
        std::memcpy(&gpsData.longitude, pkt.data() + 4, 8);
        std::memcpy(&gpsData.latitude, pkt.data() + 12, 8);
        std::memcpy(&gpsData.altitude, pkt.data() + 20, 8);

        gps_msg.longitude = gpsData.longitude;
        gps_msg.latitude = gpsData.latitude;
        gps_msg.altitude = gpsData.altitude;
        gps_pub_.publish(gps_msg);
        break;
    }
    case ID_ACT_VEL:{
        if(data_length < 16+1) {
            ROS_WARN("Invalid actual speed packet length");
            return;
        }
        ROS_DEBUG("Get actual speed data!");
        geometry_msgs::Twist act_vel_msg;

        std::memcpy(&act_vel_msg.linear.x, pkt.data() + 4, 8);
        std::memcpy(&act_vel_msg.angular.z, pkt.data() + 12, 8);

        act_vel_pub_.publish(act_vel_msg);

        //使用速度信息更新odom
        update_odometry(act_vel_msg);
        publish_odom_tf(act_vel_msg);
        break;
    }
    case ID_ADC:{ // 修正分号为冒号
        if(data_length < 4+1) {
            ROS_WARN("Invalid ADC packet length");
            return;
        }

        std_msgs::Float32 msg;
        std::memcpy(&adcData, pkt.data() + 4, 4);
        msg.data = adcData;
        adc_pub_.publish(msg);
        break;
    
    }
    default:
        ROS_DEBUG("Unknown function code: 0x%02X", func_code);
        break;
    }
}

void car_driver::parse_IMU(const uint8_t* buf,const uint8_t DLen){
    uint16_t ctl;
    uint8_t L;
    uint8_t tmpU8;
    uint16_t tmpU16;
    uint32_t tmpU32;
    float tmpX, tmpY, tmpZ, tmpAbs;

    #ifdef Debug_IMU_
    std::stringstream ss;
    ss << "IMU packet: ";
    for(uint8_t b=0; b<DLen; b++) {
        ss << std::hex << std::setw(2) << std::setfill('0') 
            << static_cast<int>(buf[b]) << " ";
    }
    ROS_DEBUG("%s", ss.str().c_str());
    #endif

    switch (buf[0])
    {
    case 0x02: // 传感器 已睡眠 回复
        Dbp("\t sensor off success\r\n");
        break;
    case 0x03: // 传感器 已唤醒 回复
        Dbp("\t sensor on success\r\n");
        break;
    case 0x32: // 磁力计 开始校准 回复
        Dbp("\t compass calibrate begin\r\n");
        break;
    case 0x04: // 磁力计 结束校准 回复
        Dbp("\t compass calibrate end\r\n");
        break;
    case 0x05: // z轴角 已归零 回复
        Dbp("\t z-axes to zero success\r\n");
        break;
    case 0x06: // 请求 xyz世界坐标系清零 回复
        Dbp("\t WorldXYZ-axes to zero success\r\n");
        break;
    case 0x07: // 加速计简单校准正在进行，将在9秒后完成  回复
        Dbp("\t acceleration calibration, Hold still for 9 seconds\r\n");
        break;
    case 0x08: // 恢复默认的自身坐标系Z轴指向及恢复默认的世界坐标系  回复
        Dbp("\t axesZ WorldXYZ-axes to zero success\r\n");
        break;
    case 0x10: // 模块当前的属性和状态 回复
        Dbp("\t still limit: %u\r\n", buf[1]);   // 字节1 惯导-静止状态加速度阀值 单位dm/s?
        Dbp("\t still to zero: %u\r\n", buf[2]); // 字节2 惯导-静止归零速度(单位mm/s) 0:不归零 255:立即归零
        Dbp("\t move to zero: %u\r\n", buf[3]);  // 字节3 惯导-动态归零速度(单位mm/s) 0:不归零
        Dbp("\t compass: %s\r\n", ((buf[4]>>0) & 0x01)? "on":"off" );     // 字节4 bit[0]: 1=已开启磁场 0=已关闭磁场
        Dbp("\t barometer filter: %u\r\n", (buf[4]>>1) & 0x03);           // 字节4 bit[1-2]: 气压计的滤波等级[取值0-3],数值越大越平稳但实时性越差
        Dbp("\t IMU: %s\r\n", ((buf[4]>>3) & 0x01)? "on":"off" );         // 字节4 bit[3]: 1=传感器已开启  0=传感器已睡眠
        Dbp("\t auto report: %s\r\n", ((buf[4]>>4) & 0x01)? "on":"off" ); // 字节4 bit[4]: 1=已开启传感器数据主动上报 0=已关闭传感器数据主动上报
        Dbp("\t FPS: %u\r\n", buf[5]); // 字节5 数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
        Dbp("\t gyro filter: %u\r\n", buf[6]);    // 字节6 陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
        Dbp("\t acc filter: %u\r\n", buf[7]);     // 字节7 加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
        Dbp("\t compass filter: %u\r\n", buf[8]); // 字节8 磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
        Dbp("\t subscribe tag: 0x%04X\r\n", (U16)(((U16)buf[10]<<8) | buf[9])); // 字节[10-9] 功能订阅标识
        Dbp("\t charged state: %u\r\n", buf[11]); // 字节11 充电状态指示 0=未接电源 1=充电中 2=已充满
        Dbp("\t battery level: %u%%\r\n", buf[12]); // 字节12 当前剩余电量[0-100%]
        Dbp("\t battery voltage: %u mv\r\n", (U16)(((U16)buf[14]<<8) | buf[13])); // 字节[14-13] 电池的当前电压mv
        Dbp("\t Mac: %02X:%02X:%02X:%02X:%02X:%02X\r\n", buf[15],buf[16],buf[17],buf[18],buf[19],buf[20]); // 字节[15-20] MAC地址
        Dbp("\t version: %s\r\n", &buf[21]); // 字节[21-26] 固件版本 字符串
        Dbp("\t product model: %s\r\n", &buf[27]); // 字节[26-32] 产品型号 字符串
        break;
    case 0x11: // 获取订阅的功能数据 回复或主动上报
        ctl = ((U16)buf[2] << 8) | buf[1];// 字节[2-1] 为功能订阅标识，指示当前订阅了哪些功能
        Dbp("\t subscribe tag: 0x%04X\r\n", ctl);
		tmpU32=(U32)(((U32)buf[6]<<24) | ((U32)buf[5]<<16) | ((U32)buf[4]<<8) | ((U32)buf[3]<<0));	
        Dbp("\t ms: %lu\r\n",tmpU32); // 字节[6-3] 为模块开机后的时间戳(单位ms)
		imuData.timestamp=tmpU32;
        L =7; // 从第7字节开始根据 订阅标识tag来解析剩下的数据
        if ((ctl & 0x0001) != 0)
        {// 加速度xyz 去掉了重力 使用时需*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\taX: %.3f\r\n", tmpX); // x加速度aX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\taY: %.3f\r\n", tmpY); // y加速度aY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\taZ: %.3f\r\n", tmpZ); // z加速度aZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); Dbp("\ta_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值
			imuData.ax=tmpX;imuData.ay=tmpY;imuData.az=tmpZ;

        }
        if ((ctl & 0x0002) != 0)
        {// 加速度xyz 包含了重力 使用时需*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\tAX: %.3f\r\n", tmpX); // x加速度AX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\tAY: %.3f\r\n", tmpY); // y加速度AY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\tAZ: %.3f\r\n", tmpZ); // z加速度AZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); Dbp("\tA_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值
			imuData.AX=tmpX;imuData.AY=tmpY;imuData.AZ=tmpZ;
        }
        if ((ctl & 0x0004) != 0)
        {// 角速度xyz 使用时需*scaleAngleSpeed °/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; Dbp("\tGX: %.3f\r\n", tmpX); // x角速度GX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; Dbp("\tGY: %.3f\r\n", tmpY); // y角速度GY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; Dbp("\tGZ: %.3f\r\n", tmpZ); // z角速度GZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); Dbp("\tG_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值
			imuData.GX=tmpX;imuData.GY=tmpY;imuData.GZ=tmpZ;
        }
        if ((ctl & 0x0008) != 0)
        {// 磁场xyz 使用时需*scaleMag uT
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; Dbp("\tCX: %.3f\r\n", tmpX); // x磁场CX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; Dbp("\tCY: %.3f\r\n", tmpY); // y磁场CY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; Dbp("\tCZ: %.3f\r\n", tmpZ); // z磁场CZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); Dbp("\tC_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值
			imuData.CX=tmpX;imuData.CY=tmpY;imuData.CZ=tmpZ;
		}
        if ((ctl & 0x0010) != 0)
        {// 温度 气压 高度
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleTemperature; L += 2; Dbp("\ttemperature: %.2f\r\n", tmpX); // 温度

            tmpU32 = (U32)(((U32)buf[L+2] << 16) | ((U32)buf[L+1] << 8) | (U32)buf[L]);
            tmpU32 = ((tmpU32 & 0x800000) == 0x800000)? (tmpU32 | 0xff000000) : tmpU32;// 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
            tmpY = (S32)tmpU32 * scaleAirPressure; L += 3; Dbp("\tairPressure: %.3f\r\n", tmpY); // 气压

            tmpU32 = (U32)(((U32)buf[L+2] << 16) | ((U32)buf[L+1] << 8) | (U32)buf[L]);
            tmpU32 = ((tmpU32 & 0x800000) == 0x800000)? (tmpU32 | 0xff000000) : tmpU32;// 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
            tmpZ = (S32)tmpU32 * scaleHeight; L += 3; Dbp("\theight: %.3f\r\n", tmpZ); // 高度
			imuData.temperature=tmpX;imuData.airPressure=tmpY;imuData.height=tmpZ;
		}
        if ((ctl & 0x0020) != 0)
        {// 四元素 wxyz 使用时需*scaleQuat
            tmpAbs = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; Dbp("\tw: %.3f\r\n", tmpAbs); // w
            tmpX =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; Dbp("\tx: %.3f\r\n", tmpX); // x
            tmpY =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; Dbp("\ty: %.3f\r\n", tmpY); // y
            tmpZ =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; Dbp("\tz: %.3f\r\n", tmpZ); // z
        	imuData.tw=tmpAbs;imuData.tx=tmpX;imuData.ty=tmpY;imuData.tz=tmpZ;
		}
        if ((ctl & 0x0040) != 0)
        {// 欧拉角xyz 使用时需*scaleAngle
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; Dbp("\tangleX: %.3f\r\n", tmpX); // x角度
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; Dbp("\tangleY: %.3f\r\n", tmpY); // y角度
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; Dbp("\tangleZ: %.3f\r\n", tmpZ); // z角度
			imuData.tangleX=tmpX;imuData.tangleY=tmpY;imuData.tangleZ=tmpZ;
        }
        if ((ctl & 0x0080) != 0)
        {// xyz 空间位移 单位mm 转为 m
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; Dbp("\toffsetX: %.3f\r\n", tmpX); // x坐标
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; Dbp("\toffsetY: %.3f\r\n", tmpY); // y坐标
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; Dbp("\toffsetZ: %.3f\r\n", tmpZ); // z坐标
			imuData.offsetX=tmpX;imuData.offsetY=tmpY;imuData.offsetZ=tmpZ;
		}
        if ((ctl & 0x0100) != 0)
        {// 活动检测数据
            tmpU32 = (U32)(((U32)buf[L+3]<<24) | ((U32)buf[L+2]<<16) | ((U32)buf[L+1]<<8) | ((U32)buf[L]<<0)); L += 4; Dbp("\tsteps: %u\r\n", tmpU32); // 计步数
            tmpU8 = buf[L]; L += 1;
            Dbp("\t walking: %s\r\n", (tmpU8 & 0x01)?  "yes" : "no"); // 是否在走路
            Dbp("\t running: %s\r\n", (tmpU8 & 0x02)?  "yes" : "no"); // 是否在跑步
            Dbp("\t biking: %s\r\n",  (tmpU8 & 0x04)?  "yes" : "no"); // 是否在骑车
            Dbp("\t driving: %s\r\n", (tmpU8 & 0x08)?  "yes" : "no"); // 是否在开车
        }
        if ((ctl & 0x0200) != 0)
        {// 加速度xyz 去掉了重力 使用时需*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\tasX: %.3f\r\n", tmpX); // x加速度asX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\tasY: %.3f\r\n", tmpY); // y加速度asY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; Dbp("\tasZ: %.3f\r\n", tmpZ); // z加速度asZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); Dbp("\tas_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值
			imuData.tasX=tmpX;imuData.tasY=tmpY;imuData.tasZ=tmpZ;
		}
        if ((ctl & 0x0400) != 0)
        {// ADC的值
            tmpU16 = (U16)(((U16)buf[L+1]<<8) | ((U16)buf[L]<<0)); L += 2; Dbp("\tadc: %u\r\n", tmpU16); // 单位mv
			imuData.adc=tmpU16;
		}
        if ((ctl & 0x0800) != 0)
        {// GPIO1的值
            tmpU8 = buf[L]; L += 1;
            Dbp("\t GPIO1  M:%X, N:%X\r\n", (tmpU8>>4)&0x0f, (tmpU8)&0x0f);
			imuData.GPIO1=tmpU8;
        }
        pub_imu();
        pub_imu_adc();
        ROS_DEBUG("PUB IMU!");
        break;
    case 0x12: // 设置参数 回复
        Dbp("\t set parameters success\r\n");
        break;
    case 0x13: // 惯导三维空间位置清零 回复
        Dbp("\t clear INS position success\r\n");
        break;
    case 0x14: // 恢复出厂校准参数 回复
        Dbp("\t Restore calibration parameters from factory mode success\r\n");
        break;
    case 0x15: // 保存当前校准参数为出厂校准参数 回复
        Dbp("\t Save calibration parameters to factory mode success\r\n");
        break;
    case 0x16: // 计步数清零 回复
        Dbp("\t clear steps success\r\n");
        break;
    case 0x17: // 加速计高精度校准 回复
        if (buf[1] == 255)
        {// 字节1 值255 表示采集完成，正在结束校准(设备需继续保持静止等待10秒钟)
            Dbp("\t calibration success, please wait 10 seconds\r\n");
        }
        else if (buf[1] == 254)
        {// 字节1 值254 表示陀螺仪自检失败
            Dbp("\t calibration fail, gyro error\r\n");
        }
        else if (buf[1] == 253)
        {// 字节1 值253 表示加速计自检失败
            Dbp("\t calibration fail, accelerometer error\r\n");
        }
        else if (buf[1] == 252)
        {// 字节1 值252 表示磁力计自检失败
            Dbp("\t calibration fail, compass error\r\n");
        }
        else if (buf[1] == 251)
        {// 字节1 值251 表示设备未在校准中
            Dbp("\t calibration fail, Hasn't started\r\n");
        }
        else if (buf[1] != 0)
        {// 值[1-250] 表示当前已采集的次数
            Dbp("\t calibration, Points collected is %u\r\n", buf[1]);
        }
        else
        {// 值0 表示模块已经在校准中
            Dbp("\t calibration is running\r\n");
        }
        break;
    case 0x18: // 已关闭主动上报 回复
        Dbp("\t auto report off\r\n");
        break;
    case 0x19: // 已打开主动上报 回复
        Dbp("\t auto report on\r\n");
        break;
    case 0x20: // 设置PCB安装方向矩阵 回复
        Dbp("\t set PCB direction success\r\n");
        break;
    case 0x21: // 是请求 读取安装方向矩阵
        // Dbp_U8_buf("\t get PCB direction: 0x[", "]\r\n",
        //            "%02x ",
        //            &buf[1], 9); // 字节[1-9]     为加速计安装方向矩阵
        // Dbp_U8_buf("\t get PCB direction: 0x[", "]\r\n",
        //            "%02x ",
        //            &buf[10], 9); // 字节[10-18] 为磁力计安装方向矩阵
        break;
    case 0x22: // 是请求 设置蓝牙广播名称
        Dbp("\t set BLE name success\r\n");
        break;
    case 0x23: // 读取蓝牙广播名称 回复
        Dbp("\t get BLE name: %s\r\n", &buf[1]); // 字节[1-16] 为蓝牙广播名称字符串
        break;
    case 0x24: // 设置关机电压和充电参数 回复
        Dbp("\t set PowerDownVoltage and charge parameters success\r\n");
        break;
    case 0x25: // 读取关机电压和充电参数 回复
        Dbp("\t PowerDownVoltageFlag: %u\r\n", buf[1]); // 字节1 关机电压选择标志 0表示3.4V, 1表示2.7V
        Dbp("\t charge_full_mV: %u\r\n", buf[2]); // 字节2 充电截止电压 0:3962mv 1:4002mv 2:4044mv 3:4086mv 4:4130mv 5:4175mv 6:4222mv 7:4270mv 8:4308mv 9:4349mv 10:4391mv
        Dbp("\t charge_full_mA: %u ma\r\n", buf[3]); // 字节3 充电截止电流 0:2ma 1:5ma 2:7ma 3:10ma 4:15ma 5:20ma 6:25ma 7:30ma
        Dbp("\t charge_mA: %u ma\r\n", buf[4]); // 字节3 充电电流 0:20ma 1:30ma 2:40ma 3:50ma 4:60ma 5:70ma 6:80ma 7:90ma 8:100ma 9:110ma 10:120ma 11:140ma 12:160ma 13:180ma 14:200ma 15:220ma
        break;
    case 0x27: // 设置用户的GPIO引脚 回复
        Dbp("\t set gpio success\r\n");
        break;
    case 0x28: // 设置Z轴角度为指定值 回复
        Dbp("\t set AngleZ success\r\n");
        break;
    case 0x2A: // 重启设备 回复
        Dbp("\t will reset\r\n");
        break;
    case 0x2B: // 设备关机 回复
        Dbp("\t will power off\r\n");
        break;
    case 0x2C: // 设置空闲关机时长 回复
        Dbp("\t set idleToPowerOffTime success\r\n");
        break;
    case 0x2D: // 读取空闲关机时长 回复
        Dbp("\t idleToPowerOffTime:%u minutes\r\n", buf[1]*10);
        break;
    case 0x2E: // 设置禁止蓝牙方式更改名称和充电参数标识 回复
        Dbp("\t set FlagForDisableBleSetNameAndCahrge success\r\n");
        break;
    case 0x2F: // 读取禁止蓝牙方式更改名称和充电参数标识 回复
        Dbp("\t FlagForDisableBleSetNameAndCahrge:%u\r\n", buf[1]);
        break;
    case 0x30: // 设置串口通信地址 回复
        Dbp("\t set address success\r\n");
        break;
    case 0x31: // 读取串口通信地址 回复
        Dbp("\t address:%u\r\n", buf[1]);
        break;
    case 0x33: // 设置加速计和陀螺仪量程 回复
        Dbp("\t set accelRange and gyroRange success\r\n");
        break;
    case 0x34: // 读取加速计和陀螺仪量程 回复
        Dbp("\t accelRange:%u gyroRange:%u\r\n", buf[1], buf[2]);
        break;
    case 0x35: // 设置陀螺仪自动校正标识 回复
        Dbp("\t set GyroAutoFlag success\r\n");
        break;
    case 0x36: // 读取陀螺仪自动校正标识 回复
        Dbp("\t GyroAutoFlag:%u\r\n", buf[1]);
        break;
    case 0x37: // 设置静止节能模式的触发时长 回复
        Dbp("\t set EcoTime success\r\n");
        break;
    case 0x38: // 读取静止节能模式的触发时长 回复
        Dbp("\t EcoTime:%u\r\n", buf[1]);
        break;
    case 0x40: // 设置开机后工作模式 回复
        Dbp("\t set WorkMode success\r\n");
        break;
    case 0x41: // 读取开机后工作模式 回复
        Dbp("\t WorkMode:%u\r\n", buf[1]);
        break;		
    case 0x42: // 设置高度为指定值 回复
        Dbp("\t set Height success\r\n");
        break;
    case 0x43: // 设置自动补偿高度标识 回复
        Dbp("\t set HeightAutoFlag success\r\n");
        break;
    case 0x44: // 读取自动补偿高度标识 回复
        Dbp("\t HeightAutoFlag:%u\r\n", buf[1]);
        break;
    case 0x47: // 设置串口波特率 回复
        Dbp("\t set BaudRate success\r\n");
        break;
    case 0x48: // 读取串口波特率 回复
        Dbp("\t BaudRate:%u\r\n", buf[1]);
        break;
    case 0x50: // 是透传过来的数据 回复  把透传数据以十六进制打印出来
        // Dbp_U8_buf("DTU: ", "\r\n",
        //    "%02X ",
        //    &buf[1], DLen-1);
        break;
    case 0x51: // 设置用圈数代替欧拉角传输 回复
        Dbp("\t set Cycle success\r\n");
        break;

    default:
        break;
    }
}

void car_driver::pub_imu(){
    Imu_pub_data.header.frame_id = imu_frame_;
    Imu_pub_data.header.stamp = ros::Time::now();
    Imu_pub_data.orientation.x=imuData.tx;
    Imu_pub_data.orientation.y=imuData.ty;
    Imu_pub_data.orientation.z=imuData.tz;
    Imu_pub_data.orientation.w=imuData.tw;
    Imu_pub_data.angular_velocity.x=imuData.GX;
    Imu_pub_data.angular_velocity.y=imuData.GY;
    Imu_pub_data.angular_velocity.z=imuData.GZ;
    Imu_pub_data.linear_acceleration.x=imuData.ax;
    Imu_pub_data.linear_acceleration.y=imuData.ay;
    Imu_pub_data.linear_acceleration.z=imuData.az;
    Imu_pub_data.orientation_covariance = {0, 0, 0,
        0, 0, 0,
        0, 0, 0.0};
    Imu_pub_data.angular_velocity_covariance = {0, 0, 0,
             0, 0, 0,
             0, 0, 0};
    Imu_pub_data.linear_acceleration_covariance = {0, 0, 0,
                 0, 0, 0,
                 0, 0, 0};
    imu_pub_.publish(Imu_pub_data);
}

void car_driver::pub_pose(){
    // 使用odometry计算的位置 + IMU的朝向，发布完整的位姿信息
    pose_pub_data.position.x = x_pos_;
    pose_pub_data.position.y = y_pos_;
    pose_pub_data.position.z = 0;  // 对于地面机器人，Z轴通常为0
    
    // 使用IMU的四元数作为朝向（比欧拉角更稳定）
    pose_pub_data.orientation.w = imuData.tw;
    pose_pub_data.orientation.x = imuData.tx;
    pose_pub_data.orientation.y = imuData.ty;
    pose_pub_data.orientation.z = imuData.tz;
    
    pose_pub_.publish(pose_pub_data);
    
    // 可选：添加调试输出
    // ROS_DEBUG_THROTTLE(5.0, "Published pose - Position: (%.3f, %.3f, %.3f), Orientation: (%.3f, %.3f, %.3f, %.3f)", 
    //                   pose_pub_data.position.x, pose_pub_data.position.y, pose_pub_data.position.z,
    //                   pose_pub_data.orientation.x, pose_pub_data.orientation.y, 
    //                   pose_pub_data.orientation.z, pose_pub_data.orientation.w);
}


void car_driver::pub_gps(gps_data* data){
    sensor_msgs::NavSatFix gps_pub_data;
    gps_pub_data.header.stamp=ros::Time::now();
    gps_pub_data.header.frame_id=gps_frame_;
    gps_pub_data.latitude=data->latitude;
    gps_pub_data.longitude=data->longitude;
    gps_pub_data.altitude=data->altitude;
    gps_pub_.publish(gps_pub_data);
}

void car_driver::pub_imu_adc(){
    Imu_adc_pub_data.data=(float)imuData.adc/2788.0;
    imu_battery_pub_.publish(Imu_adc_pub_data);
}

void car_driver::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg) {
    vel_data vdata{msg->linear.x, msg->angular.z};
    set_vel(&vdata);
    //弧度制转角度,以及路径拟合算法左为正,car_driver中左为负
    float tmp=-(msg->angular.z)*M_PI/180.0;
    servo_data a{tmp};
    set_servo_angle(a);
}

void car_driver::check_connent(){
    uint8_t buffer[2];
    buffer[0]=ID_CONNECT;
    buffer[1]=0x11;
    pack_to_tx(buffer,sizeof(buffer));
}

void car_driver::set_pid(pid_data* data) {
    uint8_t buffer[13]; // 明确数组大小
    buffer[0] = ID_PID;
    std::memcpy(&buffer[1], &data->p, sizeof(float));
    std::memcpy(&buffer[5], &data->i, sizeof(float));
    std::memcpy(&buffer[9], &data->d, sizeof(float));
    pack_to_tx(buffer, sizeof(buffer));
}

void car_driver::set_vel(vel_data* data) {
    uint8_t buffer[17] = {0}; // ID_VEL + 2 doubles
    buffer[0] = ID_VEL;
    static_assert(sizeof(vel_data) == 16, "vel_data size mismatch");
    memcpy(buffer+1, data, sizeof(vel_data));
    pack_to_tx(buffer, sizeof(buffer));
}

void car_driver::set_servo_angle(servo_data* data) {
    uint8_t buffer[5] = {0}; // ID_SERVO + 1 flaot
    float servo_angle_tmp=data->servo_angle;
    if(servo_angle_tmp<-75){servo_angle_tmp=-75;}
    else if(servo_angle_tmp>75){servo_angle_tmp=75;}
    servo_angle_tmp+=135; //偏置到单片机对应角度,jetson上中间为0度,单片机上50ms对应0度
    buffer[0] = ID_SERVO;
    static_assert(sizeof(servo_data) == 4, "vel_data size mismatch");
    memcpy(buffer+1, data, sizeof(servo_data));
    pack_to_tx(buffer, sizeof(buffer));
}

void car_driver::pack_to_tx(uint8_t* data, uint8_t data_length) {
    // 改用动态内存分配
    std::vector<uint8_t> tx_buf(data_length + 5);
    
    tx_buf[0] = pack_begin;
    tx_buf[1] = pack_addr; // 地址码
    tx_buf[2] = data_length;
    std::memcpy(tx_buf.data()+3, data, data_length);
    
    // 计算校验和
    tx_buf[data_length+3] = std::accumulate(
        tx_buf.begin()+1, tx_buf.begin()+data_length+3, 0);
    tx_buf[data_length+4] = pack_end;

    safe_write(tx_buf.data(), tx_buf.size());
}

void car_driver::safe_write(const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const ssize_t written = write(serial_fd_, data, len);
    if(written != static_cast<ssize_t>(len)){
        ROS_ERROR("Partial write: %zd/%zu bytes", written, len);
    }
}

/*动态参数配置服务回调函数*/
void car_driver::dynamic_reconfig_callback(my_pkg::PID_reconfigConfig &config,uint32_t level)
{
   if(first_init_)
   {
        geometry_msgs::Twist pur_vel_pub_data;
	    ROS_INFO("Set PID P:[%f], I:[%f], D:[%f] ServoBias:[%d],p_vel(m/s)[%f],p_ang('/s)[%f]",config.Kp,config.Ki,config.Kd,config.ServoBias,config.p_vel,config.p_ang);
	    //servo_bias_ = config.ServoBias;
        pidData.p=config.Kp;
        pidData.i=config.Ki;
        pidData.d=config.Kd;
        velData.p_vel=config.p_vel;
        velData.p_ang=config.p_ang;
        servoData.servo_angle=config.servo_angle;
        pur_vel_pub_data.linear.x=velData.p_vel;
        pur_vel_pub_data.angular.x=velData.p_ang;
        pur_vel_pub_.publish(pur_vel_pub_data);
        set_pid(&pidData);
        ros::Duration(0.02).sleep();
        set_vel(&velData);
        ros::Duration(0.02).sleep();
        set_servo_angle(&servoData);

        //ros::Duration(0.02).sleep();
        //SetParams(linear_correction_factor_,config.ServoBias);
   }
    else
    {
	    first_init_=true;
	    ROS_INFO("Set PID P:[%f], I:[%f], D:[%f] ServoBias:[%d]",pidData.p,pidData.i,pidData.d,servo_bias_);
    }   
}

void car_driver::update_odometry(const geometry_msgs::Twist& twist_msg) {
    ros::Time current_time = ros::Time::now();
    
    if (first_odom_) {
        last_odom_time_ = current_time;
        first_odom_ = false;

        // 初始化时记录初始IMU角度
        //first_theta_ = imuData.tangleZ * M_PI / 180.0; // 将度转换为弧度
        return;
    }
    
    // 计算时间差
    double dt = (current_time - last_odom_time_).toSec();
    if (dt <= 0) return;
    
    // 获取线速度
    double vx = twist_msg.linear.x;
    double vy = twist_msg.linear.y;  // 车子不是万向轮,通常为0
    
    // 直接从IMU获取偏航角（绕Z轴的欧拉角）
    // imuData.tangleZ 是角度制，需要转换为弧度制
    double new_theta = imuData.tangleZ * M_PI / 180.0 ;
    
    // 角度归一化到 [-pi, pi]
    while (new_theta > M_PI) new_theta -= 2.0 * M_PI;
    while (new_theta < -M_PI) new_theta += 2.0 * M_PI;
    
    // 计算位移（使用IMU的角度）
    double delta_x = (vx * cos(new_theta) - vy * sin(new_theta)) * dt;
    double delta_y = (vx * sin(new_theta) + vy * cos(new_theta)) * dt;
    
    // 更新位置和角度
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    theta_ = new_theta;  // 直接使用IMU的角度
    
    last_odom_time_ = current_time;
}

void car_driver::publish_odom_tf(const geometry_msgs::Twist& twist_msg) {
    ros::Time current_time = ros::Time::now();
    
    // // 发布TF变换
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame_;
    odom_trans.child_frame_id = base_frame_;
    
    odom_trans.transform.translation.x = x_pos_;
    odom_trans.transform.translation.y = y_pos_;
    odom_trans.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();
    
    // // 发布TF
    // odom_broadcaster_.sendTransform(odom_trans);
    
    // 发布Odometry消息
    odom_msg_.header.stamp = current_time;
    odom_msg_.pose.pose.position.x = x_pos_;
    odom_msg_.pose.pose.position.y = y_pos_;
    odom_msg_.pose.pose.position.z = 0.0;
    odom_msg_.pose.pose.orientation = odom_trans.transform.rotation;
    
    odom_msg_.twist.twist = twist_msg;
    
    odom_pub_.publish(odom_msg_);

    pub_pose();
}

void car_driver::reset_odometry() {
    x_pos_ = 0.0;
    y_pos_ = 0.0;
    theta_ = 0.0;
    first_odom_ = true;
    ROS_INFO("Odometry reset to origin");
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"car_driver_node");

#ifdef Debug_
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
#endif

    car_driver driver;
    driver.loop();
    return 0;
}


