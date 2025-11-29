import rospy
import serial
import struct
from collections import deque
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist

# 定义FIFO缓存队列

fifo_queue = deque()

# 串口设置
serial_port = '/dev/ttyTHS1'
baud_rate = 115200

# 数据包格式
START_BYTE = 0x0F
END_BYTE = 0x4D

# ROS节点初始化
rospy.init_node('serial_communication_node')

# 订阅的话题
rospy.Subscriber("/desired_speed", Twist, lambda msg: send_desired_speed(msg))
rospy.Subscriber("/pid_config", Float32, lambda msg: send_pid_config(msg))

# 串口通信函数
def serial_read():
    # 打开串口
    ser = serial.Serial(serial_port, baud_rate, timeout=1)

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            byte = ser.read(1)
            if byte:
                fifo_queue.append(byte)
                process_fifo()

# 数据包处理函数
def process_fifo():
    # 判断FIFO队列中是否有完整的数据包
    if len(fifo_queue) >= 7:  # 至少要有标识码、地址码、长度码、功能码和结束码
        # 获取标识码、地址码、长度码、功能码和结束码
        start_byte = fifo_queue.popleft()
        if start_byte != bytes([START_BYTE]):
            return  # 跳过无效的起始字节

        address_byte = fifo_queue.popleft()
        length_byte = fifo_queue.popleft()
        func_byte = fifo_queue.popleft()

        length = int.from_bytes(length_byte, byteorder='big')
        data_bytes = [fifo_queue.popleft() for _ in range(length)]
        
        checksum_byte = fifo_queue.popleft()
        if checksum_byte != calculate_checksum(address_byte, length_byte, func_byte, data_bytes):
            rospy.logwarn("Checksum error.")
            return  # 校验错误

        end_byte = fifo_queue.popleft()
        if end_byte != bytes([END_BYTE]):
            return  # 跳过无效的结束字节

        # 调用对应的功能处理函数
        handle_function(func_byte, data_bytes)

# 校验和计算函数
def calculate_checksum(start, address, length, func, data):
    checksum = start[0] + address[0] + length[0] + func[0] + sum([d[0] for d in data])
    return bytes([checksum % 256])

# 根据功能码调用相应处理函数
def handle_function(func_byte, data_bytes):
    if func_byte == bytes([0x00]):
        handle_communication_check(data_bytes)
    elif func_byte == bytes([0x01]):
        handle_gps_data(data_bytes)
    elif func_byte == bytes([0x02]):
        handle_imu_data(data_bytes)
    elif func_byte == bytes([0x03]):
        handle_adc_data(data_bytes)
    elif func_byte == bytes([0x04]):
        handle_adc_request()
    elif func_byte == bytes([0x05]):
        handle_desired_speed(data_bytes)
    elif func_byte == bytes([0x07]):
        handle_displacement_data(data_bytes)

# 各种数据包的处理函数
def handle_communication_check(data_bytes):
    # 数据体是一个随机数，直接发送回去
    rospy.loginfo("Received communication check response.")

def handle_gps_data(data_bytes):
    # 假设data_bytes为16字节，两个double值，解析经纬度
    longitude = struct.unpack('d', data_bytes[:8])[0]
    latitude = struct.unpack('d', data_bytes[8:])[0]
    gps_msg = NavSatFix()
    gps_msg.longitude = longitude
    gps_msg.latitude = latitude
    rospy.loginfo(f"Received GPS data: Longitude={longitude}, Latitude={latitude}")
    gps_pub.publish(gps_msg)

def handle_imu_data(data_bytes):
    # IMU数据透传
    rospy.loginfo("Received IMU data.")

def handle_adc_data(data_bytes):
    # 假设data_bytes为4字节，float数据
    adc_value = struct.unpack('f', data_bytes)[0]
    rospy.loginfo(f"Received ADC data: {adc_value}")
    adc_pub.publish(adc_value)

def handle_adc_request():
    # 发送ADC请求数据包到单片机
    send_data(0x04, b'')

def handle_desired_speed(data_bytes):
    # 解析线速度和角速度
    linear_velocity = struct.unpack('d', data_bytes[:8])[0]
    angular_velocity = struct.unpack('d', data_bytes[8:])[0]
    speed_msg = Twist()
    speed_msg.linear.x = linear_velocity
    speed_msg.angular.z = angular_velocity
    rospy.loginfo(f"Sending desired speed: Linear={linear_velocity}, Angular={angular_velocity}")
    desired_speed_pub.publish(speed_msg)

def handle_displacement_data(data_bytes):
    # 位移数据包
    x_displacement = struct.unpack('q', data_bytes[:8])[0]
    y_displacement = struct.unpack('d', data_bytes[8:])[0]
    rospy.loginfo(f"Received displacement data: X={x_displacement}, Y={y_displacement}")

# 发送数据包函数
def send_data(func_byte, data):
    # 计算长度
    length = len(data)
    # 构建数据包
    packet = bytes([START_BYTE]) + bytes([0x00])  # 地址码为0x00
    packet += bytes([length])  # 长度码
    packet += bytes([func_byte])  # 功能码
    packet += data  # 数据体
    checksum = calculate_checksum(packet[0:4], packet[1:2], packet[2:3], packet[3:4], [data])
    packet += checksum
    packet += bytes([END_BYTE])  # 结束码
    # 发送数据包
    ser = serial.Serial(serial_port, baud_rate)
    ser.write(packet)
    rospy.loginfo(f"Sent data: {packet.hex()}")

# 发送期望速度数据
def send_desired_speed(msg):
    # 线速度和角速度
    linear_velocity = struct.pack('d', msg.linear.x)
    angular_velocity = struct.pack('d', msg.angular.z)
    data = linear_velocity + angular_velocity
    send_data(0x05, data)

# 发送PID配置数据
def send_pid_config(msg):
    # 发送PID配置数据包
    pid_values = struct.pack('3f', msg.data, msg.data, msg.data)
    send_data(0x08, pid_values)

if __name__ == '__main__':
    gps_pub = rospy.Publisher('/gps_data', NavSatFix, queue_size=10)
    adc_pub = rospy.Publisher('/adc_data', Float32, queue_size=10)
    desired_speed_pub = rospy.Publisher('/desired_speed', Twist, queue_size=10)

    # 开始串口读取线程
    serial_read()
