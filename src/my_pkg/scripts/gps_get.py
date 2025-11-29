#!/usr/bin/env python3
# encoding: utf-8
import serial
import pynmea2
import rospy
from geometry_msgs.msg import Point

# 打开串口，读取 NMEA 数据
ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)

def parse_and_publish():
    rospy.init_node('gps_publisher', anonymous=True)
    pub = rospy.Publisher('gps_coordinates', Point, queue_size=10)
    print("gps节点成功建立!")
    while not rospy.is_shutdown():
        nmea_sentence = ser.readline().decode('ascii', errors='ignore')
        
        # 解析 NMEA 数据
        msg = parse_nmea_data(nmea_sentence)
        if msg:
            if isinstance(msg, pynmea2.types.talker.GGA):
                latitude = msg.latitude
                longitude = msg.longitude
                altitude = msg.altitude
                
                # 将经纬度和高度发布为 ROS 消息
                gps_point = Point()
                gps_point.x = latitude  # 根据需要进行坐标转换
                gps_point.y = longitude  # 根据需要进行坐标转换
                gps_point.z = altitude  # 高度
                
                pub.publish(gps_point)

def parse_nmea_data(nmea_sentence):
    if nmea_sentence.startswith('$GNGGA'):
        msg = pynmea2.parse(nmea_sentence)
        if isinstance(msg, pynmea2.types.talker.GGA):
            return msg
    else:
        return None

if __name__ == '__main__':
    try:
        parse_and_publish()
    except rospy.ROSInterruptException:
        pass
