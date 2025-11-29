#!/usr/bin/env python3
# encoding: utf-8

#使用时需要先运行sudo chmod 777 /dev/ttyTHS1
#获得串口权限
#本程序是对GNGAA,GNRMC解析的测试
import serial
import pynmea2
from geopy.distance import geodesic
import rospy
from geometry_msgs.msg import Point

rospy.init_node('gps_pub',anonymous=True)
pub = rospy.Publisher('gps_coordinates', Point, queue_size=10)
ser = serial.Serial('/dev/ttyTHS1', 115200,timeout=1)
if ser.isOpen():
    print("串口打开成功!")
else:
    print("串口打开失败!")
    ser.close()

def parse_nmea(sentence):
    if sentence.startswith('$GNGGA'):
        record = pynmea2.parse(sentence)
        gps_time = str(record.timestamp)
        gps_time_beijing = int(gps_time[0:2]) * 10000 + int(gps_time[3:5]) * 100 + int(gps_time[6:8]) + 80000
        if gps_time_beijing > 240000:
            gps_time_beijing = gps_time_beijing - 240000
        fix_mode = record.gps_qual  #定位模式，1->普通单点定位；2->差分定位；4->固定解；5->浮点解；
        #latitude_dir=record.latitude_dir
        #longitude_dir=record.longitude_dir
        num_satallites_used = record.num_sats #定位时卫星使用个数
        latitude = record.latitude   #纬度
        lat=str(latitude)+" "+record.lat_dir
        longitude = record.longitude  #经度
        lon=str(longitude)+" "+record.lon_dir
        altitude = record.altitude  #高度
        #distance_1 = '%.3f' % geodesic((latitude, longitude), (lat1, lng1)).m  # 与基准点的距离
        hdop = record.horizontal_dil  #水平精度因子
        ref_station_id = record.ref_station_id #差分钟ID，使用RTK定位时才有
        print('UTC时间: ', gps_time)
        print('时间戳: ', gps_time_beijing)
        print('定位模式: ', fix_mode)
        print('定位时卫星使用个数: ', num_satallites_used)
        print('纬度: ', lat)
        print('经度: ', lon)
        #print('地面速度: ',record.spd_over_grnd)
        #print('地面航向: ',record.course_over_grnd)
        #print('日期: ',record.datestamp)
        print("高度: ",altitude)
        print('水平精度因子: ', hdop,"\n")
        #print('distance:', distance_1)
    elif sentence.startswith('$GNRMC'):
        record = pynmea2.parse(sentence)
        print("RMC Data:")
        print(f"Time: {record.timestamp}")
        print(f"Latitude: {record.latitude} {record.lat_dir}")
        print(f"Longitude: {record.longitude} {record.lon_dir}")
        print(f"加速度: {record.data[6]}")
        print(f"方位角: {record.data[7]}")



while True:
    sentence=ser.readline().decode('ascii',errors='ignore')
    if sentence:
       # print(sentence)
        parse_nmea(sentence)

