#!/usr/bin/env python3
# encoding: utf-8
from platform import python_version
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

print(python_version())
def callback(data):
    # 获取 PointCloud2 消息的字段信息
    # 例如，检查数据的高度、宽度和总点数
    width = data.width
    height = data.height
    point_step = data.point_step  # 每个点占的字节数
    row_step = data.row_step      # 每行占的字节数

    # 打印关于点云的一些基本信息
    rospy.loginfo("PointCloud2 Message Information: ")
    rospy.loginfo(f"Width: {width}")
    rospy.loginfo(f"Height: {height}")
    rospy.loginfo(f"Point Step: {point_step}")
    rospy.loginfo(f"Row Step: {row_step}")

    # 计算总点数
    total_points = (width * height)
    rospy.loginfo(f"Total Points: {total_points}")

    # 如果需要，可以通过以下方式读取点云数据并检查其中的元素大小
    points = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    
    # 遍历点云数据，打印第一个点的坐标
    for p in points:
        rospy.loginfo(f"First point: {p}")
        break  # 这里只是获取第一个点

def listener():
    rospy.init_node('pointcloud_size_checker', anonymous=True)
    
    # 订阅 /camera/depth/points 话题，接收 PointCloud2 消息
    rospy.Subscriber("/camera/depth/points", PointCloud2, callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
