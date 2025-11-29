#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from collections import defaultdict, deque
from geometry_msgs.msg import PoseArray, Pose, Point
from yahboomcar_msgs.msg import TargetArray, Target
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

class ConeTracker:
    def __init__(self):
        # 锥桶跟踪参数
        self.cone_id_counter = 0
        self.cone_tracks = {}  # {cone_id: {'position': (x, z), 'color': str, 'confidence': float, 'last_seen': timestamp, 'detection_count': int}}
        self.detection_history = defaultdict(lambda: deque(maxlen=5))  # 存储最近5帧的检测
        
        # 聚类参数
        self.cluster_radius = 0.3  # 聚类半径（米）
        self.min_detections = 3    # 最小连续检测次数
        self.max_age = 5.0         # 锥桶最大保留时间（秒）
        
    def update(self, detections):
        """
        更新锥桶跟踪状态
        detections: [(x, z, color, confidence)]
        """
        current_time = rospy.Time.now().to_sec()
        
        # 将新检测添加到历史记录
        for det in detections:
            x, z, color, confidence = det
            self.detection_history[(x, z, color)].append((current_time, confidence))
        
        # 更新现有锥桶或创建新锥桶
        updated_cones = {}
        
        for det_key, detection_times in self.detection_history.items():
            if len(detection_times) >= self.min_detections:
                x, z, color = det_key
                
                # 计算平均位置和置信度
                avg_x = x
                avg_z = z
                avg_confidence = np.mean([conf for _, conf in detection_times])
                
                # 查找最近的现有锥桶
                matched_id = self._find_nearest_cone(avg_x, avg_z, color)
                
                if matched_id is not None:
                    # 更新现有锥桶
                    old_pos = self.cone_tracks[matched_id]['position']
                    # 使用加权平均更新位置（新检测权重更高）
                    new_x = 0.7 * avg_x + 0.3 * old_pos[0]
                    new_z = 0.7 * avg_z + 0.3 * old_pos[1]
                    
                    self.cone_tracks[matched_id].update({
                        'position': (new_x, new_z),
                        'confidence': avg_confidence,
                        'last_seen': current_time,
                        'detection_count': self.cone_tracks[matched_id]['detection_count'] + 1
                    })
                    updated_cones[matched_id] = self.cone_tracks[matched_id]
                else:
                    # 创建新锥桶
                    cone_id = self.cone_id_counter
                    self.cone_id_counter += 1
                    
                    self.cone_tracks[cone_id] = {
                        'position': (avg_x, avg_z),
                        'color': color,
                        'confidence': avg_confidence,
                        'last_seen': current_time,
                        'detection_count': len(detection_times)
                    }
                    updated_cones[cone_id] = self.cone_tracks[cone_id]
        
        # 清理旧锥桶
        self._clean_old_cones(current_time)
        
        return updated_cones
    
    def _find_nearest_cone(self, x, z, color, max_distance=0.5):
        """
        查找最近的同颜色锥桶
        """
        closest_id = None
        min_distance = float('inf')
        
        for cone_id, cone_data in self.cone_tracks.items():
            if cone_data['color'] != color:
                continue
                
            cone_x, cone_z = cone_data['position']
            distance = math.sqrt((x - cone_x)**2 + (z - cone_z)**2)
            
            if distance < min_distance and distance < max_distance:
                min_distance = distance
                closest_id = cone_id
        
        return closest_id
    
    def _clean_old_cones(self, current_time):
        """
        清理长时间未见的锥桶
        """
        cones_to_remove = []
        for cone_id, cone_data in self.cone_tracks.items():
            if current_time - cone_data['last_seen'] > self.max_age:
                cones_to_remove.append(cone_id)
        
        for cone_id in cones_to_remove:
            del self.cone_tracks[cone_id]

class ConeMapper:
    def __init__(self):
        rospy.init_node('cone_mapper', anonymous=True)
        
        # 订阅和发布
        self.sub = rospy.Subscriber('/DetectMsg', TargetArray, self.detect_callback, queue_size=10)
        
        # 发布转换后的锥桶位置（在相机光学坐标系下）
        self.pub_cones = rospy.Publisher('/camera_cones', PoseArray, queue_size=10)
        
        # 发布可视化标记
        self.pub_markers = rospy.Publisher('/visualization_markers', MarkerArray, queue_size=10)
        
        # 发布稳定的锥桶位置（经过跟踪确认的）
        self.pub_stable_cones = rospy.Publisher('/stable_cones', PoseArray, queue_size=10)
        
        # 锥桶颜色定义
        self.cone_colors = {
            'red': ColorRGBA(1.0, 0.0, 0.0, 1.0),      # 红色
            'blue': ColorRGBA(0.0, 0.0, 1.0, 1.0),     # 蓝色  
            'yellow': ColorRGBA(1.0, 1.0, 0.0, 1.0),   # 黄色
            'green': ColorRGBA(0.0, 1.0, 0.0, 1.0),    # 绿色
            'default': ColorRGBA(0.5, 0.5, 0.5, 1.0)   # 默认灰色
        }
        
        # 锥桶尺寸定义
        self.cone_height = 0.45  # 锥桶高度（米）
        self.cone_radius = 0.1   # 锥桶底部半径（米）
        
        # 锥桶跟踪器
        self.cone_tracker = ConeTracker()
        
        # 存储所有确认的锥桶
        self.confirmed_cones = {}  # {cone_id: cone_data}
        
        # 标记ID计数器
        self.marker_id_counter = 0
        
        rospy.loginfo("改进版锥桶映射节点已启动，具有跟踪和过滤功能...")

    def detect_callback(self, msg):
        """
        处理检测到的锥桶消息，使用跟踪器确认稳定的锥桶
        """
        try:
            # 提取当前帧的检测结果
            current_detections = []
            for target in msg.data:
                # 锥桶在相机坐标系下的位置
                cone_camera_x = -target.distance_y  # 坐标轴相反
                cone_camera_z = target.distance_x   # distance_x对应相机坐标系的Z轴
                
                current_detections.append((
                    cone_camera_x, 
                    cone_camera_z, 
                    target.frame_id, 
                    target.scores
                ))
            
            # 更新锥桶跟踪器
            updated_cones = self.cone_tracker.update(current_detections)
            
            # 更新确认的锥桶列表
            self.confirmed_cones.update(updated_cones)
            
            # 发布所有确认的锥桶
            self.publish_confirmed_cones()
            
            # 发布当前帧检测到的锥桶（用于实时显示）
            self.publish_current_detections(msg)
            
        except Exception as e:
            rospy.logerr("处理锥桶数据失败: %s", str(e))
    
    def publish_confirmed_cones(self):
        """
        发布经过确认的稳定锥桶
        """
        # 创建PoseArray消息
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "camera_color_optical_frame"
        
        # 创建MarkerArray消息
        marker_array = MarkerArray()
        
        for cone_id, cone_data in self.confirmed_cones.items():
            x, z = cone_data['position']
            color = cone_data['color']
            confidence = cone_data['confidence']
            detection_count = cone_data['detection_count']
            
            # 创建锥桶的位姿
            cone_pose = Pose()
            cone_pose.position.x = x
            cone_pose.position.y = -1  # 假设锥桶在地面上
            cone_pose.position.z = z
            cone_pose.orientation.w = 1.0
            
            pose_array.poses.append(cone_pose)
            
            # 创建可视化标记
            marker = self.create_cone_marker(
                cone_pose.position, 
                color, 
                confidence,
                cone_id,
                detection_count
            )
            marker_array.markers.append(marker)
            
            rospy.loginfo("确认锥桶 #%d: %s(置信度:%.2f, 检测次数:%d) -> 相机坐标 (X:%.2f, Z:%.2f)", 
                         cone_id, color, confidence, detection_count, x, z)
        
        # 发布稳定的锥桶位置
        self.pub_stable_cones.publish(pose_array)
        
        # 发布可视化标记
        self.pub_markers.publish(marker_array)
    
    def publish_current_detections(self, msg):
        """
        发布当前帧检测到的锥桶（用于实时显示）
        """
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "camera_color_optical_frame"
        
        for target in msg.data:
            cone_camera_x = -target.distance_y
            cone_camera_z = target.distance_x
            
            cone_pose = Pose()
            cone_pose.position.x = cone_camera_x
            cone_pose.position.y = -1
            cone_pose.position.z = cone_camera_z
            cone_pose.orientation.w = 1.0
            
            pose_array.poses.append(cone_pose)
        
        self.pub_cones.publish(pose_array)
    
    def create_cone_marker(self, position, cone_type, confidence, cone_id, detection_count):
        """
        创建锥桶的可视化标记
        """
        marker = Marker()
        marker.header.frame_id = "camera_color_optical_frame"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "confirmed_cones"
        marker.id = cone_id  # 使用锥桶ID作为标记ID
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # 设置位置
        marker.pose.position = position
        marker.pose.position.y = 0.3  # 锥桶中心高度
        marker.pose.orientation.w = 0.7071
        marker.pose.orientation.x = 0.7071
        
        # 设置尺寸（根据检测次数调整大小）
        size_factor = min(1.0, detection_count / 10.0)  # 检测次数越多，尺寸越大
        marker.scale.x = self.cone_radius * 2 * (0.8 + 0.2 * size_factor)
        marker.scale.y = self.cone_height * (0.8 + 0.2 * size_factor)
        marker.scale.z = self.cone_radius * 2 * (0.8 + 0.2 * size_factor)
        
        # 设置颜色
        if cone_type in self.cone_colors:
            marker.color = self.cone_colors[cone_type]
        else:
            marker.color = self.cone_colors['default']
            
        # 设置透明度基于置信度和检测次数
        alpha = max(0.5, min(1.0, confidence)) * (0.7 + 0.3 * min(1.0, detection_count / 5.0))
        marker.color.a = alpha
        
        # 设置存活时间（确认的锥桶保持更长时间）
        marker.lifetime = rospy.Duration(10.0)
        
        return marker

    def run(self):
        """
        主循环
        """
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        mapper = ConeMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        pass