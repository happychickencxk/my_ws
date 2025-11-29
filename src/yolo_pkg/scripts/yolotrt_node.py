#!/usr/bin/env python3
# encoding: utf-8
import sys
import time
import rospy
import rospkg
import cv2 as cv
import numpy as np
import threading
import sensor_msgs.point_cloud2 as pc2
from yahboomcar_msgs.msg import *
from yolov5_trt import YoLov5TRT
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import Header
import math

IMAGE_TOPIC = "/camera/color/image_raw"  # 输入的图像话题
DEPTH_IMAGE_TOPIC = "/camera/depth/image_raw"  # 使用深度图像替代点云
CAMERA_INFO_TOPIC = "/camera/color/camera_info"  
YOLO_MSG_TOPIC = "/DetectMsg"  
OUTPUT_IMAGE_TOPIC = "/Detect/image_msg"  
OUTPUT_CAMERA_INFO_TOPIC = "/Detect/camera_info"  

class YoloDetect:
    def __init__(self):
        rospy.init_node("YoloDetect", anonymous=False)
        self.pTime = self.cTime = 0
        
        # 加载YOLO模型
        param_ = rospkg.RosPack().get_path("yolo_pkg") + '/model/' 
        file_yaml = param_ + 'luzhang.yaml'
        PLUGIN_LIBRARY = param_  + "/libmyplugins.so"
        engine_file_path = param_  + "/640X480.engine"
        self.yolov5_wrapper = YoLov5TRT(file_yaml, PLUGIN_LIBRARY, engine_file_path)
        
        # 相机参数 (640x480分辨率)
        self.image_width = 640
        self.image_height = 480
        
        # 使用实际的相机内参（从camera_info获取），这里先设置默认值
        self.focal_length_x = 450.0  # 默认值，会被camera_info覆盖
        self.focal_length_y = 450.0  # 默认值，会被camera_info覆盖
        self.principal_point_x = self.image_width / 2
        self.principal_point_y = self.image_height / 2
        
        # 发布器
        self.pub_camera_info = rospy.Publisher(OUTPUT_CAMERA_INFO_TOPIC, CameraInfo, queue_size=10)
        self.pub_image = rospy.Publisher(OUTPUT_IMAGE_TOPIC, Image, queue_size=10)
        self.pub_msg = rospy.Publisher(YOLO_MSG_TOPIC, TargetArray, queue_size=10)
        
        # 订阅图像话题和深度图像
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(CAMERA_INFO_TOPIC, CameraInfo, self.camera_info_callback)
        self.depth_image_sub = rospy.Subscriber(DEPTH_IMAGE_TOPIC, Image, self.depth_image_callback)

        # 存储当前帧和线程锁
        self.current_frame = None
        self.current_depth_image = None
        self.current_camera_info = None
        self.frame_lock = threading.Lock()
        self.depth_lock = threading.Lock()
        self.camera_info_lock = threading.Lock()

        rospy.loginfo("YOLO检测节点已启动，使用深度图像优化版本...")

    def image_callback(self, ros_image):
        """图像话题回调函数"""
        try:
            cv_image = self.ros_image_to_cv2(ros_image)
            if cv_image is not None:
                with self.frame_lock:
                    self.current_frame = cv_image
        except Exception as e:
            rospy.logerr(f"图像转换失败: {e}")

    def depth_image_callback(self, depth_image):
        """深度图像回调函数"""
        try:
            # 将深度图像转换为numpy数组
            if depth_image.encoding == '16UC1':
                # 16位无符号整数，单位毫米
                depth_array = np.frombuffer(depth_image.data, dtype=np.uint16).reshape(
                    depth_image.height, depth_image.width
                )
                # 转换为米，同时处理无效值
                depth_in_meters = depth_array.astype(np.float32) / 1000.0
                depth_in_meters[depth_array == 0] = 0  # 将0值保持为0
            elif depth_image.encoding == '32FC1':
                # 32位浮点数，单位米
                depth_in_meters = np.frombuffer(depth_image.data, dtype=np.float32).reshape(
                    depth_image.height, depth_image.width
                )
            else:
                rospy.logwarn(f"不支持的深度图像格式: {depth_image.encoding}")
                return
                
            with self.depth_lock:
                self.current_depth_image = depth_in_meters
                
        except Exception as e:
            rospy.logerr(f"深度图像处理失败: {e}")

    def camera_info_callback(self, camera_info):
        """摄像头信息话题回调函数"""
        try:
            with self.camera_info_lock:
                self.current_camera_info = camera_info
                # 从相机信息中获取实际的内参
                if camera_info.K:  # 内参矩阵 [fx, 0, cx, 0, fy, cy, 0, 0, 1]
                    self.focal_length_x = camera_info.K[0]  # fx
                    self.focal_length_y = camera_info.K[4]  # fy
                    self.principal_point_x = camera_info.K[2]  # cx
                    self.principal_point_y = camera_info.K[5]  # cy
                    #rospy.loginfo(f"更新相机内参: fx={self.focal_length_x}, fy={self.focal_length_y}, cx={self.principal_point_x}, cy={self.principal_point_y}")
        except Exception as e:
            rospy.logerr(f"相机信息处理失败: {e}")

    def get_depth_from_depth_image(self, pixel_x, pixel_y, sample_radius=2):
        """
        从深度图像中获取深度值
        """
        if self.current_depth_image is None:
            return None
            
        try:
            # 确保坐标在有效范围内
            x_min = max(0, int(pixel_x) - sample_radius)
            x_max = min(self.current_depth_image.shape[1] - 1, int(pixel_x) + sample_radius)
            y_min = max(0, int(pixel_y) - sample_radius)
            y_max = min(self.current_depth_image.shape[0] - 1, int(pixel_y) + sample_radius)
            
            # 提取采样区域内的深度值
            depth_samples = []
            for y in range(y_min, y_max + 1):
                for x in range(x_min, x_max + 1):
                    depth_val = self.current_depth_image[y, x]
                    # 检查深度值是否有效
                    if depth_val > 0.1 and depth_val < 10.0:  # 有效深度范围：0.1-10米
                        depth_samples.append(depth_val)
            
            if len(depth_samples) > 0:
                # 计算中位数深度，对异常值更鲁棒
                median_depth = np.median(depth_samples)
                return float(median_depth)
            else:
                return None
                
        except Exception as e:
            rospy.logwarn(f"获取深度信息失败: {e}")
            return None

    def calculate_distance_from_depth(self, pixel_x, pixel_y, depth):
        """
        基于深度值和像素坐标计算在机器人坐标系下的X和Y距离
        深度值depth是相机到物体的欧几里得直线距离
        
        坐标系定义：
        - 相机坐标系：X向右，Y向下，Z向前
        - 机器人坐标系：X向前，Y向左，Z向上
        """
        if depth is None or depth <= 0:
            return 0.0, 0.0, 0.0  # 返回三个值，包括欧几里得距离
            
        try:
            # 计算相对于图像中心的归一化坐标
            dx = (pixel_x - self.principal_point_x) / self.focal_length_x
            dy = (pixel_y - self.principal_point_y) / self.focal_length_y
            
            # 深度值depth是欧几里得距离：√(X² + Y² + Z²)
            # 我们需要正确分解到相机坐标系的各个轴
            
            # 计算分母：√(dx² + dy² + 1)
            denominator = math.sqrt(dx*dx + dy*dy + 1.0)
            
            # 计算相机坐标系下的实际坐标
            Z_camera = depth / denominator  # 相机坐标系Z轴（向前）
            X_camera = dx * Z_camera       # 相机坐标系X轴（向右）  
            Y_camera = dy * Z_camera       # 相机坐标系Y轴（向下）
            
            # 验证欧几里得距离（应该约等于depth）
            # euclidean_distance = math.sqrt(X_camera**2 + Y_camera**2 + Z_camera**2)
            
            # 转换为机器人坐标系（X向前，Y向左，Z向上）
            robot_x = Z_camera      # 相机Z -> 机器人X（前方）
            robot_y = -X_camera     # 相机-X -> 机器人Y（左侧为正）
            # robot_z = -Y_camera   # 相机-Y -> 机器人Z（上方为正）
            
            return robot_x, robot_y
            
        except Exception as e:
            rospy.logwarn(f"计算距离失败: {e}")
            return 0.0, 0.0, 0.0

    def ros_image_to_cv2(self, ros_image):
        """手动将sensor_msgs/Image转换为OpenCV图像"""
        try:
            encoding = ros_image.encoding.lower()
            np_arr = np.frombuffer(ros_image.data, dtype=np.uint8)
            
            if encoding == 'rgb8':
                rgb_image = np_arr.reshape(ros_image.height, ros_image.width, 3)
                return cv.cvtColor(rgb_image, cv.COLOR_RGB2BGR)
            elif encoding == 'bgr8':
                return np_arr.reshape(ros_image.height, ros_image.width, 3)
            elif encoding == 'mono8':
                gray_image = np_arr.reshape(ros_image.height, ros_image.width)
                return cv.cvtColor(gray_image, cv.COLOR_GRAY2BGR)
            else:
                rospy.logwarn(f"不支持的图像编码格式: {encoding}")
                return None
        except Exception as e:
            rospy.logerr(f"ROS图像转OpenCV失败: {e}")
            return None

    def cv2_to_ros(self, cv_image, frame_id="camera_color_optical_frame"):
        """将OpenCV BGR图像转换为sensor_msgs/Image"""
        try:
            ros_image = Image()
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = frame_id
            
            height, width = cv_image.shape[:2]
            ros_image.height = height
            ros_image.width = width
            ros_image.is_bigendian = 0
            
            rgb_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
            ros_image.encoding = 'rgb8'
            ros_image.step = width * 3
            ros_image.data = rgb_image.tobytes()
                
            return ros_image
        except Exception as e:
            rospy.logerr(f"BGR转ROS RGB失败: {e}")
            return None

    def publish_camera_info(self):
        """发布相机信息"""
        try:
            with self.camera_info_lock:
                if self.current_camera_info is not None:
                    camera_info = CameraInfo()
                    camera_info.header.stamp = rospy.Time.now()
                    camera_info.header.frame_id = "camera_color_optical_frame"
                    
                    camera_info.height = self.current_camera_info.height
                    camera_info.width = self.current_camera_info.width
                    camera_info.distortion_model = self.current_camera_info.distortion_model
                    camera_info.D = self.current_camera_info.D
                    camera_info.K = self.current_camera_info.K
                    camera_info.R = self.current_camera_info.R
                    camera_info.P = self.current_camera_info.P
                    camera_info.binning_x = self.current_camera_info.binning_x
                    camera_info.binning_y = self.current_camera_info.binning_y
                    camera_info.roi = self.current_camera_info.roi
                    
                    self.pub_camera_info.publish(camera_info)
                    return True
                else:
                    return False
        except Exception as e:
            rospy.logerr(f"发布相机信息失败: {e}")
            return False

    def detect(self, frame):
        """执行YOLO检测并添加深度信息"""
        if frame is None:
            return None
            
        target_array = TargetArray()
        
        try:
            # YOLO推理
            frame, result_boxes, result_scores, result_classid = self.yolov5_wrapper.infer(frame)
            
            # 绘制检测结果
            for j in range(len(result_boxes)):
                box = result_boxes[j]
                class_name = self.yolov5_wrapper.categories[int(result_classid[j])]
                score = result_scores[j]
                
                # 计算目标中心点
                center_x = int((box[0] + box[2]) / 2)
                center_y = int((box[1] + box[3]) / 2)
                
                # 从深度图像获取深度信息
                depth = self.get_depth_from_depth_image(center_x, center_y, sample_radius=3)
                
                # 计算X和Y方向的距离（机器人坐标系）
                if depth is not None and depth > 0:
                    distance_x, distance_y = self.calculate_distance_from_depth(center_x, center_y, depth)
                    
                    # 绘制检测框和标签
                    label = "{}:{:.2f} {:.2f}m".format(class_name, score, depth)
                    coord_text = "X:{:.2f}m Y:{:.2f}m".format(distance_x, distance_y)
                    cv.putText(frame, coord_text, (int(box[0]), int(box[3])+20), 
                              cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                else:
                    label = "{}:{:.2f} N/A".format(class_name, score)
                    distance_x, distance_y = 0.0, 0.0
                    depth = 0.0
                
                self.yolov5_wrapper.plot_one_box(
                    box,
                    frame,
                    [255, 192, 203],
                    label=label,
                )
                
                # 在中心点绘制标记
                cv.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                
                # 创建目标消息
                target = Target()
                target.frame_id = class_name
                target.stamp = rospy.Time.now()
                target.scores = score
                target.ptx = box[0]
                target.pty = box[1]
                target.distw = box[2] - box[0]
                target.disth = box[3] - box[1]
                target.centerx = center_x
                target.centery = center_y
                target.distance_x = distance_x  # 机器人前方距离
                target.distance_y = distance_y  # 机器人左侧距离
                target.distance = depth         # 直线距离
                    
                target_array.data.append(target)
                
            # 计算并显示FPS
            self.cTime = time.time()
            fps = 1 / (self.cTime - self.pTime) if (self.cTime - self.pTime) > 0 else 0
            self.pTime = self.cTime
            text = "FPS : " + str(int(fps))
            cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
            
            # 发布检测结果
            self.pub_msg.publish(target_array)
            
            # 发布处理后的图像
            ros_image = self.cv2_to_ros(frame, "camera_color_optical_frame")
            if ros_image is not None:
                self.pub_image.publish(ros_image)

            # 发布相机信息
            self.publish_camera_info()
            
        except Exception as e:
            rospy.logerr(f"YOLO检测失败: {e}")
            
        return frame

    def run(self):
        """主循环"""
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            with self.frame_lock:
                frame = self.current_frame
                self.current_frame = None
                
            if frame is not None:
                self.detect(frame)
                    
            rate.sleep()

if __name__ == "__main__":
    print("Python version: ", sys.version)
    
    try:
        detect = YoloDetect()
        detect.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv.destroyAllWindows()