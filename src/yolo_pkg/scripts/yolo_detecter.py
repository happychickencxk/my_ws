#!/usr/bin/env python3
# encoding: utf-8
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import torch
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords
from utils.torch_utils import select_device
import numpy as np
import cv2

# ROS Topic
rospy.init_node('yolo_detector',anonymous=True)
rospy.loginfo("hello!")
IMAGE_TOPIC = "/camera/color/image_raw"  # 输入的图像话题
YOLO_MSG_TOPIC = "/yolo_msg"  # 发布的目标物体坐标话题
OUTPUT_IMAGE_TOPIC = "/yolo_output_image"  # 发布带框图像的图像话题

# YOLOv5初始化
weights = 'src/yolo_pkg/scripts/best.pt'  # 使用自定义训练的YOLOv5模型
device = select_device('')  # 使用默认的设备
model = attempt_load(weights, map_location=device)  # 加载模型
rospy.loginfo("success load model!")
img_size = 640  # 输入图像大小
stride = int(model.stride.max())  # 获取模型步长
img_size = check_img_size(img_size, s=stride)  # 确保图像大小与模型兼容
half = device.type != 'cpu'  # 是否使用FP16
if half:
    model.half()  # 将模型转换为半精度
model.eval()  # 设置为评估模式

# 初始化CV桥接
bridge = CvBridge()
rospy.loginfo("success initial the cvbridge")

# 回调函数，用于处理图像消息
def image_callback(msg):
    # 将ROS图像消息转为OpenCV图像
    img = bridge.imgmsg_to_cv2(msg)
    
    # 转换为PyTorch张量并准备图像进行推理
    img_tensor = torch.from_numpy(img).to(device)
    img_tensor = img_tensor.half() if half else img_tensor.float()  # 半精度或全精度
    img_tensor /= 255.0  # 标准化到0-1
    if img_tensor.ndimension() == 3:
        img_tensor = img_tensor.unsqueeze(0)

    # 推理
    pred = model(img_tensor)[0]
    
    # 应用非极大值抑制（NMS）去除多余的框
    pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45)

    # 获取图像的宽高
    im0 = img.copy()

    detected_coords = []
    labels = []  # 用于存储类别标签
    colors = []  # 用于存储颜色（可选）

    # 处理检测结果
    for det in pred:  # 每张图片的检测结果
        if len(det):
            # 调整框的坐标，使其与原图匹配
            det[:, :4] = scale_coords(img_tensor.shape[2:], det[:, :4], im0.shape).round()

            # 遍历每个检测到的物体
            for *xyxy, conf, cls in det:
                # 获取中心坐标 (xywh 格式的坐标转为中心点 x, y)
                xyxy = torch.tensor(xyxy).view(1, 4)
                x_center, y_center = (xyxy[0][0] + xyxy[0][2]) / 2, (xyxy[0][1] + xyxy[0][3]) / 2

                # 将目标物体的坐标添加到列表中
                detected_coords.append([x_center.item(), y_center.item()])
                labels.append(f"Class: {int(cls)} Conf: {conf.item():.2f}")

                # 绘制框和标签
                color = [0, 255, 0]  # 默认绿色
                colors.append(color)
                cv2.rectangle(im0, (int(xyxy[0][0]), int(xyxy[0][1])), 
                            (int(xyxy[0][2]), int(xyxy[0][3])), color, 2)
                cv2.putText(im0, labels[-1], (int(xyxy[0][0]), int(xyxy[0][1]) - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # 发布带框图像
    if im0 is not None:
        output_img_msg = bridge.cv2_to_imgmsg(im0, encoding='bgr8')
        output_img_pub.publish(output_img_msg)

    # 将检测到的坐标封装成 Float32MultiArray 消息
    if detected_coords:
        coords_msg = Float32MultiArray()
        coords_msg.data = np.array(detected_coords).flatten()  # 将坐标展平为一维数组
        yolo_msg_pub.publish(coords_msg)  # 发布消息

# 初始化ROS节点和发布器
yolo_msg_pub = rospy.Publisher(YOLO_MSG_TOPIC, Float32MultiArray, queue_size=10)
output_img_pub = rospy.Publisher(OUTPUT_IMAGE_TOPIC, Image, queue_size=10)
rospy.Subscriber(IMAGE_TOPIC, Image, image_callback)

# 持续运行
rospy.spin()
