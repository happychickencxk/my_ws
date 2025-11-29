#!/usr/bin/env python3

import rospy
import cv2
import base64
import torch
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords,xyxy2xywh
from utils.torch_utils import select_device
from utils.datasets import letterbox
import numpy as np
from std_msgs.msg import Float32MultiArray
from yahboomcar_msgs.msg import Image_Msg  # 假设定义了Image_Msg消息
import time

# ROS Topic
IMAGE_TOPIC = "/camera/color/image_raw"  # 输入的图像话题
YOLO_MSG_TOPIC = "/yolo_msg"  # 发布的目标物体坐标话题

# YOLOv5初始化
weights = 'src/yolo_pkg/model/640X480.pt'  # 使用自定义训练的YOLOv5模型
print("预加载模型:",weights)
device = select_device('')  # 使用默认的设备
if device:
    print("选用设备:", device)
else:
    print("设备初始化失败!")
print("正在初始化模型......")
model = attempt_load(weights, map_location=device)  # 加载模型
if model:
    print("成功加载模型")
else:
    print("模型初始化失败!")
img_size = 640  # 输入图像大小
stride = int(model.stride.max())  # 获取模型步长
print("模型步长:", stride)
img_size = check_img_size(img_size, s=stride)  # 确保图像大小与模型兼容
print("输入图像大小:",img_size)
half = device.type != 'cpu'  # 是否使用FP16
if half:
    model.half()  # 将模型转换为半精度
    print("模型转换为半精度")
model.eval()  # 设置为评估模式

def infer_and_pub(img):  # img为cv格式
    start = time.time()
    # 使用letterbox函数处理图像，确保输入尺寸和模型匹配
    #img = letterbox(img, new_shape=img_size)[0]  # 调整图像大小并保持宽高比
    # 将图像从BGR（OpenCV）转为RGB，并转换为PyTorch张量
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # 转换为RGB
    img_tensor = torch.from_numpy(img_rgb).to(device)  # 转为tensor
    img_tensor = img_tensor.permute(2, 0, 1)  # 通道转置 (HWC -> CHW)
    img_tensor = img_tensor.float() / 255.0  # 标准化
    if img_tensor.ndimension() == 3:
        img_tensor = img_tensor.unsqueeze(0)  # 添加batch维度
    if half:  # 如果模型在FP16模式
        img_tensor = img_tensor.half()  # 将图像转换为半精度
    # 推理
    pred = model(img_tensor)[0]
    
    # 应用非极大值抑制（NMS）去除多余的框
    pred = non_max_suppression(pred, conf_thres=0.75, iou_thres=0.45)

    detected_coords = []

    # 处理检测结果
    for det in pred:  # 每张图片的检测结果
        if len(det):
            # 获取图像的宽高
            im0 = img.copy()
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # 归一化增益
            det[:, :4] = scale_coords(img_tensor.shape[2:], det[:, :4], im0.shape).round()

            # 遍历每个检测到的物体
            for *xyxy, conf, cls in det:
                # 获取中心坐标 (xywh 格式的坐标转为中心点 x, y)
                xywh = xyxy2xywh(torch.tensor(xyxy).view(1, 4))
                x_center, y_center = xywh[0][0].item(), xywh[0][1].item()
                #print(conf)
                #print(cls)
                # 将目标物体的坐标添加到列表中
                detected_coords.append([x_center, y_center])

    # 将检测到的坐标封装成 Float32MultiArray 消息
    if detected_coords:
        print(detected_coords)
        coords_msg = Float32MultiArray()
        coords_msg.data = np.array(detected_coords).flatten()  # 将坐标展平为一维数组
        yolo_msg_pub.publish(coords_msg)  # 发布消息
    end = time.time()
    print("帧率 :",int(1/(end-start)))

def image_publisher():
    # 初始化ROS节点
    global yolo_msg_pub
    rospy.init_node('usb_camera_publisher', anonymous=True)

    # 创建图像发布者，发布到"/camera/image"话题
    image_pub = rospy.Publisher("yolo/image_raw", Image_Msg, queue_size=10)
    yolo_msg_pub = rospy.Publisher(YOLO_MSG_TOPIC, Float32MultiArray, queue_size=10)

    # 设置图像发布的频率，10 Hz
    rate = rospy.Rate(10)

    # 打开串口摄像头（默认设备是/dev/video0）
    capture = cv2.VideoCapture(0)
    capture.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv2.CAP_PROP_FPS))

    if not capture.isOpened():
        rospy.logerr("无法打开摄像头!")
        return

    while not rospy.is_shutdown():
        # 读取摄像头的一帧图像
        ret, frame = capture.read()

        if not ret:
            rospy.logerr("无法获取视频帧")
            break
        else:
            # 进行目标检测并发布坐标
            infer_and_pub(frame)

        # 将图像编码为 base64
        _, img_encoded = cv2.imencode('.jpg', frame)  # 编码为jpg格式
        pic_base64 = base64.b64encode(img_encoded).decode('utf-8')

        # 创建消息并发布图像数据
        image = Image_Msg()
        size = frame.shape
        image.height = size[0]
        image.width = size[1]
        image.channels = size[2]
        image.data = pic_base64
        image_pub.publish(image)

        # 按照设定频率发布图像
        rate.sleep()

    # 关闭摄像头
    capture.release()

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
