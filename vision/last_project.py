# 导入需要的库
import os
import sys
from pathlib import Path
import numpy as np
import cv2
import torch
import math

import serial
import struct

# 设置串口
port = 'COM6'  # 替换为你的串口设备
baudrate = 115200
timeout = 1
# 包头和包尾
tou = 0xFE  # 0xFE 对应十进制 254
wei = 0xEF  # 0xEF 对应十进制 239

# ser = serial.Serial(port, baudrate, timeout=timeout)
scale_factor = 1

def trans_Byte(short_num):
    packed_data = struct.pack('h', int(short_num))

    # 分别存储这两个字节
    byte1 = packed_data[0]
    byte2 = packed_data[1]
    return byte1,byte2

# 定义深度信息调用函数
def get_world_pos(x, y, param):
    threeD = param
    distance = math.sqrt(threeD[y][x][0] ** 2 + threeD[y][x][1] ** 2 + threeD[y][x][2] ** 2)
    distance = distance / 1000.0  # mm -> m
    # 距离 x y z
    data = [distance, threeD[y][x][0],threeD[y][x][1],threeD[y][x][2]]
    return data

obj = "FUN"
def send_data(obj, msg):
    np_data = np.zeros((len(obj), 5))
    for i in range(len(msg)):
        x1 = msg[i]['position'][0]
        y1 = msg[i]['position'][1]
        w1 = msg[i]['position'][2]
        h1 = msg[i]['position'][3]
        center_x = int(x1 + w1 / 2)
        center_y = int(y1 + h1 / 2)
        cv2.rectangle(img, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 0), 4)  # 第一张图片的处理结果图片

        data = get_world_pos(center_x, center_y, threeD)
        dis = data[0]
        if dis < 30:
            cv2.putText(img, "%.2f m" % dis, (center_x, center_y), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
        cv2.putText(img, str(msg[i]['class']) + ' ' + str(round(msg[i]['conf'], 2)), (x1, y1 - 5),
                    cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
        for j in range(len(obj)):
            if str(msg[i]['class']) == obj[j]:
                # print("class:%s, x = %.2f, y = %.2f, z = %.2f, distance = %.2f"%(str(msg[i]['class']), data[1], data[2], data[3], data[0]))
                alpha_class = int((ord(str(msg[i]['class'])) - ord('A') + 1))
                for k in range(4):
                    if data[k]<2000:
                        pass
                    else:
                        data[k] = 0
                try:
                    alpha_x = int(data[1] * scale_factor)
                    alpha_y = int(data[2] * scale_factor)
                    alpha_z = int(data[3] * scale_factor)
                    distance = int(data[0] * 100)
                except:
                    alpha_x = 0
                    alpha_y = 0
                    alpha_z = 0
                    distance = 0
                np_data[j] = np.array([alpha_class, alpha_x, alpha_y, alpha_z, distance])

        
        flag_data = -1
        for i in range(len(obj)):
            if np_data[i,1]!=0:
                flag_data = i
                break
            # 打包数据
        if flag_data!=-1:
            class_byte1, class_byte2 = trans_Byte(flag_data+1)
            x_byte1, x_byte2 = trans_Byte(-(np_data[flag_data,1]+70))
            y_byte1, y_byte2 = trans_Byte(np_data[flag_data,2]-380)
            z_byte1, z_byte2 = trans_Byte(np_data[flag_data,3])
            packed_data = struct.pack('BBBBBBBBBB', tou, class_byte1, class_byte2, x_byte1, x_byte2, y_byte1, y_byte2, z_byte1, z_byte2, wei)
            print(np_data[flag_data, 0],-(np_data[flag_data, 1]), np_data[flag_data, 2]-360,np_data[flag_data, 3])
            print("--",np_data[flag_data])
            # 打开串口
            ser = serial.Serial(port, baudrate, timeout=timeout)

            # 发送数据
            ser.write(packed_data)

            # 接收数据（示例，假设接收的数据长度与发送的相同）
            received_data = ser.read(len(packed_data))

            # 关闭串口
            ser.close()

            # 打印接收到的数据
            print('Received Data:', received_data)
            # time.sleep(0.1)

# 初始化目录
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # 定义YOLOv5的根目录
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # 将YOLOv5的根目录添加到环境变量中（程序结束后删除）
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync

# 导入letterbox
from utils.augmentations import Albumentations, augment_hsv, copy_paste, letterbox, mixup, random_perspective

weights = ROOT / 'H:\\jetson_nano\\bio_dis_yolo\\runs\\train\exp10\weights\\best.pt'  # 权重文件地址   .pt文件
source = ROOT / 'data/images'  # 测试数据文件(图片或视频)的保存路径
data = ROOT / 'data/coco128.yaml'  # 标签文件地址   .yaml文件

imgsz = (640, 480)  # 输入图片的大小 默认640(pixels)
conf_thres = 0.6  # object置信度阈值 默认0.25  用在nms中
iou_thres = 0.45  # 做nms的iou阈值 默认0.45   用在nms中
max_det = 1000  # 每张图片最多的目标数量  用在nms中
device = '0'  # 设置代码执行的设备 cuda device, i.e. 0 or 0,1,2,3 or cpu
classes = None  # 在nms中是否是只保留某些特定的类 默认是None 就是所有类只要满足条件都可以保留 --class 0, or --class 0 2 3
agnostic_nms = False  # 进行nms是否也除去不同类别之间的框 默认False
augment = False  # 预测是否也要采用数据增强 TTA 默认False
visualize = False  # 特征图可视化 默认FALSE
half = False  # 是否使用半精度 Float16 推理 可以缩短推理时间 但是默认是False
dnn = False  # 使用OpenCV DNN进行ONNX推理

# 左镜头的内参，如焦距
left_camera_matrix = np.array(
    [[484.0110,   -0.1191,  310.7593],
      [0,  483.8597,  266.5785], 
      [0., 0., 1.]])
# left_camera_matrix = left_camera_matrix.T

right_camera_matrix = np.array(
    [[484.4944,   -0.2365,  296.1681], 
     [0,  484.4724,  265.9326], 
     [0., 0., 1.]])
# right_camera_matrix = right_camera_matrix.T

# 畸变系数,K1、K2、K3为径向畸变,P1、P2为切向畸变
left_distortion = np.array([[0.063407, 0.071894, -0.002046, -0.000872, -0.285164]])
right_distortion = np.array([[0.044974, 0.253598, -0.001125, 0.000035, -0.797135]])

# 旋转矩阵
R = np.array([[1.0000,   -0.0007,    0.0037],
              [0.0007,    1.0000,   -0.0008],
              [-0.0037,    0.0008,    1.0000]])

# 平移矩阵
T = np.array([59.1001,    0.0046,   -0.2262])

size = (640, 480)

R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                  right_camera_matrix, right_distortion, size, R, T)

# 校正查找映射表,将原始图像和校正后的图像上的点一一对应起来
left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)
print(Q)

# 获取设备
device = select_device(device)

# 载入模型
model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data)
stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
imgsz = check_img_size(imgsz, s=stride)  # 检查图片尺寸

# Half
# 使用半精度 Float16 推理
half &= (pt or jit or onnx or engine) and device.type != 'cpu'  # FP16 supported on limited backends with CUDA
if pt or jit:
    model.model.half() if half else model.model.float()


def detect(img):
    # Dataloader
    # 载入数据
    dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt)

    # 开始预测
    model.warmup(imgsz=(1, 3, *imgsz))  # warmup
    dt, seen = [0.0, 0.0, 0.0], 0

    # 对图片进行处理
    im0 = img
    # Padded resize
    im = letterbox(im0, imgsz, stride, auto=pt)[0]
    # Convert
    im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    im = np.ascontiguousarray(im)
    t1 = time_sync()
    im = torch.from_numpy(im).to(device)
    im = im.half() if half else im.float()  # uint8 to fp16/32
    im /= 255  # 0 - 255 to 0.0 - 1.0
    if len(im.shape) == 3:
        im = im[None]  # expand for batch dim
    t2 = time_sync()
    dt[0] += t2 - t1

    # Inference
    # 预测
    pred = model(im, augment=augment, visualize=visualize)
    t3 = time_sync()
    dt[1] += t3 - t2

    # NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
    dt[2] += time_sync() - t3

    # 用于存放结果
    detections = []

    # Process predictions
    for i, det in enumerate(pred):  # per image 每张图片
        seen += 1
        if len(det):
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
            # 写入结果
            for *xyxy, conf, cls in reversed(det):
                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4))).view(-1).tolist()
                xywh = [round(x) for x in xywh]
                xywh = [xywh[0] - xywh[2] // 2, xywh[1] - xywh[3] // 2, xywh[2],
                        xywh[3]]  # 检测到目标位置，目标框[x,y,x+w,y+h],
                cls = names[int(cls)]
                conf = float(conf)
                detections.append({'class': cls, 'conf': conf, 'position': xywh, })
    # 输出结果
    for i in detections:
        print(i)

    # 推测的时间
    LOGGER.info(f'({t3 - t2:.3f}s)')
    return detections

count = 1

cap = cv2.VideoCapture(1 + cv2.CAP_DSHOW)
width = 2560
height = 720
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

img_channels = 3

# 建立BM滑动窗口
# cv2.namedWindow('SGBM')
# cv2.createTrackbar('num', 'SGBM', 2, 10, lambda x: None) # 2
# cv2.createTrackbar('blockSize', 'SGBM', 2, 100, lambda x: None) # 7

while True:
    ret, frame = cap.read()
    frame0 = frame[:, 160:1120]
    frame1 = frame[:, 1440:2400]
    frame0 = cv2.resize(frame0, (640, 480))
    frame1 = cv2.resize(frame1, (640, 480))
    img = frame0
    # 重建无畸变图
    img1_rectified = cv2.remap(frame0, left_map1, left_map2, cv2.INTER_LINEAR)
    img2_rectified = cv2.remap(frame1, right_map1, right_map2, cv2.INTER_LINEAR)
    # 转化为灰度图，方便进行视差图计算
    imageL = cv2.cvtColor(img1_rectified, cv2.COLOR_BGR2GRAY)
    imageR = cv2.cvtColor(img2_rectified, cv2.COLOR_BGR2GRAY)

    blockSize = 7
    # 设置SGBM算法参数
    # num = 2
    # num = cv2.getTrackbarPos('num', 'SGBM')
    # blockSize = cv2.getTrackbarPos('blockSize', 'SGBM')
    # if blockSize % 2 == 0:
    #     blockSize += 1
    # if blockSize < 5:
    #     blockSize = 5
    stereo = cv2.StereoSGBM_create(minDisparity=0,
                                   numDisparities=16*8,  # 16*num
                                   blockSize=7,  # blockSize
                                   P1=8 * img_channels * blockSize * blockSize,
                                   P2=32 * img_channels * blockSize * blockSize,
                                   disp12MaxDiff=-1,
                                   preFilterCap=1,
                                   uniquenessRatio=10,
                                   speckleWindowSize=100,
                                   speckleRange=100,
                                   mode=cv2.STEREO_SGBM_MODE_HH)

    # 计算视差
    disparity = stereo.compute(imageL, imageR)

    # 归一化函数算法，生成深度图（灰度图）
    disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # 生成深度图（颜色图）
    dis_color = disparity
    dis_color = cv2.normalize(dis_color, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    dis_color = cv2.applyColorMap(dis_color, 2)

    cv2.imshow('SGBM', dis_color)  # 显示视差图

    # 创建二维数组存储距离信息
    output_points = np.zeros((640 * 480, 6))  # x, y, z, r, g, b
    # 计算三维坐标数据值
    threeD = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=True)
    # 计算出的threeD，需要乘以16，才等于现实中的距离
    threeD = threeD * 16

    # 检测图像
    img = img1_rectified
    msg = detect(img)
    send_data(obj, msg)
    if len(msg) != 0:
        cv2.imwrite('./runs/res/tmp' + str(count) + '.jpg', img)
        print("tmp%d.jpg saved successfully!" % count)
        count += 1
    if count >= 100:
        count = 1
    cv2.imshow('test', img)

    a = cv2.waitKey(1)
    if a == 27:
        break
