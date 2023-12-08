#! /usr/bin/env python
# -*- coding: UTF-8 -*-
import roslib
import rospy
import string
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2

#--------------------------------------------------------------------------
import os
import sys
from pathlib import Path

import numpy as np
import torch

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH，避免后续找不到需要导入的包
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative，绝对路径转为相对路径

from ultralytics.utils.plotting import Annotator, colors
from ultralytics.utils import ops
from utils.augmentations import letterbox

from models.common import DetectMultiBackend
from utils.general import (LOGGER, Profile, check_img_size, cv2,
                           non_max_suppression, scale_boxes)
from utils.segment.general import process_mask
from utils.torch_utils import select_device, smart_inference_mode


class YOLOv5Seg:

    def __init__(self,
                 weights=ROOT / 'yolov5s-seg.pt',  # model.pt path(s)
                 data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
                 imgsz=(640, 480),  # inference size (height, width)
                 conf_thres=0.25,  # confidence threshold
                 iou_thres=0.45,  # NMS IOU threshold
                 max_det=1000,  # maximum detections per image
                 device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
                 view_img=False,  # show results
                 save_txt=False,  # save results to *.txt
                 classes=0,  # filter by class: --class 0, or --class 0 2 3
                 agnostic_nms=False,  # class-agnostic NMS
                 augment=False,  # augmented inference
                 line_thickness=3,  # bounding box thickness (pixels)
                 hide_labels=False,  # hide labels
                 hide_conf=False,  # hide confidences
                 half=False,  # use FP16 half-precision inference
                 dnn=False,  # use OpenCV DNN for ONNX inference
                 ):

        self.imgsz = imgsz

        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.view_img = view_img
        self.max_det = max_det
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.augment = augment
        self.line_thickness = line_thickness
        self.hide_labels = hide_labels
        self.hide_conf = hide_conf

        # Load model
        self.device = select_device(device)
        self.model = DetectMultiBackend(weights, device=self.device, dnn=dnn, data=data,
                                        fp16=half)  # 根据模型后缀选择不同的深度模型加载，如pytorch/TensorRT等
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt  # 获得模型的几个属性
        imgsz = check_img_size(imgsz, s=self.stride)  # check image size，是否满足步长倍数的关系

        # Run inference
        bs = 1
        self.model.warmup(imgsz=(1 if self.pt else bs, 3, *imgsz))  # warmup
        self.dataset = None
        rospy.loginfo('Dataloader finished')

        #-----------------------------------------------------------------------------
        rospy.init_node('yolov5_seg')
        image_topic_1 = rospy.get_param('image_topic', '/camera/image')
        rospy.Subscriber(image_topic_1, Image, self.image_callback, queue_size=1, buff_size=52428800)
        image_topic_2 = rospy.get_param('result_topic', '/gray_image')
        self.image_pub = rospy.Publisher(image_topic_2, Image, queue_size=1)

    @staticmethod
    def loadimg(img0, imgsz):  # 接受opencv原始图片
        cap = None
        path = None
        img = letterbox(img0, new_shape=imgsz, stride=32, auto=True)[0]  # 原图变为指定大小
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        return path, img, img0, cap

    def detect(self,im0):
        # Dataloader
        global ros_image, mask_my
        self.dataset = self.loadimg(im0,self.imgsz)
        path = self.dataset[0]
        im = self.dataset[1]
        s = ''
        bs = 1  # batch_size，每次输入一张图片

        seen, windows, dt = 0, [], (Profile(), Profile(), Profile())  # Profile统计时间

        with dt[0]:
            im = torch.from_numpy(im).to(self.model.device)
            im = im.half() if self.model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim

        # Inference
        with dt[1]:
            pred, proto = self.model(im, augment=self.augment, visualize=False)[:2]

        # NMS
        with dt[2]:
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms,
                                       max_det=self.max_det, nm=32)

        mask_my = np.zeros(im0.shape[:2], dtype=np.uint8)
        
        # Process predictions
        for i, det in enumerate(pred):  # per image，实际只有一张照片
            # im0原始图像，im为转化后图像，在原始图像上绘制标签
            annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
            if len(det):
                # print(det)
                # scale bbox first the crop masks
                masks = process_mask(proto[i], det[:, 6:], det[:, :4], im.shape[2:], upsample=True)  # HWC
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()  # rescale boxes to im0 size

                # Print results
                for c in det[:, 5].unique():
                    n = (det[:, 5] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Mask plotting
                annotator.masks(
                    masks,
                    colors=[colors(x, True) for x in det[:, 5]],
                    im_gpu=im[i])

                masks = masks.cpu()
                # 将Tensor的通道求和，仅识别人，故将所有通道数据相加
                gray_tensor = masks.sum(dim=0)
                # 将Tensor转换为NumPy数组，并进行数据范围调整
                array = (gray_tensor * 255).byte().numpy()
                # 创建灰度图像
                image_gray = cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
                # 将图像转为原始尺寸
                image_ans = ops.scale_image(image_gray, im0.shape)
                mask_my = cv2.cvtColor(image_ans, cv2.COLOR_BGR2GRAY)

                # 展示图像
                # cv2.imshow('image', image_ans)
                # cv2.waitKey(1)

                # Write results
                for j, (*xyxy, conf, cls) in enumerate(reversed(det[:, :6])):
                    c = int(cls)  # integer class
                    label = None if self.hide_labels else (
                        self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                    annotator.box_label(xyxy, label, color=colors(c, True))

            # Stream results
            im0 = annotator.result()
            if self.view_img:
                # img1_bg = cv2.bitwise_and(im0, im0, mask=mask_my)
                # cv2.imshow('image1', img1_bg)
                cv2.imshow('str(p)', im0)
                cv2.waitKey(1)  # 1 millisecond

        # Print time (inference-only)
        LOGGER.info(f"{s}{'' if len(det) else '(no detections), '}{dt[1].dt * 1E3:.1f}ms")
        # 将灰度图像转换为 ROS 消息
        gray_ros_msg = CvBridge().cv2_to_imgmsg(mask_my, encoding="mono8")
        gray_ros_msg.header.frame_id = "livox_frame"
        gray_ros_msg.header.stamp = rospy.Time.now()
        self.image_pub.publish(gray_ros_msg)

    def image_callback(self,image):
        global ros_image
        try:
            ros_image = CvBridge().imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        with torch.no_grad():
            self.detect(ros_image)


if __name__ == '__main__':
    node = YOLOv5Seg()
    rate = rospy.Rate(30)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()
