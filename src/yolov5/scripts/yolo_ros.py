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
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from ultralytics.utils.plotting import Annotator, colors, save_one_box
from models.common import DetectMultiBackend
from utils.augmentations import letterbox
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.torch_utils import select_device, smart_inference_mode


class YOLOv5(object):
    def __init__(self,
                 weights=ROOT/ 'yolov5s.pt',
                 data=ROOT / 'data/coco128.yaml',
                 imgsz=(640, 480),
                 conf_thres=0.5,  # confidence threshold
                 iou_thres=0.45,  # NMS IOU threshold
                 max_det=1000,
                 device='',
                 vid_stride=3,
                 save_img=False):
        #-----------------------------------------------------------------------------
        self.dataset = None
        self.imgsz = imgsz
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.max_det = max_det
        self.save_img = save_img
        
        # 绘制结果时是否隐藏类别标注和置信度
        self.hide_labels = False
        self.hide_conf= False

        # Load model
        self.device = select_device(device)
        self.half = False
        self.model = DetectMultiBackend(weights, device=self.device, dnn=False, data=data, fp16=False)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size(imgsz, s=self.stride)  # check image size

        rospy.loginfo('Load model finished')
        
        # Dataloader
        self.bs = 1  # batch_size
        project = ROOT / 'runs/detect'
        self.save_dir = increment_path(Path(project) / 'exp', exist_ok=False)  # increment run
        self.save_dir.mkdir(parents=True, exist_ok=True)  # make dir
        self.model.warmup(imgsz=(1 if self.pt or self.model.triton else self.bs, 3, *self.imgsz))  # warmup
        rospy.loginfo('Dataloader finished')

        #-----------------------------------------------------------------------------
        rospy.init_node('yolov5')
        image_topic_1 = rospy.get_param('image_topic', '/camera/image')
        rospy.Subscriber(image_topic_1, Image, self.image_callback, queue_size=1, buff_size=52428800)
        image_topic_2 = rospy.get_param('result_topic', '/result_topic')
        image_pub = rospy.Publisher(image_topic_2, Image, queue_size=1)

    def loadimg(self,img0):  # 接受opencv图片
        img_size=640
        cap=None
        path=None
        img = letterbox(img0, new_shape=img_size)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        return path, img, img0, cap
    
    def detect(self,im0):
        global ros_image
        self.dataset = self.loadimg(im0)
        path = self.dataset[0]
        img = self.dataset[1]
        im0s = self.dataset[2]
        vid_cap = self.dataset[3]
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        
        if len(img.shape) == 3:
            img = img[None]  # expand for batch dim，扩增维度

        pred = self.model(img, augment=False, visualize=False)

        # NMS 根据置信度过滤，class指定仅检测人（0）
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=0, agnostic=False,
                                   max_det=self.max_det)

        # Process predictions，det每个检测框信息
        for i, det in enumerate(pred):  # per image
            p, s, im0 = path, '', im0s
            s += '%gx%g ' % img.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            
            annotator = Annotator(im0, line_width=3, example=str(self.names))
            
            # if len(det):
            if det is not None:# 识别到结果
                # print(det)
                # Rescale boxes from img_size to im0 size，将预测结果映射回原图
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, 5].unique():
                    n = (det[:, 5] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                print(s)

                for *xyxy, conf, cls in reversed(det):# 将每个结果一一对应
                    c = int(cls)  # integer class
                    label = None if self.hide_labels else (self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                    
                    annotator.box_label(xyxy, label, color=colors(c, True))
                
                # Stream results
                im0 = annotator.result()
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

    
    def image_callback(self,image):
        print('hello')
        global ros_image
        try:
            ros_image = CvBridge().imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        with torch.no_grad():
            self.detect(ros_image)


if __name__ == '__main__':
    node = YOLOv5()
    rate = rospy.Rate(30)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()