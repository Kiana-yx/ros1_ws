#! /usr/bin/env python
# -*- coding: UTF-8 -*-
import roslib
import rospy
import string
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

#--------------------------------------------------------------------------
import argparse
import os
import platform
import sys
import time
from pathlib import Path

import numpy as np
import torch

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH，避免后续找不到需要导入的包
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative，绝对路径转为相对路径

from ultralytics.utils.plotting import Annotator, colors, save_one_box
from utils.augmentations import letterbox

from models.common import DetectMultiBackend
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, scale_segments,
                           strip_optimizer)
from utils.segment.general import masks2segments, process_mask, process_mask_native
from utils.torch_utils import select_device, smart_inference_mode


def loadimg(img0, imgsz=(640, 480)):  # 接受opencv原始图片
    cap = None
    path = None
    img = letterbox(img0, new_shape=imgsz, stride=32, auto=True)[0]  # 原图变为指定大小
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    img = np.ascontiguousarray(img)
    return path, img, img0, cap


@smart_inference_mode()
def run(
        weights=ROOT / 'yolov5s-seg.pt',  # model.pt path(s)
        data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 480),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=True,  # show results
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
    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half) # 根据模型后缀选择不同的深度模型加载，如pytorch/TensorRT等
    stride, names, pt = model.stride, model.names, model.pt  # 获得模型的几个属性
    imgsz = check_img_size(imgsz, s=stride)  # check image size，是否满足步长倍数的关系

    # Dataloader
    im0 = cv2.imread('/home/kiana/Pictures/bus.jpg')  # BGR
    dataset = loadimg(im0, imgsz)
    path = dataset[0]
    im = dataset[1]
    s = ''
    bs = 1  # batch_size，每次输入一张图片

    # Run inference
    model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup
    seen, windows, dt = 0, [], (Profile(), Profile(), Profile())    # Profile统计时间

    with dt[0]:
        im = torch.from_numpy(im).to(model.device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

    # Inference
    with dt[1]:
        pred, proto = model(im, augment=augment, visualize=False)[:2]

    # NMS
    with dt[2]:
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det, nm=32)

    # Process predictions
    for i, det in enumerate(pred):  # per image，实际只有一张照片
        # im0原始图像，im为转化后图像，在原始图像上绘制标签
        annotator = Annotator(im0, line_width=line_thickness, example=str(names))
        if len(det):
            print(det)
            # scale bbox first the crop masks
            masks = process_mask(proto[i], det[:, 6:], det[:, :4], im.shape[2:], upsample=True)  # HWC
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()  # rescale boxes to im0 size

            # Print results
            for c in det[:, 5].unique():
                n = (det[:, 5] == c).sum()  # detections per class
                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

            # Mask plotting
            annotator.masks(
                masks,
                colors=[colors(x, True) for x in det[:, 5]],
                im_gpu=im[i])

            # Write results
            for j, (*xyxy, conf, cls) in enumerate(reversed(det[:, :6])):
                c = int(cls)  # integer class
                label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                annotator.box_label(xyxy, label, color=colors(c, True))

        # Stream results
        im0 = annotator.result()
        if view_img:
            cv2.imshow('str(p)', im0)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            if cv2.waitKey(1000) == ord('q'):  # 1 millisecond
                exit()

    # Print time (inference-only)
    LOGGER.info(f"{s}{'' if len(det) else '(no detections), '}{dt[1].dt * 1E3:.1f}ms")

def image_callback(self,image):
    global ros_image
    ros_image = CvBridge().imgmsg_to_cv2(image, "bgr8")
    with torch.no_grad():
        self.detect(ros_image)


if __name__ == '__main__':
    run()
