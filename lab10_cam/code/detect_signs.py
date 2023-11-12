# YOLOv5 🚀 by Ultralytics, AGPL-3.0 license
"""
Demo of running YOLOv5 for sign detection on an image.  
Code simplified from detect.py in YOLOv5

Requires cloning of YOLOv5 repo: https://github.com/ultralytics/yolov5
"""

import argparse
import csv
import os
import platform
import sys
from pathlib import Path
import numpy as np
import torch

# Adjust the following to point to the YOLOv5 repo:
FILE = Path(__file__).resolve()
ROOT = FILE.parents[2] / 'yolov5'  # YOLOv5 root directory -- adjust this
if not ROOT.is_dir():
    raise ValueError(f"No yolov5 repo located here: {ROOT}")
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from ultralytics.utils.plotting import Annotator, colors, save_one_box

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.torch_utils import select_device, smart_inference_mode

def run_on_image(weights, # .pt file
                 imgname, # input image name
                 imgsz = (256,256), # inference size (height, width)
                 device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
                 ):

    # Load model
    print('Loading model')
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=False, data='coco128.yaml', fp16=False)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Read image
    print(f'Reading image {imgname}')
    im0 = cv2.imread( imgname )
    if im0 is None:
        print('Warning, unable to read:', imgname)
    img = cv2.resize(im0,imgsz)  # convert to target size
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # convert to RGB
    img = img.transpose(2,0,1)  # Pytorch needs format: channels x height x width
    img = np.ascontiguousarray(img)

    # convert image to Pytorch tensor
    im = torch.from_numpy(img).to(model.device)
    im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
    im /= 255  # 0 - 255 to 0.0 - 1.0
    im = im[None]  # Add extra dimension for convolutions

    print(f'Input image shape: {im.shape}' )
    
    # Inference
    print('Doing inference')
    pred = model(im)

    # NMS
    print('Doing NMS')
    pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45)

    print('Processing predictions')
    # Process predictions
    det = pred[0]   # Single image so single prediction
    # Rescale boxes from img_size to im0 size
    det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

    print('Creating annotator')    
    annotator = Annotator(im0, line_width=1, example=str(names))
    for *xyxy, conf, cls in reversed(det):
        c = int(cls)  # integer class
        label = f'{names[c]} {conf:.2f}'
        annotator.box_label(xyxy, label, color=colors(c, True))

    print('Showing output')
    imout = annotator.result()
    cv2.namedWindow('Detection', cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
    cv2.resizeWindow('Detection', imout.shape[1], imout.shape[0])
    cv2.imshow('Detection', imout)
    cv2.waitKey()  # Press any key when complete    
    
    cv2.destroyAllWindows()
            

if __name__ == '__main__':
    
    tcpath = '/egr/courses/unix/ECE/434/course_files/signs_256.pt'
    hpccpath = '/mnt/research/ece_cse_434/signs/signs_256.pt'
    path = tcpath if os.path.exists(tcpath) else hpccpath
    
    run_on_image(weights=path,
                 imgname='signs.png')
