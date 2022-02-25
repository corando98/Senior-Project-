# Copyright 2019 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KqIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A demo that runs object detection on camera frames using OpenCV.

TEST_DATA=../all_models

Run face detection model:
python3 detect.py \
  --model ${TEST_DATA}/mobilenet_ssd_v2_face_quant_postprocess_edgetpu.tflite

Run coco model:
python3 detect.py \
  --model ${TEST_DATA}/mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite \
  --labels ${TEST_DATA}/coco_labels.txt

"""
import argparse
import cv2
import os

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

import RPi.GPIO as GPIO
import keyboard
import time

IOActions = [["FWleft", 13, 13, 15], #White
             ["RWleft", 15, 18, 22], 
             ["FWright", 16, 33, 35], #Green
             ["RWright", 18, 38, 40]]

GPIO.setmode(GPIO.BOARD)
# Setup Pins:
for i in range(4):
    GPIO.setup(IOActions[i][1], GPIO.OUT)
print('Pins initialized...')

def reset():
    for i in range(4):
        GPIO.output(IOActions[i][1], GPIO.LOW)
    print('Setting all motors to LOW')

def reverse():
    #Moves left and right side of the motors forward
    GPIO.output(IOActions[0][1], GPIO.LOW)
    GPIO.output(IOActions[1][1], GPIO.HIGH)
    GPIO.output(IOActions[2][1], GPIO.HIGH)
    GPIO.output(IOActions[3][1], GPIO.LOW)
    
def forward():
    #Moves left and right side of the motors forward
    
    GPIO.output(IOActions[0][1], GPIO.HIGH)
    GPIO.output(IOActions[1][1], GPIO.LOW)
    GPIO.output(IOActions[2][1], GPIO.LOW)
    GPIO.output(IOActions[3][1], GPIO.HIGH)
    
    
def turn(direction):
    
    # direction negative is left, positive is right
    if(direction > 0):

        GPIO.output(IOActions[0][1], GPIO.HIGH)
        GPIO.output(IOActions[1][1], GPIO.LOW)
        GPIO.output(IOActions[2][1], GPIO.HIGH)
        GPIO.output(IOActions[3][1], GPIO.LOW)

    # Left
    elif (direction < 0):
        GPIO.output(IOActions[0][1], GPIO.LOW)
        GPIO.output(IOActions[1][1], GPIO.HIGH)
        GPIO.output(IOActions[2][1], GPIO.LOW)
        GPIO.output(IOActions[3][1], GPIO.HIGH)
    

def main():
    default_model_dir = 'all_models'
    default_model = 'efficientdet-lite-litter_edgetpuv4.tflite'
    default_labels = 'litter-labels.txt'
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', help='.tflite model path',
                        default=os.path.join(default_model_dir,default_model))
    parser.add_argument('--labels', help='label file path',
                        default=os.path.join(default_model_dir, default_labels))
    parser.add_argument('--top_k', type=int, default=3,
                        help='number of categories with highest score to display')
    parser.add_argument('--camera_idx', type=int, help='Index of which video source to use. ', default = 0)
    parser.add_argument('--threshold', type=float, default=0.1,
                        help='classifier score threshold')
    args = parser.parse_args()

    print('Loading {} with {} labels.'.format(args.model, args.labels))
    interpreter = make_interpreter(args.model)
    interpreter.allocate_tensors()
    labels = read_label_file(args.labels)
    inference_size = input_size(interpreter)

    cap = cv2.VideoCapture(args.camera_idx)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        cv2_im = frame

        cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
        cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
        run_inference(interpreter, cv2_im_rgb.tobytes())
        objs = get_objects(interpreter, args.threshold)[:args.top_k]
        cv2_im = append_objs_to_img(cv2_im, inference_size, objs, labels)
        
        cv2.imshow('frame', cv2_im)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            GPIO.cleanup()
            break
        
        


    cap.release()
    cv2.destroyAllWindows()

def append_objs_to_img(cv2_im, inference_size, objs, labels):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    
    
    
    trackmultiple = True
    
#     Track only one object at a time, for pickup by robot.
    cv2_im = cv2.line(cv2_im, (320, 0), (320, 480), (255, 0, 0), 1)
    if len(objs) != 0:
        
        bbox = objs[0].bbox.scale(scale_x, scale_y)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)
        percent = int(100 * objs[0].score)
        #This is our tracking threshold
        if percent > 40 :
            
            label = '{}% {} Tracking'.format(percent, labels.get(objs[0].id, objs[0].id))
            print(labels.get(objs[0].id))
            if labels.get(objs[0].id) != 'Carton':
                obj_midsection = ((x0+x1)/2)
                if obj_midsection > 330 and obj_midsection < 410.0:
                    reset()
                    color = (255, 255, 0)
                    obj_box_height = (y1 - y0)
                    
                    if (obj_box_height > height / 2) and percent > 40:
                        print('Object infront stop moving')
                        # Check with ultrasonic sensor
                        reset()
                    else:
                        print('Move forward')
                        forward()
                elif obj_midsection > 410:
                    color = (0, 255, 0)
                    print('Forward right motor')
                    turn(1)
                elif obj_midsection < 330:
                    color = (0, 0, 255)
                    print('Forward left motor')
                    turn(-1)
                    
                
                cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), color, 2)
                cv2_im = cv2.putText(cv2_im, label, (x0, y0+30),
                             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
#                 cv2_im = cv2.putText(cv2_im, 'Currently Tracking', (x0, y0+60), 
    
#     Track multiple objects at a time, we want to focus on one at a time, preferably with the
#     highest confidence score.
    if trackmultiple:
        #Index 0 is always the highest confidence, if we want to still show the remaining,
        #we iterate over after 1.
        for i in range(1, len(objs)):
            bbox = objs[i].bbox.scale(scale_x, scale_y)
            x0, y0 = int(bbox.xmin), int(bbox.ymin)
            x1, y1 = int(bbox.xmax), int(bbox.ymax)

            percent = int(100 * objs[i].score)
            if percent > 30:
                
                label = '{}% {}'.format(percent, labels.get(objs[i].id, objs[i].id))
                
                cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (255,255,255), 2)
                cv2_im = cv2.putText(cv2_im, label, (x0, y0+30),
                             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
                    

            
    return cv2_im

if __name__ == '__main__':
    reset()
    main()
    
