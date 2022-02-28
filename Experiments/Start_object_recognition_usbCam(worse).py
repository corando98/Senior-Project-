"""
LitterBot - UTRGV senior design

This script demostrates a camera, using a Raspberry Pi and a pre-compiled model

@Diego Garcia, Trey Jones, Sergio Montoya
"""


# edgetpu.detection.engine import DetectionEngine -> deprecated
from pycoral.adapters import detect
from pycoral.utils.edgetpu import make_interpreter, run_inference
from pycoral.utils.dataset import read_label_file
from pycoral.adapters import common

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

from imutils.video import FPS
from imutils.video import VideoStream

import numpy as np
import time
import sys
import os
import cv2
from threading import Thread, active_count
import paho.mqtt.client as mqtt

# Faster refreshrate and wider lens (Could be an upgrade as webcam as capped at 720p on USB (pi))
import picamera

message = "litter"
action = False

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("detection")


server = "localhost"
path = "model"

client = mqtt.Client()
client.on_connect = on_connect
client.connect(server, 1883, 60)


def runThread():
    client.on_message = on_message
    client.loop_forever()


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global message
    global action
    message = str(msg.payload)
    print(msg.topic + " " + str(msg.payload))

    if (message.find("litter") != -1) or (message.find("face") != -1):
        action = True


def process_objects(image, inferenceResults, elapsedTime, labels):
    # Results are sorted from highest confidence to lower confidence by default
    result_size = len(inferenceResults)

    for idx, obj in enumerate(inferenceResults):

        # Draw on image
        draw = ImageDraw.Draw(image)

        # Bounding box
        box = obj.bbox
        for x in range(0, 4):
            draw.rectangle(box, outline=(255, 255, 0))

        # Annotate on image with label and confidence
        display_str = labels.get(obj.id) + ": " + str(round(obj.score * 100, 2)) + "%"
        draw.text((box[0], box[1]), display_str)

        print(
            "Object ("
            + str(idx + 1)
            + " of "
            + str(result_size)
            + "): "
            + labels.get(obj.id)
            + " ("
            + str(obj.id)
            + ")"
            + ", Confidence:"
            + str(round(obj.score * 100.0, 2))
            + "%"
            + ", Elapsed:"
            + str(round(elapsedTime * 1000.0, 2))
            + "ms"
            + ",Box: "
            + str(obj.bbox)
        )
        # BBox(xmin, ymin, xmax, ymax)
        centerObj_x = (obj.bbox[0] + obj.bbox[2]) / 2
        centerObj_y = (obj.bbox[1] + obj.bbox[3]) / 2
        centerObj = [centerObj_x, centerObj_y]
        # Sending all objs found to mqtt server
        # draw.point(centerObj, fill="blue")
        # data = (
        # labels.get(obj.id)
        # + ":"
        # + str(obj.bbox[2] - obj.bbox[0])
        # + "x"
        # + str(obj.bbox[3] - obj.bbox[1])
        # )
        # data = data + "@" + str(centerObj)

        # client.publish(path, data)

    # Making sure there is a detection before calling the values
    if len(inferenceResults) > 0:
        centerObj_x = (inferenceResults[0].bbox[0] + inferenceResults[0].bbox[2]) / 2
        centerObj_y = (inferenceResults[0].bbox[1] + inferenceResults[0].bbox[3]) / 2
        centerObj = [centerObj_x, centerObj_y]
        # Sending only highest confidence item
        data = (
            "litter:"
            + labels.get(inferenceResults[0].id)
            + ":"
            + str(inferenceResults[0].bbox[2] - inferenceResults[0].bbox[0])
            + "x"
            + str(inferenceResults[0].bbox[3] - inferenceResults[0].bbox[1])
        )
        data = data + "@" + str(centerObj)

        client.publish(path, data)

    if "DISPLAY" in os.environ:
        displayimg = np.asarray(image)
        cv2.imshow("Live inference", displayimg)

    # process box to determine where should robot turn


def runModel():
    script_dir = "/home/pi/LitterBot/"
    model_file = os.path.join(
        script_dir, "all_models/efficientdet-lite-litter_edgetpuv5.tflite"
    )
    label_file = os.path.join(script_dir, "all_models/litter-labels-v5.txt")
    # load labels
    labels = read_label_file(label_file)

    picamera = False
    if picamera:
        print("Using picamera")
    else:
        print("Using USB camera")

    # Create inference engine using Google Coral
    inferenceEngine = make_interpreter(model_file)
    inferenceEngine.allocate_tensors()
    inference_size = common.input_size(inferenceEngine)
    # Tensor image input size
    print(inference_size)

    # Videostream
    vs = cv2.VideoCapture(1)  # First camera found

    fps = FPS().start()
    loop = True
    while loop and vs.isOpened() and (message.find("litter") != -1):
        try:
            start = time.perf_counter()
            # Reading frame
            retval, screenshot = vs.read()
            if not retval:
                break

            # Image preprcessing (tweak to increase performance)
            cv2_im = screenshot
            # cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
            # cv2_im_blur = cv2.blur(cv2_im, (3, 3))
            # Resizing seems to improve performance, both in speed and accuracy

            cv2_im_rgb = cv2.resize(cv2_im, inference_size)

            # Converting cv2 to pil

            im_pil = Image.fromarray(cv2_im_rgb)

            # Inference and not time taken
            startMs = time.time()
            run_inference(inferenceEngine, cv2_im_rgb.tobytes())
            inference_time = time.perf_counter() - start
            # Storing objs that match threshold
            objs = detect.get_objects(inferenceEngine, 0.4)

            process_objects(im_pil, objs, inference_time, labels)

            # Allows frame to be displayed, and waits 5ms per frame. Close window with q
            if cv2.waitKey(2) & 0xFF == ord("q"):
                fps.stop()
                break

        # Exit using CTRL+C
        except KeyboardInterrupt:
            fps.stop()
            break

    cv2.destroyAllWindows
    time.sleep(2)


def runModelFace():
    script_dir = "/home/pi/LitterBot/"
    model_file = os.path.join(
        script_dir, "all_models/mobilenet_ssd_v2_face_quant_postprocess_edgetpu.tflite"
    )
    label_file = os.path.join(script_dir, "all_models/coco_labels.txt")
    # load labels
    labels = read_label_file(label_file)

    picamera = False
    if picamera:
        print("Using picamera")
    else:
        print("Using USB camera")

    # Create inference engine using Google Coral
    inferenceEngine = make_interpreter(model_file)
    inferenceEngine.allocate_tensors()
    inference_size = common.input_size(inferenceEngine)
    # Tensor image input size
    print(inference_size)

    # Videostream
    vs = cv2.VideoCapture(1)  # First camera found

    fps = FPS().start()
    loop = True

    while loop and vs.isOpened() and (message.find("face") != -1):
        try:
            start = time.perf_counter()
            # Reading frame
            retval, screenshot = vs.read()
            if not retval:
                break

            # Image preprcessing (tweak to increase performance)
            cv2_im = screenshot
            # cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
            # cv2_im_blur = cv2.blur(cv2_im, (3, 3))
            # Resizing seems to improve performance, both in speed and accuracy

            cv2_im_rgb = cv2.resize(cv2_im, inference_size)

            # Converting cv2 to pil

            im_pil = Image.fromarray(cv2_im_rgb)

            # Inference and not time taken
            startMs = time.time()
            run_inference(inferenceEngine, cv2_im_rgb.tobytes())
            inference_time = time.perf_counter() - start
            # Storing objs that match threshold
            objs = detect.get_objects(inferenceEngine, 0.3)

            process_objects(im_pil, objs, inference_time, labels)

            # Allows frame to be displayed, and waits 5ms per frame. Close window with q
            if cv2.waitKey(2) & 0xFF == ord("q"):
                fps.stop()
                break

        # Exit using CTRL+C
        except KeyboardInterrupt:
            fps.stop()
            break

    cv2.destroyAllWindows
    time.sleep(2)


if __name__ == "__main__":
    t2 = Thread(target=runThread)
    t2.setDaemon(True)
    t2.start()

    while True:
        if action == True:
            action = False
            if message.find("litter") != -1:
                runModel()
            if message.find("face") != -1:
                runModelFace()
