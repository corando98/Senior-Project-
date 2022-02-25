import time
import cv2
import os
import pathlib

# from edgetpu.detection.engine import DetectionEngine
from pycoral.utils import edgetpu
from pycoral.adapters import common
from pycoral.adapters import classify  # for image classification
from pycoral.adapters import detect  # for object detection
from pycoral.utils.dataset import read_label_file
from time import perf_counter
from PIL import Image
from PIL import ImageDraw


def draw_objects(draw, objs, labels):
    for obj in objs:
        bbox = obj.bbox
        draw.rectangle([(bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax)], outline="red")
        draw.text(
            (bbox.xmin + 10, bbox.ymin + 10),
            "%s\n%.2f" % (labels.get(obj.id, obj.id), obj.score),
            fill="red",
        )


def main():
    # Specify model, and labels, and test image
    # script_dir = pathlib.Path(__file__).parent.absolute()
    script_dir = "/home/pi/LitterBot/"
    model_file = os.path.join(
        script_dir, "all_models/efficientdet-lite-litter_edgetpuv4.tflite"
    )
    label_file = os.path.join(script_dir, "all_models/litter-labels.txt")
    image_file = os.path.join(script_dir, "litter-KC.jpg")

    # Parameters
    threshold = 0.2
    # Run one inference per frame, since one picture contains only one frame here we have 1
    count_inference = 3

    labels = read_label_file(label_file)
    # Init TF interpreter
    interpreter = edgetpu.make_interpreter(model_file)
    interpreter.allocate_tensors()

    image = Image.open(image_file)

    _, scale = common.set_resized_input(
        interpreter, image.size, lambda size: image.resize(size, Image.ANTIALIAS)
    )

    print("------ INFERENCE TIME -----")
    print(
        "Note: 1st inference is slow becasue it includes loading the mdoel into the EDGE TPU memory"
    )

    for _ in range(count_inference):
        start = perf_counter()
        interpreter.invoke()
        inference_time = perf_counter() - start
        objs = detect.get_objects(interpreter, threshold, scale)
        print("%.2f ms" % (inference_time * 1000))

    print("-------- Results --------")
    if not objs:
        print("No objects detected")

    for obj in objs:
        print(labels.get(obj.id, obj.id))
        print("  id:    ", obj.id)
        print("  score:    ", obj.score)
        print("  bbox:    ", obj.bbox)

    image = image.convert("RGB")
    draw_objects(ImageDraw.Draw(image), objs, labels)
    image.save("results_4.png", format="PNG")


if __name__ == "__main__":
    main()
