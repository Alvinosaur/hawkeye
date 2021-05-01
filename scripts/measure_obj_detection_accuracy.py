import numpy as np
import cv2

from Object_detection import objectDetect
import os

pos_labels_fname = "position_labels.txt"
with open(pos_labels_fname, "r") as f:
    text = f.read()
    labels = text.split("\n")[:-1]

image_dir = "/home/alvin/drone_ws/src/hawkeye/scripts/Object_detection/live_images"
images = os.listdir(image_dir)
images.sort()
total_dist = 0
no_detect_count = 0
for i, imname in enumerate(images):
    impath = os.path.join(image_dir, imname)

    image = cv2.imread(impath)
    try:
        x, y, w, h, mask = objectDetect.detect_object(image)
    except:
        no_detect_count += 1
        # cv2.imshow('res1', image)
        # cv2.waitKey()
        continue

    # res1 = cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 5)
    # cv2.imshow('res1', res1)
    # cv2.waitKey()

    true_x, true_y = labels[i].split(" ")
    true_x, true_y = int(true_x), int(true_y)
    dist = ((true_x - x) ** 2 + (true_y - y) ** 2) ** 0.5
    total_dist += dist

print(no_detect_count)
print(total_dist / len(images))
print(len(images))
print("False negative rate: ")
