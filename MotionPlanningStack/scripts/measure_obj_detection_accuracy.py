import numpy as np
import cv2

from Object_detection import objectDetect
import os

pos_labels_fname = "position_labels.txt"
with open(pos_labels_fname, "r") as f:
    text = f.read()
    labels = text.split("\n")[:-1]

positives_dir = "/home/alvin/drone_ws/src/hawkeye/scripts/Object_detection/positives"
positives = os.listdir(positives_dir)
positives.sort()

negatives_dir = "/home/alvin/drone_ws/src/hawkeye/scripts/Object_detection/negatives"
negatives = os.listdir(negatives_dir)

total_dist = 0
false_negative_count = 0
for i, imname in enumerate(positives):
    impath = os.path.join(positives_dir, imname)

    image = cv2.imread(impath)
    try:
        x, y, w, h, mask = objectDetect.detect_object(image)
    except:
        false_negative_count += 1

        # cv2.imshow('res1', image)
        # cv2.waitKey()
        continue

    true_x, true_y = labels[i].split(" ")
    true_x, true_y = int(true_x), int(true_y)
    dist = ((true_x - x) ** 2 + (true_y - y) ** 2) ** 0.5
    total_dist += dist
    if imname == "frame0068.jpg":
        print(imname)
        res1 = cv2.circle(image, (true_x, true_y), 5, (0, 69, 255), -1)
        cv2.imshow('res1', res1)
        cv2.imwrite("detection.png", res1)
        exit()
false_positive_count = 0
for i, imname in enumerate(negatives):
    impath = os.path.join(positives_dir, imname)

    image = cv2.imread(impath)
    try:
        x, y, w, h, mask = objectDetect.detect_object(image)
        res1 = cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 5)
        cv2.imshow('res1', res1)
        cv2.waitKey()
        false_positive_count += 1  # should return None since no target in image
    except:
        continue

print("Average pixel distance: ", total_dist / len(positives))
print("False negative rate: ", false_negative_count / len(positives))
print("False positives rate: ", false_positive_count / len(negatives))
