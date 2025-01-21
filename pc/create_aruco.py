# _*_ coding: utf-8 _*_
import cv2
import numpy as np
from cv2 import aruco

marker_id = [0, 1, 2, 3]
# Size and offset value
size = 200
offset = 0
x_offset = y_offset = int(offset) // 2

# get dictionary and generate image
for marker in marker_id:
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    ar_img = aruco.generateImageMarker(dictionary, marker, size)

    # make white image
    img = np.zeros((size + offset, size + offset), dtype=np.uint8)
    img += 255

    # overlap image
    img[y_offset:y_offset + ar_img.shape[0], x_offset:x_offset + ar_img.shape[1]] = ar_img

    # add black border
    # border_size = 10  # size of the border
    # img_with_border = cv2.copyMakeBorder(img, border_size, border_size, border_size, border_size, cv2.BORDER_CONSTANT, value=0)

    cv2.imwrite(f"marker_{marker}.png", img)