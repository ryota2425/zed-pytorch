# -*- coding: utf-8 -*-

#############################################
##      D415 Depth?????
#############################################
import pyrealsense2 as rs
import numpy as np
import cv2
import random

import urllib.request
import time
import json
import struct
from elasticsearch import Elasticsearch


es = Elasticsearch(host='133.19.62.11', port=9200,http_auth=('elastic','InfoNetworking'))



def uploadSensorValues(value):
  #???URL
    #sensordata = {
     #   "value": value,"date": ut
    #}
    dict = {}
    dict["leaf_area"] = value
    ut = str(time.time()*1000)
    dict["date"] = ut
    enc = json.dumps(dict)
    print(enc)
    try:
        res = es.index(index="aquaponics", doc_type='fish', body=enc)
    except Exception as e:
        print(e)



total = 0
#?????(Depth/Color)???
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#?????(Color/Depth)???
# config = rs.config()
# # ? ??????????
# config.enable_device_from_file('infodata.bag')
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# ?????????
pipeline = rs.pipeline()
profile = pipeline.start(config)

#   ??[m] = depth * depth_scale
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print(depth_scale)
clipping_distance_in_meters = 0.7 # ????
clipping_distance = clipping_distance_in_meters / depth_scale

# Align????????
align_to = rs.stream.color
align = rs.align(align_to)

try:

    # ??????(Color & Depth)
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    if not depth_frame or not color_frame:
        print("error")

    color_image = np.asanyarray(color_frame.get_data())
    color_image2 = cv2.GaussianBlur(color_image, (5,5), 0)
    depth_image = np.asanyarray(depth_frame.get_data())

    #print(depth_image)
    np.set_printoptions(threshold=10000000)
    #print(depth_image)



    grey_color = 153
    depth_image_3d = np.dstack((depth_image,depth_image,depth_image))
    bg_removed = np.where((depth_image_3d > clipping_distance) | \
        (depth_image_3d <= 0), grey_color, color_image)
    #bg_removed_depth = np.where(bg_removed == grey_color,depth_image,0)




    # detect tulips
    # H: 0-179, S: 0-255, V: 0-255
    cv2.imshow("bg_removed",bg_removed)
    img_HSV = cv2.cvtColor(bg_removed, cv2.COLOR_BGR2HSV)
    cv2.imshow("img_HSV",img_HSV)
    # smoothing
    #img_HSV = cv2.GaussianBlur(img_HSV, (5, 5), 3)
    # ???????????
    lower_green = np.array([30, 64, 0])
    upper_green = np.array([150, 255, 255])
    green_mask = cv2.inRange(img_HSV, lower_green, upper_green)

    # crop green area
    img_green_masked = cv2.bitwise_and(color_image, color_image, mask=green_mask)


    #hsv_mask = cv2.inRange(img_HSV, hsvLower, hsvUpper)    # HSV????????
    cv2.imshow("hsv_mask",img_green_masked)
    #result = cv2.bitwise_and(bg_removed, bg_removed, mask=hsv_mask) # ??????????
    #cv2.imshow("img_greenarea",result)

    ##?????????????????????AND?????????????
    img_HSV = cv2.cvtColor(img_green_masked, cv2.COLOR_BGR2HSV)
    img_H, img_S, img_V = cv2.split(img_HSV)
    _thre, img_leaf = cv2.threshold(img_H, 0,1, cv2.THRESH_BINARY)

    #?????????????????????AND??
    mask_depth = img_leaf * depth_image
    #1???????????????(????640????)
    mask_depth = mask_depth / 640
    #?????????2????????????????
    mask_depth_square = np.square(mask_depth)
    #??????????????(mm^2=>cm^2)
    leafarea = np.sum(mask_depth_square)/100
    print(leafarea)
    uploadSensorValues(leafarea)

    # # #?????????
    # kernel = np.ones((10,10),np.uint8)
    # #dst = cv2.dilate(img_leaf,kernel,iterations = 1)
    # dst = cv2.morphologyEx(img_leaf,cv2.MORPH_CLOSE, kernel)

    #
    # find tulips
    contours, hierarchy = cv2.findContours(img_leaf, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    #print(contours)
    #print(hierarchy)
    total = 0
    #print(hierarchy)
    for i in range(0, len(contours)):
        #print(i)
        if len(contours[i]) > 0:

        # remove small objects
            if cv2.contourArea(contours[i]) < 500:
                continue
            cv2.polylines(color_image, contours[i], True, (random.randint(0, 255),255,255), 2)
        #print(hierarchy[0][i][0:3])

        #print(cv2.contourArea(contours[i]))
        total += cv2.contourArea(contours[i])

    #????print(total)

    # ??????
    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.05), cv2.COLORMAP_JET)
    # images = np.hstack((bg_removed, depth_colormap))
    # cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('RealSense', images)
    cv2.imshow("a",mask_depth)
    cv2.imshow("leaf",color_image)

    if cv2.waitKey(1) & 0xff == 27:
       print(stop)

finally:
    # ?????????
    pipeline.stop()
    cv2.destroyAllWindows()
