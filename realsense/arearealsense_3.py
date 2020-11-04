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
from datetime import datetime, timedelta, timezone
import os
import usb


ID_VENDOR_REALSENSE = 0x8086 # Intel
MANUFACTURER_REALSENSE = "Intel(R) RealSense(TM)"
PRODUCT_REALSENSE = "Intel(R) RealSense(TM)"
# set uploading time range
UPLOAD_TIME_LIMIT_JST_BEGIN = 10
UPLOAD_TIME_LIMIT_JST_END = 20


es = Elasticsearch(host='133.19.62.11', port=9200,http_auth=('elastic','InfoNetworking'))

def reset_realsense_devices():
    usb_devices = usb.core.find(find_all=True)

    def is_realsense_device(dev):
        is_same_idVendor = dev.idVendor == ID_VENDOR_REALSENSE
        if not is_same_idVendor:
            return False

        is_same_manufacturer = MANUFACTURER_REALSENSE in dev.manufacturer
        is_same_product = PRODUCT_REALSENSE in dev.product

        return is_same_manufacturer and is_same_product

    realsense_devices = filter(is_realsense_device, usb_devices)

    for dev in realsense_devices:
        dev.reset()

        
def uploadSensorValues(value):
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

def imagesaver(img,name):
    nowtime = datetime.now()
    savedir = os.getcwd()


    if not os.path.exists(os.path.join(savedir)):
        os.mkdir(os.path.join(savedir))
        print("MAKE_DIR: " + savedir)
    #年のフォルダを作成
    savedir += datetime.now().strftime("/%Y")
    if not os.path.exists(os.path.join(savedir)):
        os.mkdir(os.path.join(savedir))
        print("MAKE_DIR: " + savedir)
    #月のフォルダ作成
    savedir += nowtime.strftime("/%m")
    if not os.path.exists(os.path.join(savedir)):
        os.mkdir(os.path.join(savedir))
        print("MAKE_DIR: " + savedir)

    #日のフォルダを生成
    savedir += nowtime.strftime("/%d")
    if not os.path.exists(os.path.join(savedir)):
        os.mkdir(os.path.join(savedir))
        print("MAKE_DIR: " + savedir)

    # 時間_分_秒のフォルダを生成

    savefile = savedir
    datenow = datetime.now()

    saveFileName = datenow.strftime("%Y%m%d_%H%M%S" + name + ".png")
    saveFileName = os.path.join(savedir, saveFileName)

    cv2.imwrite(saveFileName, img)
    print(str(savedir) +"に保存しました")


total = 0

while True:
    
    try:
            # reset usb connection of realsense devices
        
        JST = timezone(timedelta(hours=+9),'JST')
        if UPLOAD_TIME_LIMIT_JST_BEGIN <= datetime.now(JST).hour <= UPLOAD_TIME_LIMIT_JST_END:
            print("continue")
            #JST = timezone(timedelta(hours=+9), 'JST')
        else:
            print("out of time!")
            time.sleep(60*60)


        reset_realsense_devices()   
                


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

        #   depth * depth_scale
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print(depth_scale)
        clipping_distance_in_meters = 0.7 # 
        clipping_distance = clipping_distance_in_meters / depth_scale


        align_to = rs.stream.color
        align = rs.align(align_to)

        # (Color & Depth)
    
        for i in range(100):
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

        if not depth_frame or not color_frame:
            print("error")


        
        time.sleep(3)
        color_image = np.asanyarray(color_frame.get_data())
        color_image2 = cv2.GaussianBlur(color_image, (5,5), 0)
        depth_image = np.asanyarray(depth_frame.get_data())

        #print(depth_image)
        np.set_printoptions(threshold=10000000)
        #print(depth_image)
        print("debug")



        grey_color = 153
        imagesaver(color_image,"original")
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image))
        bg_removed = np.where((depth_image_3d > clipping_distance) | \
            (depth_image_3d <= 0), grey_color, color_image)
        #bg_removed_depth = np.where(bg_removed == grey_color,depth_image,0)





        #cv2.imshow("bg_removed",bg_removed)
        
        img_HSV = cv2.cvtColor(bg_removed, cv2.COLOR_BGR2HSV)
        #cv2.imshow("img_HSV",img_HSV)
        # smoothing
        #img_HSV = cv2.GaussianBlur(img_HSV, (5, 5), 3)

        lower_green = np.array([80/360*179, 0, 40])
        upper_green = np.array([130/360*179, 255, 255])
        green_mask = cv2.inRange(img_HSV, lower_green, upper_green)

        # crop green area
        img_green_masked = cv2.bitwise_and(color_image, color_image, mask=green_mask)


        #hsv_mask = cv2.inRange(img_HSV, hsvLower, hsvUpper)   
        #cv2.imshow("hsv_mask",img_green_masked)
        #result = cv2.bitwise_and(bg_removed, bg_removed, mask=hsv_mask) # ??????????
        #cv2.imshow("img_greenarea",result)

        imagesaver(img_green_masked,"img_green_masked")
        img_HSV = cv2.cvtColor(img_green_masked, cv2.COLOR_BGR2HSV)
        img_H, img_S, img_V = cv2.split(img_HSV)
        _thre, img_leaf = cv2.threshold(img_H, 0,1, cv2.THRESH_BINARY)


        mask_depth = img_leaf * depth_image
    
        mask_depth = mask_depth / 640

        mask_depth_square = np.square(mask_depth)
    
        leafarea = np.sum(mask_depth_square)/100
        print(leafarea)
        uploadSensorValues(leafarea)


        # kernel = np.ones((10,10),np.uint8)
        # #dst = cv2.dilate(img_leaf,kernel,iterations = 1)
        # dst = cv2.morphologyEx(img_leaf,cv2.MORPH_CLOSE, kernel)

        #
        # find tulips
        contours, hierarchy = cv2.findContours(img_leaf, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        #print(contours)
        #print(hierarchy)
        total = 0
        
        #print(hierarchy)
        for i in range(0, len(contours)):
            #print(i)
            if len(contours[i]) > 0:

            # remove small objects
                if cv2.contourArea(contours[i]) < 50:
                    continue
                cv2.polylines(color_image, contours[i], True, (random.randint(0, 255),255,255), 2)
                # if hierarchy[0][i][3] != -1:
                #     cv2.polylines(color_image, contours[i], True, (random.randint(0, 255),255,255), 2)cv2.drawContours(color_image, contours, i, 255, -1)
            
            
            #print(hierarchy[0][i][0:3])
            #print(cv2.contourArea(contours[i]))
            total += cv2.contourArea(contours[i])
        imagesaver(color_image,"line")

        #????print(total)

        
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.05), cv2.COLORMAP_JET)
        # images = np.hstack((bg_removed, depth_colormap))
        # cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', images)
        #cv2.imshow("a",mask_depth)
        #cv2.imshow("leaf",color_image)
        #画像を保存
        #西暦と月のフォルダを作成
        time.sleep(60*30)
        print("fin")

        if cv2.waitKey(1) & 0xff == 27:
            print(stop)

    finally:
        # ?????????
        #pipeline.stop()
        cv2.destroyAllWindows()
