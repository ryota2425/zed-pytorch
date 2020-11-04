# -*- coding: utf-8 -*-

#############################################
##      D415 Depth?????
#############################################
import pyrealsense2 as rs
import numpy as np
import cv2
import random


total = 0
#?????(Depth/Color)???
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#?????(Color/Depth)???
#config = rs.config()
# ? ??????????
#config.enable_device_from_file('infodata.bag')
#config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

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
    while True:
        # ??????(Color & Depth)
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
       
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        color_image2 = cv2.GaussianBlur(color_image, (5,5), 0)
        depth_image = np.asanyarray(depth_frame.get_data())
        
        #print(depth_image)
        np.set_printoptions(threshold=10000000)
        #print(depth_image)
        

 
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image))
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        #bg_removed_depth = np.where(bg_removed == grey_color,depth_image,0)
        

        

        #detect tulips
        #H: 0-179, S: 0-255, V: 0-255
        img_HSV = cv2.cvtColor(bg_removed, cv2.COLOR_BGR2HSV)
        img_H, img_S, img_V = cv2.split(img_HSV)
        # _thre, img_leaf = cv2.threshold(img_H, 0,1, cv2.THRESH_BINARY)
        # ??????
        img_leaf = cv2.threshold(img_H, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

        # ???????(???)
        label = cv2.connectedComponentsWithStats(img_leaf)

        
        #?????????????????????AND??
        # mask_depth = img_leaf * depth_image
        #1???????????????(????640????)
        #mask_depth = mask_depth / 640
        mask_depth = depth_image / 640
        #?????????2????????????????
        mask_depth_square = np.square(mask_depth)
        #??????????????(mm^2=>cm^2)
        #print(np.sum(mask_depth_square)/100)

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
        # for i in range(0, len(contours)):
        #  #print(i)
        #  if len(contours[i]) > 0:
             

        #     # remove small objects
        #    if cv2.contourArea(contours[i]) < 500:
        #       continue
        #    cv2.polylines(color_image, contours[i], True, (random.randint(0, 255),random.randint(0, 255),255), 2)
        #    #print(hierarchy[0][i][0:3])

        #    #print(cv2.contourArea(contours[i]))
        #    total += cv2.contourArea(contours[i])
           # ???????????????
        n = label[0] - 1
        data = np.delete(label[2], 0, 0)
        center = np.delete(label[3], 0, 0)
        # ???????????????????????
        color_src = color_image
        # ???????????????????????
        for i in range(1, n):

           if data[i][4] >= 1000: 
                #?????????????????????????0???
                mask = np.where(label[1] == i, 1, 0)
                #?????????????????????AND??
                mask_depth = mask_depth_square * mask
                #cm^2???
                area = np.sum(mask_depth)/100
                
                # ???????????????????????
                if area >= 5:
                    x0 = data[i][0]
                    y0 = data[i][1]
                    x1 = data[i][0] + data[i][2]
                    y1 = data[i][1] + data[i][3]
                    cv2.rectangle(color_src, (x0, y0), (x1, y1), (0, 0, 255))
                    cv2.putText(color_src, "ID: " +str(i + 1), (x0, y1 + 15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
                    cv2.putText(color_src, "S: " +str(area), (x0, y1 + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
                    print("ID:",str(i+1))
                    print("S:",area)

                # # ????????????????????
                # cv2.putText(color_src, "X: " + str(int(center[i][0])), (x1 - 10, y1 + 15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))
                # cv2.putText(color_src, "Y: " + str(int(center[i][1])), (x1 - 10, y1 + 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))

        #????print(total)
        
        # ??????
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.05), cv2.COLORMAP_JET)
        # images = np.hstack((bg_removed, depth_colormap))
        # cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', images)
        cv2.imshow("a",mask_depth)
        cv2.imshow("leaf",color_image)
        
        if cv2.waitKey(1) & 0xff == 27:
            break

finally:
    # ?????????
    pipeline.stop()
    cv2.destroyAllWindows()



