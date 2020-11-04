import pyrealsense2 as rs
import usb

import numpy as np
import cv2
import time

ID_VENDOR_REALSENSE = 0x8086 # Intel
MANUFACTURER_REALSENSE = "Intel(R) RealSense(TM)"
PRODUCT_REALSENSE = "Intel(R) RealSense(TM)"

CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480
CAPTURE_FPS = 60

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

def get_realsense_serialnumbers(max_n=1):
    # pyrealsense2.context
    ctx = rs.context()

    # pyrealsense2.device_list
    devices = ctx.query_devices()
    serial_numbers = map(lambda device: device.get_info(rs.camera_info.serial_number), devices)

    serial_numbers_ = list(serial_numbers)[:max_n]

    return serial_numbers_

def setup_realsense_camera(device_serial_number, width, height, fps):
    # ?????(Color/Depth)???
    config = rs.config()
    config.enable_device(device_serial_number)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

    # ?????????
    pipeline = rs.pipeline()
    profile = pipeline.start(config)

    return pipeline, profile

def get_images(pipeline):
    frames = pipeline.wait_for_frames()

    # RGB
    RGB_frame = frames.get_color_frame()
    RGB_image = np.asanyarray(RGB_frame.get_data())

    # depth
    depth_frame = frames.get_depth_frame()
    depth_image = np.asanyarray(depth_frame.get_data())

    return RGB_image, depth_image

def main():
    # reset usb connection of realsense devices
    reset_realsense_devices()

    # setup pipeline
    serial_numbers = get_realsense_serialnumbers(max_n=1)
    if len(serial_numbers) == 0:
        print("Not found realsense devices")
        return

    pipeline, _ = setup_realsense_camera(serial_numbers[0], CAPTURE_WIDTH, CAPTURE_HEIGHT, CAPTURE_FPS)

    # get image and show
    i = 0
    while True:
        print(i)
        RGB_image, depth_image = get_images(pipeline)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)

        cv2.imshow("realsense image", RGB_image)
        key = cv2.waitKey(1)
        if key == 27: # ESC
            break

        i += 1

if __name__ == "__main__":
    main()
