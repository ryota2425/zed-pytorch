import numpy as np
import cv2
import time
import json
import base64
import requests

def send_image(img):
  # 画像を送信可能な形式に変換してJSONに格納
  _, encimg = cv2.imencode(".png", img)
  img_str = encimg.tostring()
  img_byte = base64.b64encode(img_str).decode("utf-8")
  img_json = json.dumps({'image': img_byte}).encode('utf-8')

  # HTTPリクエストを送信
  response = requests.post("http://192.168.11.12:8082/save", data=img_json)
  #print('{0} {1}'.format(response.status_code, json.loads(response.text)["message"]))


def send_info(id,categories,ver,hor):
  # 画像を送信可能な形式に変換してJSONに格納
  sendjson = {}
  sendjson["id"] = id
  sendjson["horizon"] = hor
  #forecast["prob-precip"] = float(prob_precip[itr].text.strip())
  sendjson["vertical"] = ver
  sendjson["categories"] = categories
  outdata = json.dumps(sendjson)
  # HTTPリクエストを送信
  response = requests.post("http://192.168.11.12:8082/Info", data=outdata)
  #print('{0} {1}'.format(response.status_code, json.loads(response.text)["message"]))

if __name__ == '__main__':
  cap = cv2.VideoCapture(0)
  cap.set(cv2.CAP_PROP_FPS, 30)
  i = 0
  while True:
    _, img = cap.read()
    if i % 5 == 0:
      send_image(img)
    i += 1