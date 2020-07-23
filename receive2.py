import os
import json
import cv2
import time
import base64
import numpy as np
from elasticsearch import Elasticsearch
from datetime import datetime
from flask import Flask, request, Response
app = Flask(__name__)
count = 0

# 画像を保存するフォルダの作成
image_dir = "./images"
#elasticsearchと接続
es = Elasticsearch(host='133.19.62.11', port=9200,http_auth=('elastic','InfoNetworking'))
if not os.path.isdir(image_dir):
  os.mkdir(image_dir)

@app.route('/save', methods=['POST'])
def save_image():
    # データの変換処理
    data = request.data.decode('utf-8')
    data_json = json.loads(data)
    image = data_json['image']
    image_dec = base64.b64decode(image)
    data_np = np.fromstring(image_dec, dtype='uint8')
    decimg = cv2.imdecode(data_np, 1)
    #画像を保存
       #西暦と月のフォルダを作成
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

    saveFileName = datetime.now().strftime("%Y%m%d_%H%M%S.png")
    saveFileName = os.path.join(savedir, saveFileName)
    cv2.imwrite(saveFileName, decimg)
    print(str(savedir) +"に保存しました")
    return Response(response=json.dumps({"message": "{} was saved".format(saveFileName)}), status=200)


@app.route('/Info', methods=['POST'])
def save_info():
    data = request.data.decode('utf-8')
    data_json = json.loads(data)
    prediction = {}
    prediction["date"] = time.time() * 1000 
    prediction["id"] = data_json['id']
    prediction["vertical"] = data_json['vertical']
    prediction["horizon"] = data_json['horizon']
    prediction["categories"] =  data_json['categories']
    outdata = json.dumps(prediction)
    print(outdata)
    try:
        res = es.index(index="aquaponics", doc_type='fish', body=outdata)
    except Exception as e:
        print(e)
    return Response(response=json.dumps({"message": "ok!"}), status=200)
    #return Response(response=json.dumps({"message": "{} was saved".format(id)}), status=200)
    # 画像ファイルを保存
    # global count
    # filename = "./images/image{}.png".format(count)
    # cv2.imwrite(filename, decimg)
    # count += 1

    # HTTPレスポンスを送信
    #return Response(response=json.dumps({"message": "{} was saved".format(saveFileName)}), status=200)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8082)