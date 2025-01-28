# Arucoマーカをカメラで検出するプログラム

import cv2
import cv2.aruco as aruco
import numpy as np
import subprocess
import csv
import datetime
import time
import os

os.makedirs("./trajectory", exist_ok=True)


# CSVファイルのヘッダー 一番左はPCの時間
csv_header = [
    "pc_time",
    # "ul0_x",
    # "ul0_y",
    # "ur0_x",
    # "ur0_y",
    # "dl0_x",
    # "dl0_y",
    # "dr0_x",
    # "dr0_y",
    # "c0_x",
    # "c0_y",
    "ul1_x",
    "ul1_y",
    "ur1_x",
    "ur1_y",
    "dl1_x",
    "dl1_y",
    "dr1_x",
    "dr1_y",
    "c1_x",
    "c1_y",
    "ul2_x",
    "ul2_y",
    "ur2_x",
    "ur2_y",
    "dl2_x",
    "dl2_y",
    "dr2_x",
    "dr2_y",
    "c2_x",
    "c2_y"# ,
    # "ul3_x",
    # "ul3_y",
    # "ur3_x",
    # "ur3_y",
    # "dl3_x",
    # "dl3_y",
    # "dr3_x",
    # "dr3_y",
    # "c3_x",
    # "c3_y",
]
# ファイル名の例2021-09-01_15.00.00
csv_file = "aruco_" + datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S") + ".csv"

aruco_color = [
    (0, 255, 0),  # 緑
    (0, 0, 255),  # 赤
    (255, 0, 0),  # 青
    (255, 255, 0),
]  # シアン

# カメラの設定
cap = cv2.VideoCapture(2)  # logitech c270
cap.set(3, 1280)  # Width
cap.set(4, 720)  # Height

# Arucoの辞書を設定
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# フリッカーを防ぐためにカメラの露出を固定
# v4l2で設定を行う前にread()を1回実行しておく
_, _ = cap.read()  # <-対策としてこの1行を追加

# v4l2の設定をsubprocessを用いて実行
cmd = "v4l2-ctl -d /dev/video2 -c auto_exposure=1 -c exposure_time_absolute=30"
ret = subprocess.check_output(cmd, shell=True)

# v4l2の設定値を確認
cmd = "v4l2-ctl --list-ctrls"
ret = subprocess.check_output(cmd, shell=True)
print(ret)

framenum = 0
with open(csv_file, "w", newline="") as f:
    writer = csv.writer(f, delimiter=",")
    writer.writerow(csv_header)
    print(csv_header)

    # 軌跡用画像の初期化
    marker_trajectory = np.zeros((720, 1280, 3), np.uint8)
    
    # カーネルの設定
    kernel = np.ones((5, 5), np.uint8)

    start_time = time.perf_counter()
    while True:
        # カメラ画像を取得
        ret, frame = cap.read()
        if not ret:
            break

        # 画像をグレースケールに変換
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # クロージング処理
        gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
        # オープニング処理
        gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)

        # Arucoマーカを検出
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)
        print(ids)

        # マーカの中心座標を描画
        if ids is not None:
            csv_data = [time.perf_counter() - start_time]
            for i in range(len(ids)):
                c = corners[i][0]
                x = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                y = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)
                cv2.circle(marker_trajectory, (x, y), 5, aruco_color[ids[i][0]], -1)
                # frameと合成
                frame = cv2.addWeighted(frame, 0.8, marker_trajectory, 1, 0)
            for j in range(4):
                csv_data.append(c[j][0])
                csv_data.append(c[j][1])
            csv_data.append(x)
            csv_data.append(y)
            # print(csv_data, end="\r")
            writer.writerow(csv_data)
        else:
            print("\nMarker ga naiyo!")
        # 画像を表示
        cv2.imshow("marker trajectory", frame)
        
        #画像を保存
        #cv2.imwrite("./trajectory/electreeper" + str(framenum) + ".png", frame)
        #framenum += 1

        # キー入力を取得
        key = cv2.waitKey(1)
        if key == 27:  # ESCキーで終了
            break

# 終了処理
cap.release()
cv2.destroyAllWindows()
print("\nFile saved as " + csv_file)
