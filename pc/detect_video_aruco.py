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

input_video = r"C:\Users\kataoka\OneDrive - Tokushima University\大学院\研究\Electreeper\Electreeper1.mp4"


# CSVファイルのヘッダー 一番左はPCの時間
csv_header = [
    "frame", "id",
    "ul_x", "ul_y",
    "ur_x", "ur_y",
    "dl_x", "dl_y",
    "dr_x", "dr_y",
    "c_x",  "c_y"
]
# ファイル名の例2021-09-01_15.00.00
csv_file = "aruco_" + datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S") + ".csv"

aruco_color = [
    (0, 255, 0),  # 緑
    (0, 0, 255),  # 赤
    (255, 0, 0),  # 青
    (255, 255, 0),
]  # シアン


# Arucoの辞書を設定
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

frame_num = 0
with open(csv_file, "w", newline="") as f:
    writer = csv.writer(f, delimiter=",")
    writer.writerow(csv_header)
    print(csv_header)

    # 軌跡用画像の初期化
    marker_trajectory = np.zeros((720, 1280, 3), np.uint8)
    
    # カーネルの設定
    kernel = np.ones((5, 5), np.uint8)

    start_time = time.perf_counter()
    # 動画ファイルを読み込む
    cap = cv2.VideoCapture(input_video)

    while True:

        # フレームを取得
        ret, frame = cap.read()

        # フレームが取得できなかった場合は終了
        if not ret:
            break

        # frameをはっきりさせる
        frame = cv2.resize(frame, (1280, 720)) # リサイズ
        frame = cv2.GaussianBlur(frame, (15, 15), 0) # ぼかし
        frame = cv2.erode(frame, kernel, iterations=1) # 収縮
        frame = cv2.dilate(frame, kernel, iterations=1) # 膨張

        # グレースケールに変換
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Arucoマーカを検出
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)

        # 見つかったマーカを描画
        gray = aruco.drawDetectedMarkers(gray, corners, ids, (0, 255, 0))
        # print(ids)


        # マーカの座標を取得
        if ids is not None:
            for i in range(len(ids)):
                # マーカの中心座標を計算
                x = int((corners[i][0][0][0] + corners[i][0][2][0]) / 2)
                y = int((corners[i][0][0][1] + corners[i][0][2][1]) / 2)
                # マーカの座標を取得
                ul_x = int(corners[i][0][0][0])
                ul_y = int(corners[i][0][0][1])
                ur_x = int(corners[i][0][1][0])
                ur_y = int(corners[i][0][1][1])
                dl_x = int(corners[i][0][3][0])
                dl_y = int(corners[i][0][3][1])
                dr_x = int(corners[i][0][2][0])
                dr_y = int(corners[i][0][2][1])
                # CSVに書き込むデータ
                csv_data = [
                    frame_num, ids[i][0],
                    ul_x, ul_y,
                    ur_x, ur_y,
                    dl_x, dl_y,
                    dr_x, dr_y,
                    x, y
                ]
                print(csv_data, end="\r")
                # マーカの中心座標を描画
                cv2.circle(frame, (x, y), 5, aruco_color[i], -1)
                # frameと合成
                # frame = cv2.addWeighted(frame, 0.8, marker_trajectory, 1, 0)
                writer.writerow(csv_data)
        else:
            print("\nMarker ga naiyo!")
        # 画像を表示
        cv2.imshow("marker trajectory", gray)

        
        #画像を保存
        #cv2.imwrite("./trajectory/electreeper" + str(framenum) + ".png", frame)
        frame_num += 1

        # キー入力を取得
        key = cv2.waitKey(1)
        if key == 27:  # ESCキーで終了
            break

# 終了処理
cap.release()
cv2.destroyAllWindows()
print("\nFile saved as " + csv_file)
