# シリアル通信を受信してCSVファイルに保存する

import serial
import csv
import datetime

# CSVファイルのヘッダー
csv_header = ['time', 'Target', 'Resistance', 'PWM', 'Time']
# ファイル名の例2021-09-01_15.00.00
csv_file = datetime.datetime.now().strftime('%Y-%m-%d_%H.%M.%S') + '.csv'
# 除外する文字列
ignore_str = ['INFO', 'WARNING', 'ERROR']

# シリアル通信の設定
device = '/dev/ttyACM0'
baudrate = 115200
ser = serial.Serial(device, baudrate)

# CSVファイルの作成
try:
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f, delimiter=',')
        writer.writerow(csv_header)
        print (csv_header)
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line and not any(ignore in line for ignore in ignore_str):
                print(line)
                writer.writerow(line.split(','))
except KeyboardInterrupt:
    print('File saved as ' + csv_file)