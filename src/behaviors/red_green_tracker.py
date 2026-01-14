from IPython import get_ipython
import numpy as np
import cv2 as cv
from urllib.request import urlopen
from urllib.error import URLError
import socket
import sys
import json
import re
import time

cv.namedWindow('Camera')
cv.moveWindow('Camera', 0, 0)
cmd_no = 0

def capture():
    global cmd_no
    cmd_no += 1
    print(str(cmd_no) + ': capture image')
    try:
        cam = urlopen('http://192.168.4.1/capture', timeout=1.0)
        img_data = cam.read()
    except URLError as e:
        print("Camera error:", e)
        return None

    img = np.asarray(bytearray(img_data), dtype='uint8')
    img = cv.imdecode(img, cv.IMREAD_UNCHANGED)
    if img is not None:
        cv.imshow('Camera', img)
        cv.waitKey(1)
    return img

off = [0.007, 0.022, 0.091, 0.012, -0.011, -0.05]

def cmd(sock, do, what='', where='', at=''):
    global cmd_no
    cmd_no += 1
    msg = {"H": str(cmd_no)}  # dictionary

    if do == 'move':
        msg["N"] = 3
        what = ' car '
        if where == 'forward':
            msg["D1"] = 3
        elif where == 'back':
            msg["D1"] = 4
        elif where == 'left':
            msg["D1"] = 1
        elif where == 'right':
            msg["D1"] = 2
        msg["D2"] = at  # at is speed here
        where = where + ' '
    elif do == 'stop':
        msg.update({"N": 1, "D1": 0, "D2": 0, "D3": 1})
        what = ' car'
    elif do == 'rotate':
        msg.update({"N": 5, "D1": 1, "D2": at})  # at is an angle here
        what = ' ultrasonic unit'
        where = ' '
    elif do == 'measure':
        if what == 'distance':
            msg.update({"N": 21, "D1": 2})
        elif what == 'motion':
            msg["N"] = 6
        what = ' ' + what
    elif do == 'check':
        msg["N"] = 23
        what = ' off the ground'

    msg_json = json.dumps(msg)
    print(str(cmd_no) + ': ' + do + what + where + str(at), end=': ')

    try:
        sock.sendall(msg_json.encode())
    except OSError as e:
        print('Send error:', e)
        return None

    data = ''
    try:
        while '_' not in data:
            chunk = sock.recv(1024)
            if not chunk:
                print('Connection closed by car')
                return None
            data += chunk.decode()
    except socket.timeout:
        print('Timeout waiting for response')
        return None
    except OSError as e:
        print('Receive error:', e)
        return None

    m = re.search('_(.*)}', data)
    if not m:
        print('Bad response:', data)
        return None

    res = m.group(1)
    if res == 'ok' or res == 'true':
        res = 1
    elif res == 'false':
        res = 0
    elif msg.get("N") == 6:
        res = res.split(",")
        res = [int(x)/16384 for x in res]  # convert to units of g
        res[2] = res[2] - 1  # subtract 1G from az
        res = [round(res[i] - off[i], 4) for i in range(6)]
    else:
        res = int(res)

    print(res)
    return res

ip = "192.168.4.1"
port = 100

def connect_car():
    print('Connect to {0}:{1}'.format(ip, port))
    sock = socket.socket()
    sock.settimeout(3.0)
    try:
        sock.connect((ip, port))
    except Exception as e:
        print('Connect error:', e)
        return None
    print('Connected!')

    print('Receive from {0}:{1}'.format(ip, port))
    try:
        data = sock.recv(1024).decode()
    except Exception as e:
        print('Error reading heartbeat:', e)
        sock.close()
        return None
    print('Received: ', data)
    return sock

car = connect_car()
if car is None:
    sys.exit()

def safe_cmd(do, what='', where='', at=''):
    global car
    res = cmd(car, do=do, what=what, where=where, at=at)
    if res is not None:
        return res

    print("Attempting to reconnect to car...")
    try:
        car.close()
    except:
        pass

    car = connect_car()
    if car is None:
        print("Reconnection failed.")
        return None

    return cmd(car, do=do, what=what, where=where, at=at)

def detect_red_sign(frame):
    """
    Detect a red traffic sign in the upper part of the image.
    Returns True if a sufficiently large red blob is found.
    """
    if frame is None:
        return False

    h, w = frame.shape[:2]

    roi = frame[0:int(h * 0.5), :]

    hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 100, 80])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 80])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv.inRange(hsv, lower_red2, upper_red2)
    mask = cv.bitwise_or(mask1, mask2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    cv.imshow('RedMask', mask)
    cv.waitKey(1)

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False

    largest = max(contours, key=cv.contourArea)
    area = cv.contourArea(largest)

    return area > 800

def detect_green_sign(frame):
    """
    Detect a green traffic sign in the upper part of the image.
    Returns True if a sufficiently large green blob is found.
    """
    if frame is None:
        return False

    h, w = frame.shape[:2]
    roi = frame[0:int(h * 0.5), :]

    hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

    lower_green = np.array([40, 80, 60])
    upper_green = np.array([85, 255, 255])

    mask = cv.inRange(hsv, lower_green, upper_green)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    cv.imshow('GreenMask', mask)
    cv.waitKey(1)

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False

    largest = max(contours, key=cv.contourArea)
    area = cv.contourArea(largest)

    return area > 800


forward_speed = 140
LOOP_DT = 0.05  # min loop time

car_mode = 'go'  # 'go' or 'stop'

lift_count = 0
lift_required = 3  # require 3 "lifted" readings in a row to stop the script

while True:
    start_time = time.time()

    frame = capture()
    if frame is None:
        print("Skipping frame due to camera error")
        safe_cmd('stop')
        continue

    lifted = safe_cmd('check')
    if lifted is None:
        print("Could not get lift state even after reconnect, stopping loop")
        break
    if lifted:
        lift_count += 1
        if lift_count >= lift_required:
            print("Car lifted -> stopping main loop")
            safe_cmd('stop')
            break
    else:
        lift_count = 0

    red_seen = detect_red_sign(frame)
    green_seen = detect_green_sign(frame)

    print(f"Red: {red_seen}, Green: {green_seen}, Mode: {car_mode}")

    if red_seen and not green_seen:
        if car_mode != 'stop':
            print("Red sign detected -> STOP")
            safe_cmd('stop')
        car_mode = 'stop'
    elif green_seen:
        if car_mode != 'go':
            print("Green sign detected -> GO")
        car_mode = 'go'

    if car_mode == 'go':
        safe_cmd('move', where='forward', at=forward_speed)
    else:
        safe_cmd('stop')

    elapsed = time.time() - start_time
    if elapsed < LOOP_DT:
        time.sleep(LOOP_DT - elapsed)

try:
    car.close()
except:
    pass

cv.destroyAllWindows()

