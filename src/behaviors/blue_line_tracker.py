from IPython import get_ipython
import numpy as np
import cv2 as cv
from urllib.request import urlopen
from urllib.error import URLError
import socket
import sys
import json
import re
import matplotlib.pyplot as plt
import time

plt.close('all')

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


def get_blue_offset(frame):
    """
    Find the horizontal offset of a blue line in the bottom half of the image.
    Returns:
      offset in [-1, 1], where 0 = centered, negative = left, positive = right,
      or None if no blue region is detected.
    """
    if frame is None:
        return None

    h, w = frame.shape[:2]

    roi = frame[int(h * 0.5):h, :]

    hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

    lower_blue = np.array([90, 40, 40])
    upper_blue = np.array([140, 255, 255])
    mask = cv.inRange(hsv, lower_blue, upper_blue)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    cv.imshow('BlueMask', mask)
    cv.waitKey(1)

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv.contourArea)
    area = cv.contourArea(largest)
    if area < 200:
        return None

    M = cv.moments(largest)
    if M['m00'] == 0:
        return None

    cx = int(M['m10'] / M['m00'])
    offset = (cx - w / 2) / (w / 2)
    return offset


base_speed = 90
turn_speed = 110
LOOP_DT = 0.05

NEUTRAL_THRESHOLD = 0.05
CORNER_OFFSET = 0.45

safe_cmd('rotate', at=90)

lift_count    = 0
lift_required = 3  # require 3 "lifted" readings in a row to stop

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

    offset = get_blue_offset(frame)
    print("Blue offset:", offset)

    if offset is None:
        print("No blue detected → stop")
        safe_cmd('stop')
    else:
        safe_cmd('move', where='forward', at=base_speed)

        abs_off = abs(offset)

        if abs_off > NEUTRAL_THRESHOLD:
            direction = 'left' if offset < 0 else 'right'

            if abs_off > CORNER_OFFSET:
                max_pulse = 0.35
                min_pulse = 0.12
            else:
                max_pulse = 0.22
                min_pulse = 0.05

            corr_time = max(min_pulse, min(max_pulse, abs_off * 0.30))

            print(f"Correct {direction} for {corr_time:.3f} s (abs_off={abs_off:.2f})")
            safe_cmd('move', where=direction, at=turn_speed)
            time.sleep(corr_time)
            safe_cmd('move', where='forward', at=base_speed)

    elapsed = time.time() - start_time
    if elapsed < LOOP_DT:
        time.sleep(LOOP_DT - elapsed)

try:
    car.close()
except:
    pass

cv.destroyAllWindows()

