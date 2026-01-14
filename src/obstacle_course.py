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

def cmd(sock, do='', what='', where='', at=''):
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

    try:
        N = msg.get("N")

        if res == 'ok' or res == 'true':
            res = 1
        elif res == 'false':
            res = 0
        elif N == 6:
            # Motion data: always 6 comma-separated ints
            parts = res.split(',')
            vals = [int(x) for x in parts]
            if len(vals) != 6:
                print("Unexpected motion length:", vals)
                return None
            vals = [v / 16384 for v in vals]
            vals[2] = vals[2] - 1  # subtract 1G from az
            res = [round(vals[i] - off[i], 4) for i in range(6)]
        elif N == 21:
            # Distance: should be a single integer (take first token if commas appear)
            first = res.split(',')[0]
            res = int(first)
        elif ',' in res:
            # Generic list of ints
            parts = res.split(',')
            res = [int(x) for x in parts]
        else:
            # Simple integer reply
            res = int(res)

    except ValueError as e:
        print("Parse error:", e, "res:", res)
        return None

    print(res)
    return res

ag = np.empty([0, 6])
ag_name = ['ax', 'ay', 'az', 'gx', 'gy', 'gz']
fig = plt.figure()
mgr = plt.get_current_fig_manager()

def plt_update(mot):
    global ag
    try:
        mot_arr = np.array(mot).reshape(1, -1)
    except Exception as e:
        print("plt_update reshape error:", e, "mot:", mot)
        return

    if mot_arr.shape[1] != 6:
        print("plt_update: unexpected mot shape", mot_arr.shape)
        return

    ag = np.vstack((ag, mot_arr))
    plt.clf()
    for i in range(6):
        plt.plot(ag[:, i], label=ag_name[i])
    plt.legend(loc='upper left')
    plt.pause(0.01)
    plt.show()

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
    if area < 200:   # small threshold so thin tape still counts
        return None

    M = cv.moments(largest)
    if M['m00'] == 0:
        return None

    cx = int(M['m10'] / M['m00'])
    offset = (cx - w / 2) / (w / 2)
    return offset


BLACK_AREA_THRESH = 1500  # need a reasonably large black patch

def detect_red_sign(frame):
    """
    Detect a BLACK paper on the ground in front of the car (used as STOP).
    Returns True if a sufficiently large dark blob is found on the floor region.
    """
    if frame is None:
        return False

    h, w = frame.shape[:2]
    y0 = int(h * 0.45)
    y1 = h
    x0 = int(w * 0.2)
    x1 = int(w * 0.8)
    roi = frame[y0:y1, x0:x1]

    hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 80])

    mask = cv.inRange(hsv, lower_black, upper_black)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    cv.imshow('BlackMask', mask)
    cv.waitKey(1)

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False

    largest = max(contours, key=cv.contourArea)
    area = cv.contourArea(largest)
    print(f"Black area: {area}")

    return area > BLACK_AREA_THRESH

def detect_green_sign(frame):
    """
    Detect a green paper on the ground in front of the car.
    Returns True if a sufficiently large green blob is found on the floor region.
    """
    if frame is None:
        return False

    h, w = frame.shape[:2]
    y0 = int(h * 0.45)
    y1 = h
    x0 = int(w * 0.2)
    x1 = int(w * 0.8)
    roi = frame[y0:y1, x0:x1]

    hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

    lower_green = np.array([35, 50, 50])
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
    print(f"Green area: {area}")

    return area > 800


ORANGE_AREA_THRESH = 8000

def detect_orange_finish(frame):
    """
    Detect an orange finish area on the ground near the bottom of the image.
    Returns True only when a large orange blob is seen (car is right on top).
    """
    if frame is None:
        return False

    h, w = frame.shape[:2]
    roi = frame[int(h * 0.4):h, :]

    hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

    lower_orange = np.array([5, 100, 80])
    upper_orange = np.array([25, 255, 255])

    mask = cv.inRange(hsv, lower_orange, upper_orange)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    cv.imshow('OrangeMask', mask)
    cv.waitKey(1)

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False

    largest = max(contours, key=cv.contourArea)
    area = cv.contourArea(largest)

    return area > ORANGE_AREA_THRESH

base_speed = 90
turn_speed = 110
LOOP_DT = 0.05

NEUTRAL_THRESHOLD = 0.05
CORNER_OFFSET = 0.45

speed = 150
ang = [90, 10, 170]
dist = [0, 0, 0]
dist_min = 30

car_mode = 'go'

lift_count = 0
lift_required = 3

safe_cmd('rotate', at=90)


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

    if detect_orange_finish(frame):
        print("Orange finish detected -> STOP and end course")
        safe_cmd('stop')
        break

    black_seen = detect_red_sign(frame)   # now actually "black"
    green_seen = detect_green_sign(frame)
    print(f"Black(stop): {black_seen}, Green: {green_seen}, Mode: {car_mode}")

    if green_seen:
        if car_mode != 'go':
            print("Green paper detected -> GO")
        car_mode = 'go'
    elif black_seen:
        if car_mode != 'stop':
            print("Black paper detected -> STOP")
            safe_cmd('stop')
        car_mode = 'stop'

    mot = safe_cmd('measure', what='motion')
    if mot is None:
        print("Lost connection while reading motion (even after reconnect)")
        break

    if car_mode == 'stop':
        safe_cmd('stop')
    else:
        dist0 = safe_cmd('measure', what='distance')
        if dist0 is None:
            print("Lost connection while reading distance (even after reconnect)")
            break
        dist[0] = dist0

        if dist[0] <= dist_min:
            print("Obstacle detected ahead, distance:", dist[0])
            safe_cmd('stop')

            found_dir = 0
            for i in range(1, 3):
                safe_cmd('rotate', at=ang[i])
                di = safe_cmd('measure', what='distance')
                if di is None:
                    print("Lost connection while scanning for direction")
                    found_dir = 0
                    break
                dist[i] = di
                if dist[i] > dist_min:
                    found_dir = 1

            safe_cmd('rotate', at=90)

            if not found_dir:
                print("No clear side found, backing up")
                safe_cmd('move', where='back', at=speed)
                time.sleep(0.3)

            if dist[1] > dist[2]:
                print("Turning right around obstacle")
                safe_cmd('move', where='right', at=speed)
            else:
                print("Turning left around obstacle")
                safe_cmd('move', where='left', at=speed)
            time.sleep(0.3)


        else:
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

