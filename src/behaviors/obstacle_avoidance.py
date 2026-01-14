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

off = [0.007,  0.022,  0.091,  0.012, -0.011, -0.05]
def cmd(sock, do, what = '', where = '', at = ''):
    global cmd_no
    cmd_no += 1
    msg = {"H":str(cmd_no)} # dictionary
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
        msg["D2"] = at # at is speed here
        where = where + ' '
    elif do == 'stop':
        msg.update({"N":1,"D1":0,"D2":0,"D3":1})
        what = ' car'
    elif do == 'rotate':
        msg.update({"N":5,"D1":1,"D2":at}) # at is an angle here
        what = ' ultrasonic unit'
        where = ' '
    elif do == 'measure':
        if what == 'distance':
            msg.update({"N":21,"D1":2})
        elif what == 'motion':
            msg["N"] = 6
        what = ' ' + what
    elif do == 'check':
        msg["N"] = 23
        what = ' off the ground'
    msg_json = json.dumps(msg)
    print(str(cmd_no) + ': ' + do + what + where + str(at), end = ': ')
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
        res = [int(x)/16384 for x in res] # convert to units of g
        res[2] = res[2] - 1 # subtract 1G from az
        res = [round(res[i] - off[i], 4) for i in range(6)]
    else:
        res = int(res)
    print(res)
    return res

ag = np.empty([0, 6])
ag_name = ['ax', 'ay', 'az', 'gx', 'gy', 'gz']
fig = plt.figure()
mgr = plt.get_current_fig_manager()
def plt_update(mot):
    global ag
    ag = np.vstack((ag, mot))
    plt.clf()
    for i in range(6):
        plt.plot(ag[:,i], label = ag_name[i])
    plt.legend(loc = 'upper left')
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

def safe_cmd(do, what = '', where = '', at = ''):
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

speed = 150
ang = [90, 10, 170]
dist = [0, 0, 0]
dist_min = 30
LOOP_DT = 0.05

safe_cmd('rotate', at = 90)
while 1:
    start_time = time.time()
    frame = capture()
    if frame is None:
        print("Skipping frame due to camera error")
        continue
    lifted = safe_cmd('check')
    if lifted is None:
        print("Could not get lift state even after reconnect, stopping loop")
        break
    if lifted:
        break
    mot = safe_cmd('measure', what = 'motion')
    if mot is None:
        print("Lost connection while reading motion (even after reconnect)")
        break
    plt_update(mot)
    dist0 = safe_cmd('measure', what = 'distance')
    if dist0 is None:
        print("Lost connection while reading distance (even after reconnect)")
        break
    dist[0] = dist0
    if dist[0] <= dist_min:
        safe_cmd('stop')
        found_dir = 0
        for i in range(1,3):
            safe_cmd('rotate', at = ang[i])
            di = safe_cmd('measure', what = 'distance')
            if di is None:
                print("Lost connection while scanning for direction")
                found_dir = 0
                break
            dist[i] = di
            if dist[i] > dist_min:
                found_dir = 1
        safe_cmd('rotate', at = 90)
        if ~found_dir:
            safe_cmd('move', where = 'back', at = speed)
            time.sleep(0.3)
        if dist[1] > dist[2]:
            safe_cmd('move', where = 'right', at = speed)
        else:
            safe_cmd('move', where = 'left', at = speed)
        time.sleep(0.3)
    safe_cmd('move', where = 'forward', at = speed)
    print("--- %s seconds ---" % (time.time() - start_time))
    elapsed = time.time() - start_time
    if elapsed < LOOP_DT:
        time.sleep(LOOP_DT - elapsed)

#%% Close socket
try:
    car.close()
except:
    pass

