# Setup (Elegoo Robot Car V4.0 + Python)

This project controls the **Elegoo Smart Robot Car V4.0** from a laptop over Wi-Fi by:
- grabbing camera frames from `http://192.168.4.1/capture`
- sending JSON control commands over TCP to `192.168.4.1:100`

## Requirements
- Elegoo Robot Car V4.0
- A laptop/desktop with Wi-Fi
- Spyder IDE running python version 3.12

## 1) Build Elegoo Robot Car V4.0
- Will need to assemble elegoo car based on instructions
- For better use of camera, using tape, slightly angle camera facing down at a 70 degree angle.

## 1) Connect to the car
1. Power on the robot car after charging.
2. On your computer, connect to the car’s Wi-Fi network (the car acts like an access point).
3. Confirm these endpoints are reachable (default for this repo):
   - Camera snapshot: `http://192.168.4.1/capture`
   - Command socket: `192.168.4.1:100`

If your car uses a different IP/port, update these variables in your scripts:
- `ip = "192.168.4.1"`
- `port = 100`

## 2) Install dependencies
From the repo root:

```bash
pip install -r requirements.txt
```

## 3) Test scripts with Elegoo Robot Car
1. Download Spyder IDE onto your device
2. Run scripts depending on which behavior you would like to test

