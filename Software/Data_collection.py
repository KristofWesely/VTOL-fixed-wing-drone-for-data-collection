import os
import csv
import time
import serial
# from picamera import PiCamera
from datetime import datetime
from dronekit import connect, VehicleMode
# import Adafruit_DHT
import numba as nb
import depthai as dai
import board
import adafruit_dht

# Configuration dictionary
config = {
    "camera": "Oak-D Lite",
    "iso": 200,
    "exposure_time": 0, # in microseconds, 0 for default
    "raw": False,
    "preview": True,
    "enable_dht11": False,
    "loop_frequency": 1,  # in seconds
    "arduPilot_connection": "COM7"
}

''' Packing scheme for RAW10 - MIPI CSI-2
- 4 pixels: p0[9:0], p1[9:0], p2[9:0], p3[9:0]
- stored on 5 bytes (byte0..4) as:
| byte0[7:0] | byte1[7:0] | byte2[7:0] | byte3[7:0] |          byte4[7:0]             |
|    p0[9:2] |    p1[9:2] |    p2[9:2] |    p3[9:2] | p3[1:0],p2[1:0],p1[1:0],p0[1:0] |
'''
# Optimized with 'numba' as otherwise would be extremely slow (55 seconds per frame!)
@nb.njit(nb.uint16[::1] (nb.uint8[::1], nb.uint16[::1], nb.boolean), parallel=True, cache=True)
def unpack_raw10(input, out, expand16bit):
    lShift = 6 if expand16bit else 0

   #for i in np.arange(input.size // 5): # around 25ms per frame (with numba)
    for i in nb.prange(input.size // 5): # around  5ms per frame
        b4 = input[i * 5 + 4]
        out[i * 4]     = ((input[i * 5]     << 2) | ( b4       & 0x3)) << lShift
        out[i * 4 + 1] = ((input[i * 5 + 1] << 2) | ((b4 >> 2) & 0x3)) << lShift
        out[i * 4 + 2] = ((input[i * 5 + 2] << 2) | ((b4 >> 4) & 0x3)) << lShift
        out[i * 4 + 3] = ((input[i * 5 + 3] << 2) |  (b4 >> 6)       ) << lShift

    return out

def clamp(num, v0, v1): return max(v0, min(num, v1))

print("depthai version:", dai.__version__)


# Initialize DHT11 sensor
if config["enable_dht11"]:
    dht11_sensor = adafruit_dht.DHT11(board.D23)

# Initialize RP HQ camera based on the configuration if selected
if config["camera"] == "Raspberry Pi HQ Cam":
    camera = PiCamera()
    camera.resolution = (4056, 3040)
    camera.iso = config["iso"]
    camera.shutter_speed = config["exposure_time"]
    camera.exposure_mode = 'off'
    if config["preview"]:
        camera.start_preview()


# Initialize Oak-D Lite based on the configuration if selected
if config["camera"] == "Oak-D Lite":

    streams = []
    # Enable none, one or both streams
    streams.append('isp')
    if config["raw"]:
        streams.append('raw')

    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    cam = pipeline.createColorCamera()
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)

    # Camera control input
    control = pipeline.createXLinkIn()
    control.setStreamName('control')
    control.out.link(cam.inputControl)

    if config["raw"]:
        xout_raw = pipeline.createXLinkOut()
        xout_raw.setStreamName('raw')
        cam.raw.link(xout_raw.input)
    
    device = dai.Device(pipeline)
    device.startPipeline()

    q_list = []
    for s in streams:
        q = device.getOutputQueue(name=s, maxSize=3, blocking=True)
        q_list.append(q)
        # Make window resizable, and configure initial size
        cv2.namedWindow(s, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(s, (960, 540))

    controlQueue = device.getInputQueue('control')

    # Set exposure time - 20000 microseconds by default
    if config["exposure_time"] == 0:        
        exp_time = 20000
    else:
        exp_time = config["exposure_time"]

    # If exposure time is too long, set FPS accordingly
    if config["exposure_time"]>30000:
        cam.setFps(1000000/config["exposure_time"])

    saturation = 0
    contrast = 0
    brightness = 0
    sharpness = 0
    luma_denoise = 0
    chroma_denoise = 0
    control = 'none'

    # Camera control object
    ctrl = dai.CameraControl()
    # Select autoexposure if 0 is given
    if config["exposure_time"] == 0:
        ctrl.setAutoExposureEnable()
    else:
        ctrl.setManualExposure(exp_time, sens_iso)
    ctrl.setAutoWhiteBalanceLock(awb_lock)
    ctrl.setAutoExposureLock(ae_lock)
    ctrl.setAutoExposureCompensation(ae_comp)
    ctrl.setAntiBandingMode(dai.CameraControl.AutoWhiteBalanceMode.OFF)
    ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)
    ctrl.setEffectMode(dai.CameraControl.EffectMode.OFF)
    ctrl.setBrightness(brightness)
    ctrl.setContrast(contrast)
    ctrl.setSaturation(saturation)
    ctrl.setSharpness(sharpness)
    ctrl.setLumaDenoise(luma_denoise)
    ctrl.setChromaDenoise(chroma_denoise)
    controlQueue.send(ctrl)


# Connect to ArduPilot
vehicle = connect(config["arduPilot_connection"], wait_ready=True)

# Create mission folder
mission_folder = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
os.makedirs(mission_folder, exist_ok=True)

data_list = []
collecting = False

last_time = time.time()

while True:
    # if vehicle.armed:
    if not collecting and vehicle.armed:
        collecting = True
    if collecting and (time.time()-last_time)>config["loop_frequency"]:
        # Reset timer
        last_time = time.time()
        
        # Timestamp for measurements
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        # Collect ArduPilot data
        gps = vehicle.location.global_frame
        heading = vehicle.heading
        pitch = vehicle.attitude.pitch
        roll = vehicle.attitude.roll
        altitude = vehicle.location.global_relative_frame.alt
        ground_speed = vehicle.groundspeed
        airspeed = vehicle.airspeed
        # Collect DHT11 data
        if config["enable_dht11"]:
            humidity, temperature = Adafruit_DHT.read_retry(dht11_sensor, dht11_pin)
        else:
            temperature = humidity = None

        # Image path for the next capture
        image_filename = os.path.join(mission_folder, f"{timestamp}")

        if config["camera"] == "Raspberry Pi HQ Cam":
            camera.capture(image_filename+".jpg")
        if config["camera"] == "Oak-D Lite":
            name = q.getName()
            data = q.get()
            width, height = data.getWidth(), data.getHeight()
            in_frame = color_queue.get()
            frame = in_frame.getCvFrame()
            cv2.imwrite(image_filename, frame)
            payload = data.getData()
            if name == 'raw':
                # Preallocate the output buffer
                unpacked = np.empty(payload.size * 4 // 5, dtype=np.uint16)
                # Save to capture file on bits [9:0] of the 16-bit pixels
                unpack_raw10(payload, unpacked, expand16bit=False)
                raw_filename = image_filename + '_10bit.bw'
                print("Saving to file:", raw_filename)
                unpacked.tofile(raw_filename)
                # Full range for display, use bits [15:6] of the 16-bit pixels
                unpack_raw10(payload, unpacked, expand16bit=True)
                shape = (height, width)
                bayer = unpacked.reshape(shape).astype(np.uint16)
                # See this for the ordering, at the end of page:
                # https://docs.opencv.org/4.5.1/de/d25/imgproc_color_conversions.html
                bgr = cv2.cvtColor(bayer, cv2.COLOR_BayerRG2BGR)
            else:
                png_filename = image_filename + '.png'
                print("Saving to file:", png_filename)
                bgr = np.ascontiguousarray(bgr)  # just in case
                cv2.imwrite(filename, bgr)
            if config["preview"]:    
                bgr = np.ascontiguousarray(bgr)  # just in case
                cv2.imshow(name, bgr)

        # Add data to the list
        data_list.append([timestamp, image_filename, gps.lat, gps.lon, heading, pitch, roll, altitude, ground_speed, airspeed, temperature, humidity])

    # If the vehicle disarmed after mission save the data to csv file
    if collecting and not vehicle.armed:
        # Save data to CSV file and stop collecting
        with open(os.path.join(mission_folder, "data.csv"), "w", newline="") as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(["Timestamp", "Image filename", "Latitude", "Longitude", "Heading", "Pitch", "Roll", "Altitude","Groundspeed", "Airspeed", "Temperature", "Humidity"])
            csv_writer.writerows(data_list)
        collecting = False