import belay
import time
import apriltag.scripts.apriltag as apriltag
import cv2
import numpy as np
import simple_pid
import json
import os

device = belay.Device('/dev/tty.usbmodem14101')

@device.setup
def setup():
    import network
    import espnow

    sta = network.WLAN(network.WLAN.IF_STA)  # Or network.WLAN.IF_AP
    sta.active(True)
    sta.disconnect()

    print(f"Active, this board's MAC: {sta.config('mac')}") # b"H'\xe2F\xce\xb6"

    e = espnow.ESPNow()
    e.active(True)
    peer = b"H'\xe2F\xce\x9c"   # MAC address of peer's wifi interface
    e.add_peer(peer)      # Must add_peer() before send()

@device.task
def connect_to_ball():
    pass
    # e.send(peer, "Starting...")
    # for i in range(10):
    #     e.send(peer, str(i)*20, True)
    # e.send(peer, b'end')

@device.task
def send_data(data):
    e.send(peer, data)

def set_motor_speeds(left: float, right: float):
    """
    Send motor speeds to the device.
    
    Args:
        left (float): Speed for the left motor (-1.0 to 1.0).
        right (float): Speed for the right motor (-1.0 to 1.0).
    """
    data = f"{right:.2f},{left:.2f}".encode('utf-8')
    try:
        send_data(data)
    except Exception as e:
        print(f"Error sending data:")
        print(e)
    print(f"Sent speeds: {data.decode('utf-8')}")

setup()
time.sleep(0.5)
connect_to_ball()

detector = apriltag.Detector(searchpath=apriltag._get_dll_path())

video = cv2.VideoCapture(0)

calib_images = []
calibrated = False
planevec1 = None
uncalibrated_poses = []


# camera_params = (31.73766912e+03, 9.42209249e+02, 6.36503918e+02, 3.53464641e+02) 
camera_params = (3156.71852, 3129.52243, 359.097908, 239.736909)

ps_target_pos = np.array([0.5, 0.5, 0.0])  # Target position in the plane

start_time = time.time()
def target_pos():
    # a circle parameterized by time
    t = time.time() - start_time
    radius = 0.5
    center = np.array([0.5, 0.5, 0.0])
    return center + radius * np.array([np.cos(t/2), np.sin(t/2), 0.0])

turning_pid = simple_pid.PID(-0.75, -0.3, 0, setpoint=0)
turning_pid.output_limits = (-1, 1)

moving_pid = simple_pid.PID(-2, 0, 0, setpoint=0.0)
moving_pid.output_limits = (-1, 1)


loop_start_time = 0
read_time = 0
calibration_check_time = 0
detection_time = 0
motor_control_time = 0

calibration_file = "/Users/rivques/Documents/pong-irl/system-controller/calibration_data.json"

def save_calibration_data(plane_transform, plane_transform_inv, uncalibrated_poses):
    """
    Save calibration data to a file.
    """
    data = {
        "plane_transform": plane_transform.tolist(),
        "plane_transform_inv": plane_transform_inv.tolist(),
        "uncalibrated_poses": [pose.tolist() for pose in uncalibrated_poses]
    }
    with open(calibration_file, "w") as f:
        json.dump(data, f)
    print("Calibration data saved.")

def load_calibration_data():
    """
    Load calibration data from a file.
    """
    with open(calibration_file, "r") as f:
        data = json.load(f)
    plane_transform = np.array(data["plane_transform"])
    plane_transform_inv = np.array(data["plane_transform_inv"])
    uncalibrated_poses = [np.array(pose) for pose in data["uncalibrated_poses"]]
    print("Calibration data loaded.")
    return plane_transform, plane_transform_inv, uncalibrated_poses

def cam_space_to_screen_space(points, pose):
    fx, fy, cx, cy = camera_params

    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    #rvec, _ = cv2.Rodrigues(pose[:3,:3])
    rvec = np.zeros((3,1), dtype=np.float32)
    tvec = np.zeros((3,1), dtype=np.float32) # pose[:3, 3]

    dcoeffs = np.zeros(5)

    ipoints, _ = cv2.projectPoints(points, rvec, tvec, K, dcoeffs)

    ipoints = np.round(ipoints).astype(int)

    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]
    return ipoints

def draw_line(overlay, pose, start, end, color=(0, 255, 0), thickness=2):
    start, end = cam_space_to_screen_space(np.array([start, end]), pose)
    cv2.line(overlay, start, end, color, thickness, 16)

def draw_ps_vec(overlay, pose, plane_transform, vec, origin=np.array([0, 0, 0]), color=(255, 0, 0), thickness=2):
    if np.ndim(vec) == 1 and vec.shape[0] == 3:
        vec = np.append(vec, 1)
    if np.ndim(origin) == 1 and origin.shape[0] == 3:
        origin = np.append(origin, 1)
    start = plane_transform @ origin
    end = plane_transform @ vec
    draw_line(overlay, pose, start[:3], end[:3], color, thickness)

if os.path.exists(calibration_file):
    try:
        plane_transform, plane_transform_inv, uncalibrated_poses = load_calibration_data()
        calibrated = True
    except Exception as e:
        print(f"Failed to load calibration data: {e}")
        calibrated = False
else:
    calibrated = False

try:
    while(video.isOpened()):
        loop_start_time = time.time()  # Start timing the loop

        success, frame = video.read()
        if not success:
            break
        read_time = time.time() - loop_start_time  # Time spent reading the frame

        if not calibrated and len(calib_images) < 3:
            cv2.imshow('Calibration Image', frame)
            key = cv2.waitKey(1)
            if key == ord('c'):
                calib_images.append(frame)
                print(f"Captured image {len(calib_images)}")
            continue
        calibration_check_time = time.time() - loop_start_time - read_time  # Time spent checking calibration

        if not calibrated:
            # fun linear algebra happens here
            for i, img in enumerate(calib_images):
                result, overlay = apriltag.detect_tags(img,
                                                    detector,
                                                    camera_params=camera_params,
                                                    tag_size=0.106,
                                                    vizualization=3,
                                                    verbose=3,
                                                    annotation=True
                                                    )
                uncalibrated_poses.append(result[1])
            
            # uncalibrated_poses is now a list of 3 affine matrices. 
            planevec1 = uncalibrated_poses[1][:3, 3] - uncalibrated_poses[0][:3, 3]
            planevec2 = uncalibrated_poses[2][:3, 3] - uncalibrated_poses[0][:3, 3]
            print("Poses:")
            print(uncalibrated_poses)
            plane_normal = np.cross(planevec1, planevec2)
            plane_normal /= np.linalg.norm(plane_normal)
            plane_point = uncalibrated_poses[0][:3, 3]
            plane_transform = np.eye(4)
            # plane_transform[:3, :3] = np.eye(3) - np.outer(plane_normal, plane_normal)
            plane_transform[:3, 0] = planevec1
            plane_transform[:3, 1] = planevec2
            plane_transform[:3, 2] = plane_normal # should this be (0,0,0)?
            plane_transform[:3, 3] = plane_point
            plane_transform_inv = np.linalg.inv(plane_transform)
            print(f"Plane normal: {plane_normal}, Plane point: {plane_point}")
            print(f"Plane transform: {plane_transform}")
            print(f"Plane transform inverse: {plane_transform_inv}")
            print(f"Plane transform condition number: {np.linalg.cond(plane_transform)}")
            print(f"plane_transform.plane_transform_inv: {(plane_transform @ plane_transform_inv)}")
            save_calibration_data(plane_transform, plane_transform_inv, uncalibrated_poses)
            calibrated = True
        else:
            # Skip calibration since data is already loaded
            print("Using preloaded calibration data.")
            process_start_time = time.time()
            result, overlay = apriltag.detect_tags(frame,
                                                detector,
                                                camera_params=camera_params,
                                                tag_size=0.106,
                                                vizualization=3,
                                                verbose=3,
                                                annotation=True
                                                )
            detection_time = time.time() - process_start_time  # Time spent detecting tags

            if len(result) == 0:
                set_motor_speeds(0.0, 0.0)  # Stop motors if no tags detected
                continue
            
            pose = result[1]
            cs_tag_position = pose[:3, 3]
            cs_forward_vector = pose[:3, 0]
            
            # Project the tag position onto the plane
            ps_tag_position = plane_transform_inv @ np.append(cs_tag_position, 1)

            ps_forward_vector = plane_transform_inv[:3, :3] @ cs_forward_vector
            ps_forward_vector /= np.linalg.norm(ps_forward_vector)
            theta = np.atan2(ps_forward_vector[1], ps_forward_vector[0])

            ps_target_pos = target_pos()

            ps_vec_to_target = ps_target_pos - ps_tag_position[:3]
            angle_to_target_uncorrected = np.arctan2(ps_vec_to_target[1], ps_vec_to_target[0])
            angle_to_target = angle_to_target_uncorrected - theta
            angle_to_target = (angle_to_target + np.pi) % (2 * np.pi) - np.pi # Normalize to [-pi, pi]
            
            turning_speed = turning_pid(angle_to_target)
            moving_speed = moving_pid(np.linalg.norm(ps_vec_to_target)) * np.cos(angle_to_target)

            left_speed = np.clip(-turning_speed + moving_speed, -1, 1)
            right_speed = np.clip(turning_speed + moving_speed, -1, 1)
            set_motor_speeds(left_speed, right_speed)

            motor_control_time = time.time() - process_start_time - detection_time  # Time spent on motor control

            cv2.putText(overlay, f"CS Position: {cs_tag_position[:3]}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
            cv2.putText(overlay, f"Angle to Plane Vec1: {np.degrees(theta):.2f} degrees", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
            cv2.putText(overlay, f"Turning speed: {turning_speed:.2f}, Moving speed: {moving_speed:.2f}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
            cv2.putText(overlay, f"Left speed: {left_speed:.2f}, Right speed: {right_speed:.2f}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
            cv2.putText(overlay, f"Angle to Target: {np.degrees(angle_to_target):.2f} degrees", (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
            cv2.putText(overlay, f"CS position reprojected: {((plane_transform @ plane_transform_inv) @ np.append(cs_tag_position, 1))[:3]}", (10, 230), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
            draw_line(overlay, pose, cs_tag_position, cs_tag_position + cs_forward_vector, (255, 0, 255), 2)
            draw_line(overlay, pose, cs_tag_position, np.array([0,0,0]), (255, 255, 0), 2)
            draw_line(overlay, pose, uncalibrated_poses[0][:3, 3], uncalibrated_poses[1][:3, 3], (0, 0, 255), 2)
            draw_line(overlay, pose, uncalibrated_poses[0][:3, 3], uncalibrated_poses[2][:3, 3], (0, 255, 0), 2)
            draw_line(overlay, pose, cs_tag_position, (plane_transform @ np.append(ps_target_pos, 1))[:3], (0, 255, 255), 2)
            draw_ps_vec(overlay, pose, plane_transform, ps_tag_position, color=(255,255,255))
            cv2.imshow('Pose Estimation', overlay)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        loop_end_time = time.time() - loop_start_time  # Total time for the loop iteration
        if calibrated:
            print(f"Timing: Read={read_time:.4f}s, Calibration Check={calibration_check_time:.4f}s, "
                f"Calibration={calibration_check_time if not calibrated else 0:.4f}s, "
                f"Detection={detection_time if calibrated else 0:.4f}s, "
                f"Motor Control={motor_control_time if calibrated else 0:.4f}s, "
                f"Total Loop={loop_end_time:.4f}s")
except:
    set_motor_speeds(0.0, 0.0)  # Stop motors on exit
    raise