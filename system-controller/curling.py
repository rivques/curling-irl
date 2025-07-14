import belay
import time
import apriltag.scripts.apriltag as apriltag
import cv2
import numpy as np
import simple_pid
import json
import os

state = "CALIBRATING" # CALIBRATING, TARGETTING, SHOOTING

controller = belay.Device('/dev/tty.usbmodem14101')
lights = belay.Device('/dev/tty.usbmodem14201')

player1_score = 0
player2_score = 0

selecting = "X"
player1_going = True
round_number = 1
max_rounds = 3

@lights.setup
def lights_setup():
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

@lights.task
def send_data(data):
    e.send(peer, data)

def set_lights(colors):
    # colors is a list of tuples (idx, (r, g, b))
    data_list = [f"{idx},{r},{g},{b}" for idx, (r, g, b) in colors]
    data_str = "|".join(data_list)
    print(f"Setting lights: {data_str}")
    send_data(data_str.encode('utf-8'))

def lights_for_coords(coords):
    # x and y are 0 to 1
    data = []
    for coord in coords:
        x, y, color = coord
        if 0 <= x <= 1 and 0 <= y <= 1:
            y_light = 70 - int(y * 70)
            x_light = 70 + int(x * 40)
            data.append((x_light, color))
            data.append((y_light, color))

    set_lights(data)

lights_setup()

controller.sync("./lib")

@controller.setup
def controller_setup():
    from machine import Pin, ADC, SPI
    import gc9a01py as gc9a01
    import NotoSansMono as font
    slider = ADC(Pin(26))
    button = Pin(0, Pin.IN, Pin.PULL_UP)
    spi = SPI(0, baudrate=6000000, sck=Pin(2), mosi=Pin(3))
    tft = gc9a01.GC9A01(
        spi,
        dc=Pin(28, Pin.OUT),
        cs=Pin(1, Pin.OUT),
        reset=Pin(29, Pin.OUT),
        rotation=90)

@controller.task
def show_scores(player1_score, player2_score, player1_going, round_number, max_rounds, state, selecting):
    tft.fill(gc9a01.color565(0, 0, 0))  # Clear the screen
    tft.write(font, f"{player1_score} - {player2_score}", 60, 20, gc9a01.color565(255, 255, 255))
    tft.write(font, f"Round: {round_number}/{max_rounds}", 15, 55, gc9a01.color565(255, 255, 255))
    tft.write(font, f"P{'1' if player1_going else '2'}'s turn", 10, 90, gc9a01.color565(255, 255, 255))
    if state == "SHOOTING":
        tft.write(font, f"P{'1' if player1_going else '2'} shooting", 10, 125, gc9a01.color565(255, 255, 255))
    else:
        tft.write(font, f"P{'2' if player1_going else '1'} pick:", 10, 125, gc9a01.color565(255, 255, 255))
        tft.write(font, f"{selecting} coord", 18, 160, gc9a01.color565(255, 255, 255))

@controller.task
def get_controller_state_raw():
    """
    Read the slider value and button state.
    
    Returns:
        tuple: (slider_value, button_pressed)
    """
    slider_value = slider.read_u16()
    button_pressed = not button.value()
    return slider_value, button_pressed

old_button_pressed = False
def get_controller_state():
    """
    Get the controller state, handling button debouncing.
    
    Returns:
        tuple: (slider_value, button_pressed)
    """
    global old_button_pressed
    slider_value, button_pressed = get_controller_state_raw()
    changed = False
    print(f"Slider value: {slider_value}, Button pressed: {button_pressed}")
    # Debounce the button press
    if button_pressed and not old_button_pressed:
        old_button_pressed = True
        changed = True
    elif not button_pressed:
        old_button_pressed = False
        changed = False
    else:
        changed = False

    return slider_value, changed

controller_setup()

detector = apriltag.Detector(searchpath=apriltag._get_dll_path())

video = cv2.VideoCapture(0)

calib_images = []
calibrated = False
planevec1 = None
uncalibrated_poses = []

ps_tag_position = np.array([0.5, 0.5, 0.0])  # Target position in the plane


# camera_params = (31.73766912e+03, 9.42209249e+02, 6.36503918e+02, 3.53464641e+02) 
camera_params = (3156.71852, 3129.52243, 359.097908, 239.736909)

ps_target_pos = np.array([0.5, 0.5, 0.0])  # Target position in the plane

start_time = time.time()


calibration_file = "/Users/rivques/Documents/pong-irl/system-controller/curling_calibration.json"

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

show_scores(0, 0, True, 1, 3, state, "X")  # Initial call to display scores

while(video.isOpened()):
    slider_value, button_pressed = get_controller_state()
    if state == "CALIBRATING":
        if calibrated:
            print("Already calibrated. Switching to TARGETTING state.")
            state = "TARGETTING"
            continue

        success, frame = video.read()
        if not success:
            break

        if not calibrated and len(calib_images) < 3:
            cv2.imshow('Calibration Image', frame)
            key = cv2.waitKey(1)
            if key == ord('c'):
                calib_images.append(frame)
                print(f"Captured image {len(calib_images)}")
            continue

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
    elif state == "TARGETTING" or state == "SHOOTING":
        if state == "TARGETTING":
            if selecting == "X":
                ps_target_pos[0] = slider_value / 65535.0
            else:
                ps_target_pos[1] = slider_value / 65535.0
            if button_pressed:
                if selecting == "X":
                    selecting = "Y"
                else:
                    selecting = "X"
                    state = "SHOOTING"
                button_pressed = False
                show_scores(player1_score, player2_score, player1_going, round_number, max_rounds, state, selecting)

        success, frame = video.read()
        if not success:
            break

        # Skip calibration since data is already loaded
        print("Using preloaded calibration data.")
        result, overlay = apriltag.detect_tags(frame,
                                            detector,
                                            camera_params=camera_params,
                                            tag_size=0.106,
                                            vizualization=3,
                                            verbose=3,
                                            annotation=True
                                            )
        if len(result) == 0:
            lights_for_coords([(ps_target_pos[0], ps_target_pos[1], (50, 0, 0))])  # Turn off lights if no tag detected
        else:
            pose = result[1]
            cs_tag_position = pose[:3, 3]
            cs_forward_vector = pose[:3, 0]
            
            # Project the tag position onto the plane
            ps_tag_position = plane_transform_inv @ np.append(cs_tag_position, 1)

            ps_forward_vector = plane_transform_inv[:3, :3] @ cs_forward_vector
            ps_forward_vector /= np.linalg.norm(ps_forward_vector)
            theta = np.atan2(ps_forward_vector[1], ps_forward_vector[0])

            ps_vec_to_target = ps_target_pos - ps_tag_position[:3]
            lights_for_coords([(np.clip(ps_tag_position[0], 0, 1), np.clip(ps_tag_position[1], 0, 1), (0, 50, 0)), (ps_target_pos[0], ps_target_pos[1], (50, 0, 0))])

        draw_line(overlay, pose, uncalibrated_poses[0][:3, 3], uncalibrated_poses[1][:3, 3], (0, 0, 255), 2)
        draw_line(overlay, pose, uncalibrated_poses[0][:3, 3], uncalibrated_poses[2][:3, 3], (0, 255, 0), 2)

        cv2.putText(overlay, f"Player 1 {player1_score} - {player2_score} Player 2", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 2)
        cv2.putText(overlay, f"Player {'1' if player1_going else '2'}'s turn", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        if state == "TARGETTING":
            cv2.putText(overlay, f"Player {'2' if player1_going else '1'}: Select {'X' if selecting == 'X' else 'Y'} coord", (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        elif state == "SHOOTING":
            cv2.putText(overlay, f"Player {'1' if player1_going else '2'}: Shooting", (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        cv2.putText(overlay, f"Round {round_number}/{max_rounds}", (10, 300), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        #cv2.putText(overlay, f"Tag Position: {ps_tag_position[:3]}", (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
        #draw_line(overlay, pose, cs_tag_position, cs_tag_position + cs_forward_vector, (255, 0, 255), 2)
        #draw_line(overlay, pose, cs_tag_position, np.array([0,0,0]), (255, 255, 0), 2)
       
        #draw_line(overlay, pose, cs_tag_position, (plane_transform @ np.append(ps_target_pos, 1))[:3], (0, 255, 255), 2)
        # draw_ps_vec(overlay, pose, plane_transform, ps_tag_position, color=(255,255,255))
        cv2.imshow('Pose Estimation', overlay)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if state == "SHOOTING":
            if button_pressed:
                state = "TARGETTING"
                # score by distance to target
                distance_to_target = np.linalg.norm(ps_tag_position[:3] - ps_target_pos)
                if player1_going:
                    player1_score += max(0, 10 - int(distance_to_target * 10))
                else:
                    player2_score += max(0, 10 - int(distance_to_target * 10))
                if not player1_going:
                    round_number += 1
                if round_number > max_rounds:
                    if player1_score > player2_score:
                        print("Player 1 wins!")
                    elif player2_score > player1_score:
                        print("Player 2 wins!")
                    else:
                        print("It's a tie!")
                    print(f"Final score: {player1_score} - {player2_score}")
                    break
                player1_going = not player1_going
                show_scores(player1_score, player2_score, player1_going, round_number, max_rounds, state, selecting)
                continue
