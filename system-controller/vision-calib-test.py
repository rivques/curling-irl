# let the user define a plane, then report the position on that plane.
# the program first prompts the user to take 3 photos. it does pose estimation on each image, then uses the points in those images to define a plane.

# then, it begins a video feed, doing constant pose estimation, projecting the points onto the plane,
# and reporting them.
import apriltag.scripts.apriltag as apriltag
import cv2
import numpy as np
detector = apriltag.Detector(searchpath=apriltag._get_dll_path())

video = cv2.VideoCapture(0)

calib_images = []
calibrated = False
planevec1 = None


# camera_params = (31.73766912e+03, 9.42209249e+02, 6.36503918e+02, 3.53464641e+02) 
camera_params = (3156.71852, 3129.52243, 359.097908, 239.736909)

while(video.isOpened()):
    success, frame = video.read()
    if not success:
        break
    
    if len(calib_images) < 3:
        cv2.imshow('Calibration Image', frame)
        key = cv2.waitKey(1)
        if key == ord('c'):
            calib_images.append(frame)
            print(f"Captured image {len(calib_images)}")
        continue

    if not calibrated:
        # fun linear algebra happens here
        uncalibrated_poses = []
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
        plane_normal = np.cross(planevec1, planevec2)
        plane_normal /= np.linalg.norm(plane_normal)
        plane_point = uncalibrated_poses[0][:3, 3]
        plane_transform = np.eye(4)
        plane_transform[:3, :3] = np.eye(3) - np.outer(plane_normal, plane_normal)
        plane_transform[:3, 3] = plane_point
        print(f"Plane normal: {plane_normal}, Plane point: {plane_point}")
        calibrated = True
    else:
        result, overlay = apriltag.detect_tags(frame,
                                               detector,
                                               camera_params=camera_params,
                                               tag_size=0.106,
                                               vizualization=3,
                                               verbose=3,
                                               annotation=True
                                              )
        if len(result) == 0:
            continue
        
        pose = result[1]
        tag_position = pose[:3, 3]
        forward_vector = pose[:3, 0]
        
        # Project the tag position onto the plane
        projected_position = plane_transform @ np.append(tag_position, 1)

        forward_vector_projected = plane_transform[:3, :3] @ forward_vector
        forward_vector_projected /= np.linalg.norm(forward_vector_projected)
        theta = np.arccos(np.clip(np.dot(forward_vector_projected, planevec1 / np.linalg.norm(planevec1)), -1.0, 1.0))

        cv2.putText(overlay, f"Projected Position: {projected_position[:3]}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(overlay, f"Angle to Plane Vec1: {np.degrees(theta):.2f} degrees", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow('Pose Estimation', overlay)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
