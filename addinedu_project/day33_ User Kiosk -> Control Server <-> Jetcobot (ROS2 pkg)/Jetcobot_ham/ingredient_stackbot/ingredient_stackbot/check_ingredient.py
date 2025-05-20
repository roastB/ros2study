import cv2
import numpy as np
import os
import time
import datetime
import pickle
import ingredient_stackbot.recipe as rc

def check_ingredients_order(ingredients):

    try:
        with open('/home/jetcobot/ham_ws/src/ingredient_stackbot/resource/camera_calibration.pkl', 'rb') as f:
            calibration_data = pickle.load(f)
        print("Calibration data loaded successfully")
    except FileNotFoundError:
        print("Error: Camera calibration file not found")
        return
    except Exception as e:
        print(f"Error loading calibration data: {e}")
        return

    camera_matrix = calibration_data['camera_matrix']
    dist_coeffs = calibration_data['dist_coeffs']

    print(camera_matrix)
    print(dist_coeffs)
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    marker_size = 30
    half_size = marker_size / 2
    marker_3d_edges = np.array([
        [-half_size, -half_size, 0],
        [-half_size, half_size, 0],
        [half_size, half_size, 0],
        [half_size, -half_size, 0]
    ], dtype='float32').reshape((4, 1, 3))

    blue_BGR = (255, 0, 0)
    
    cap = cv2.VideoCapture('/dev/video0')

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    time.sleep(2)
    
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        cap.release()
        return

    frame_undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
    corners, ids, rejected = detector.detectMarkers(frame_undistorted)

    plate_position = rc.get_plate_position("접시")[0][0]
    topbread_position = rc.get_bread_position("위 빵")[0][0]
    bottombread_position = rc.get_bread_position("아래 빵")[0][0]    
    meat_position = rc.get_meat_position(ingredients[0])[0][0]
    sauce_position = rc.get_sauce_position(ingredients[1])[0][0]
    vegetable_position = rc.get_vegetable_position(ingredients[2])[0][0]
    cheese_position = rc.get_cheese_position(ingredients[3])[0][0]
    target_ids = [plate_position, topbread_position, bottombread_position, meat_position, sauce_position, vegetable_position, cheese_position]
    print(target_ids)
    
    if ids is not None:
        detected_ids = [int(x[0]) for x in ids]
        print("Detected marker ids:", detected_ids)

        if any(item in detected_ids for item in target_ids):
            # empty_ingredients = [item for item in detected_ids if item in target_ids]

            print("필요한 재료가 없습니다!")
            cap.release()
            return empty_ingredients
        else:
            print("재료 없음!")
            cap.release()
            return []
    else:
        print("재료 모두 있음! (마커인식 안됨)")
        cap.release()
        return []