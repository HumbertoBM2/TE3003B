import cv2
import numpy as np
import os

cv2.ocl.setUseOpenCL(False)

CALIB_VIDEO  = "calibracion.mp4"

CHECKERBOARD   = (7, 5)   
SQUARE_SIZE    = 0.03     

CALIB_INTERVAL = 15       
CALIB_MAX      = 40

SCENE_INTERVAL = 10       
SCENE_MAX      = 30       
SCALE          = 0.4    


def extract_frames(path, interval, max_frames):
    cap = cv2.VideoCapture(path)
    frames, i, saved = [], 0, 0
    while cap.isOpened() and saved < max_frames:
        ret, frame = cap.read()
        if not ret:
            break
        if i % interval == 0:
            frames.append(frame)
            saved += 1
        i += 1
    cap.release()
    print(f"[Stage 1] {path}: {saved} frames")
    return frames


def calibrate(frames):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

    obj_pts, img_pts, img_shape = [], [], None
    os.makedirs("debug_calib", exist_ok=True)

    flags_list = [
        None,
        cv2.CALIB_CB_ADAPTIVE_THRESH,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FILTER_QUADS,
    ]

    for frame in frames:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = False, None
        for flags in flags_list:
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, flags)
            if ret:
                found = True
                break
        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_pts.append(objp)
            img_pts.append(corners)
            img_shape = gray.shape[::-1]
            dbg = frame.copy()
            cv2.drawChessboardCorners(dbg, CHECKERBOARD, corners, True)
            cv2.imwrite(f"debug_calib/{len(obj_pts):03d}.jpg", dbg)

    if len(obj_pts) < 5:
        raise RuntimeError(f"Solo {len(obj_pts)} frames con tablero detectado. "
                           "Verifica CHECKERBOARD o la iluminación del video.")

    ret, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, img_shape, None, None)
    print(f"[Stage 2] Calibración OK — {len(obj_pts)} frames, RMS={ret:.3f} px")
    return K, dist


def undistort(frames, K, dist):
    h, w = frames[0].shape[:2]
    K_new, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=0)
    x, y, rw, rh = roi
    result = []
    for f in frames:
        u = cv2.undistort(f, K, dist, None, K_new)
        result.append(u[y:y+rh, x:x+rw])
    print(f"[Stage 3] Undistort OK — {len(result)} frames")
    return result


if __name__ == "__main__":
    calib_frames = extract_frames(CALIB_VIDEO, CALIB_INTERVAL, CALIB_MAX)
    K, dist = calibrate(calib_frames)

    scene_frames = extract_frames(SCENE_VIDEO, SCENE_INTERVAL, SCENE_MAX)
    scene_ud = undistort(scene_frames, K, dist)
