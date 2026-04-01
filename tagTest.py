import cv2
import apriltag
import numpy as np
import math
import time

# --- CONFIGURATION ---
TAG_FAMILY = "tag16h5"
TAG_SIZE   = 0.07874    # 3.1 inches in meters

# --- CAMERA INTRINSICS (rough estimates) ---
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
fx = CAMERA_WIDTH
fy = CAMERA_WIDTH
cx_cam = CAMERA_WIDTH  / 2.0
cy_cam = CAMERA_HEIGHT / 2.0

CAMERA_MATRIX = np.array([[fx,  0, cx_cam],
                           [ 0, fy, cy_cam],
                           [ 0,  0,  1   ]], dtype=np.float64)
DIST_COEFFS = np.zeros((4, 1))

HALF = TAG_SIZE / 2.0
TAG_POINTS_3D = np.array([
    [-HALF,  HALF, 0],
    [ HALF,  HALF, 0],
    [ HALF, -HALF, 0],
    [-HALF, -HALF, 0],
], dtype=np.float64)

# --- DETECTOR ---
try:
    detector = apriltag.apriltag(TAG_FAMILY)
except Exception as e:
    print(f"Failed to initialize detector: {e}")
    exit()

def draw_axis(frame, rvec, tvec, length=0.05):
    axis_pts = np.float32([
        [0,      0,      0     ],
        [length, 0,      0     ],
        [0,      length, 0     ],
        [0,      0,     -length],
    ])
    img_pts, _ = cv2.projectPoints(axis_pts, rvec, tvec, CAMERA_MATRIX, DIST_COEFFS)
    img_pts = img_pts.reshape(-1, 2).astype(int)
    origin = tuple(img_pts[0])
    cv2.arrowedLine(frame, origin, tuple(img_pts[1]), (0,   0, 255), 2, tipLength=0.3)
    cv2.arrowedLine(frame, origin, tuple(img_pts[2]), (0, 255,   0), 2, tipLength=0.3)
    cv2.arrowedLine(frame, origin, tuple(img_pts[3]), (255,  0,   0), 2, tipLength=0.3)

# --- CAMERA ---
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cap.isOpened():
    cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

print(f"Detector started for {TAG_FAMILY}. Press 'q' to quit.")

last_time = time.time()

while True:
    success, frame = cap.read()
    if not success:
        break

    # FPS
    now = time.time()
    fps = 1.0 / max(now - last_time, 1e-9)
    last_time = now

    # Detect
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray)

    # Static HUD
    h, w = frame.shape[:2]
    cv2.line(frame, (w//2 - 15, h//2), (w//2 + 15, h//2), (255, 255, 255), 1)
    cv2.line(frame, (w//2, h//2 - 15), (w//2, h//2 + 15), (255, 255, 255), 1)
    cv2.putText(frame, f"FPS: {int(fps)}",         (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    cv2.putText(frame, f"Tags: {len(detections)}", (20, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    for det in detections:
        try:
            corners_2d  = np.array(det['lb-rb-rt-lt'], dtype=np.float64)
            center      = det['center']
            tag_id      = det['id']
            corners_int = corners_2d.astype(np.int32)

            cv2.polylines(frame, [corners_int.reshape(-1, 1, 2)], True, (0, 255, 0), 3)
            cv2.circle(frame, tuple(corners_int[0]), 8, (255, 100, 0), -1)

            cx_tag = int(center[0])
            cy_tag = int(center[1])
            cv2.circle(frame, (cx_tag, cy_tag), 6, (0, 0, 255), -1)

            ok, rvec, tvec = cv2.solvePnP(
                TAG_POINTS_3D, corners_2d,
                CAMERA_MATRIX, DIST_COEFFS,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if ok:
                draw_axis(frame, rvec, tvec, length=TAG_SIZE * 0.6)

                tx, ty, tz = tvec.flatten()
                dist_in = tz * 39.3701

                rot_mat, _ = cv2.Rodrigues(rvec)
                sy = math.sqrt(rot_mat[0,0]**2 + rot_mat[1,0]**2)
                if sy > 1e-6:
                    roll  = math.degrees(math.atan2( rot_mat[2,1], rot_mat[2,2]))
                    pitch = math.degrees(math.atan2(-rot_mat[2,0], sy))
                    yaw   = math.degrees(math.atan2( rot_mat[1,0], rot_mat[0,0]))
                else:
                    roll  = math.degrees(math.atan2(-rot_mat[1,2], rot_mat[1,1]))
                    pitch = math.degrees(math.atan2(-rot_mat[2,0], sy))
                    yaw   = 0.0

                lines = [
                    f"ID: {tag_id}",
                    f"Dist: {dist_in:.1f}in ({tz*100:.1f}cm)",
                    f"X: {tx*39.3701:+.2f}in  Y: {ty*39.3701:+.2f}in",
                    f"R:{roll:+.1f}  P:{pitch:+.1f}  Y:{yaw:+.1f}",
                ]
                for i, line in enumerate(lines):
                    cv2.putText(frame, line,
                                (cx_tag - 60, cy_tag - 75 + i * 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        except Exception as e:
            print(f"Error processing detection: {e}")
            print(f"  Detection: {det}")

    cv2.imshow("AprilTag Pose Estimation", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()