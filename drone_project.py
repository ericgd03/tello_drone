from djitellopy import Tello
import cv2 as cv2
import mediapipe as mp

mp_holistic = mp.solutions.holistic
mp_drawing = mp.solutions.drawing_utils
mp.drawing_styles = mp.solutions.drawing_styles

coordinates = [0, 0, 0]
reference = [0.5, 0.3, 0]

roll = 0
pitch = 0 
throttle = 0
yaw = 0

kp = 120
error_x = 0
error_y = 0
# integral_error  = error_actual - error_anterior 

tello = Tello()

tello.connect() 
print("\nBattery: %s\n" % tello.get_battery()) 

tello.streamon()
frame_read = tello.get_frame_read() 

tello.takeoff()
tello.move_up(80)

holistic = mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5)

while True:

    frame = frame_read.frame
    if frame is None:
        continue

    frame = cv2.resize(frame, (360, 240))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    results = holistic.process(frame)
    mp_drawing.draw_landmarks(frame, results.face_landmarks, mp_holistic.FACEMESH_CONTOURS, mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=1, circle_radius=1))

    if results.face_landmarks:

        face_landmark = results.face_landmarks.landmark
        coordinates[0], coordinates[1], coordinates[2] = face_landmark[9].x, face_landmark[9].y, face_landmark[9].z
        cord = str(round(coordinates[0], 3)) + "," + str(round(coordinates[1], 3)) + "," + str(round(coordinates[2], 3))
        print(cord)

        error_x = coordinates[0] - reference[0]
        error_y = coordinates[1] - reference[1]

        roll = int(-kp * error_x)
        throttle = int(-kp * error_y)

        tello.send_rc_control(-roll, pitch, throttle, yaw)

        # if coordinates[0] < 0.5:
        #     roll = -15
        # else:
        #     roll = 15
        # if coordinates[1] > 0.3:
        #     throttle = -15
        # else:
        #     throttle = 15
        # if coordinates[2] > 0:
        #     pitch = 12
        # tello.send_rc_control(roll, pitch, throttle, yaw)

    tello.send_rc_control(0, 0, 0, 0)

    cv2.imshow('Tello Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

tello.send_rc_control(0, 0, 0, 0)

tello.streamoff() 
cv2.destroyAllWindows()

tello.land()