from tkinter import Text
import cv2
import numpy as np
import typing as Text


from numpy.typing import ArrayLike

""" def sobel(frame: ArrayLike) -> :
    cv3.Sobel(frame, cv2.) + cv2.Sobel(frame, ) """

""" (40, 40) 0
(110, 52) 15
(157, 153) 30
()
"""


def resize(frame: ArrayLike) -> ArrayLike:
    h, w = frame.shape[:2]
    return frame[10:, 130:w//2, :]


def my_cap_read(cap):
    _, frame = cap.read()
    return resize(frame)

def distance_from_offset(x):
    return 0.000896267*(x**2) + 0.0798457*x -4.62785

def pixel_per_30cm(x):
    return -0.000371681*(x**2) + 0.486439*x + 90.9032

def calc_speed(curr_Pos, last_pos):
    # TODO: We need to fix these fricking numbers
    curr_x = curr_Pos[4]
    prev_x = last_pos[4]
    curr_y = curr_Pos[5]
    prev_y = last_pos[5]

    # Pixels per 1cm
    pixel_per_30cm_curr_y = pixel_per_30cm(curr_y)
    pixel_per_30cm_prev_y = pixel_per_30cm(prev_y)
    # y-axis Pixel to feet
    curr_y = distance_from_offset(curr_y)
    prev_y = distance_from_offset(prev_y)

    # x-axis Pixel to feet
    curr_x = curr_x * pixel_per_30cm_curr_y
    prev_x = prev_x * pixel_per_30cm_prev_y


    distance = np.sqrt(((curr_x - prev_x)**2) +
                       ((curr_y - prev_y)**2))
    mmps = distance / (2/15) / 10

    print("Distance: ", mmps)
    curr_Pos[8] = True
    return mmps
    #(54, 9, 237), (24, 6, 114), (0, 3, 51)



def main():
    carID = 0
    cap = cv2.VideoCapture(
        "http://192.168.66.1:9527/videostream.cgi?loginuse=admin&loginpas=admin")

    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    #cap.set(cv2.CAP_PROP_FRAME_COUNT, 30)

    background = cv2.imread('background.png')

    car_tracking = np.array([[]])

    while (cap.isOpened()):
        frame1 = my_cap_read(cap)
        # h, w = frame.shape[:2]
        roi = frame1[:, :, :]
        grayscaled1 = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        grayscaled1[grayscaled1 < 0] = 0
        grayscaled1[grayscaled1 > 255] = 255

        cv2.imshow("First Frame", frame1)

        cap.grab()

        frame2 = my_cap_read(cap)
        roi = frame2[:, :, :]
        grayscaled2 = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        grayscaled2[grayscaled2 < 0] = 0
        grayscaled2[grayscaled2 > 255] = 255

        difference = np.int16(grayscaled2)-np.int16(grayscaled1)

        difference[difference < 0] = 0
        difference[difference > 255] = 255

        difference[difference < 50] = 0
        difference[difference >= 50] = 255

        difference = np.uint8(difference)


        cv2.imshow('video', frame1 - background)
        mask = np.ones((5, 5), np.uint8)

        dilate = cv2.morphologyEx(
            difference, cv2.MORPH_DILATE, mask, iterations=5)
        closed = cv2.morphologyEx(dilate, cv2.MORPH_CLOSE, mask, iterations=5)
        opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, mask, iterations=5)

        cv2.imshow("opened", opened)

        (numLabels, labels, stats, centroids) = cv2.connectedComponentsWithStats(
            opened, 4, cv2.CV_32S)

        i = 1
        while (i < numLabels):
            x = stats[i, cv2.CC_STAT_LEFT]
            y = stats[i, cv2.CC_STAT_TOP]
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            area = stats[i, cv2.CC_STAT_AREA]
            (cX, cY) = centroids[i]
            
            new_tracker = np.array([[x, y, w, h, cX, cY, carID, 0, False]])

            # if(w <= 160 and w >= 50 and h <= 160 and h >= 70):
            matchCarID = None

            if (car_tracking.size != 0):
                for car in car_tracking:
                    prev_x = car[0]
                    prev_y = car[1]
                    prev_w = car[2]
                    prev_h = car[3]
                    prev_cX = car[4]
                    prev_cY = car[5]

                    print(f'{prev_x} <= {cX} <= {prev_x+prev_w}')
                    print(f'{prev_y} <= {cY} <= {prev_y+prev_h}')

                    if ((prev_x -10 <= cX <= (prev_x+prev_w +100)) and (prev_y -100 <= cY <= (prev_y+prev_h +100)) and (x -100<= prev_cX <= (x + w +100)) and (y-100 <= prev_cY <= (y + h +100))):
                        print("Updating Trackers...")
                        matchCarID = i

                        if (car[8] != True):
                            speed = calc_speed(new_tracker[0], car)
                            car[7] = speed
                            car[8] = True
                            #x + speed* 

                        car[0] = x
                        car[1] = y
                        car[2] = w
                        car[3] = h
                        car[4] = cX
                        car[5] = cY
                    if (matchCarID == None):
                        print("Appending new Tracker... [ID: ", carID, "]")
                        car_tracking = np.vstack((car_tracking, new_tracker))
                        carID += 1

            else:
                print("Appending new Tracker...a [ID: ", carID, "]")
                car_tracking = np.array(new_tracker)
                carID += 1

            i += 1

        i = 0
        # Update new points
        if (car_tracking.size != 0):

            # Delete Unwanted Points
            while (i < car_tracking.shape[0]):
                if (car_tracking[i][5] > 300):
                    print("Deleting Tracker... [ID: ", car_tracking[i][6], "]")
                    car_tracking = np.delete(car_tracking, i, 0)
                i += 1

            # Update Points
            for car in car_tracking:
                curr_x = car[0]
                curr_y = car[1]
                curr_w = car[2]
                curr_h = car[3]
                curr_cX = car[4]
                curr_cY = car[5]
                curr_car_id = car[6]
                speed = car[7]
                print(speed)

                cv2.rectangle(frame1, (int(curr_x), int(
                    curr_y)), (int(curr_x+curr_w), int(curr_y+curr_h)), (0, 255, 0), 1)
                cv2.circle(frame1, (int(curr_cX), int(
                    curr_cY)), 3, (0, 0, 255), -1)
                cv2.putText(frame1, Text.Text(int(speed)) + "mph", (int(curr_cX),
                            int(curr_cY)), cv2.QT_FONT_NORMAL, 0.5, (0, 255, 0), 1)
                
                

        cv2.imshow("Frame", frame1)
        cv2.waitKey(1)


if __name__ == '__main__':
    main()
