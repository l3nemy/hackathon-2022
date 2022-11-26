import cv2


def main():
    cap = cv2.VideoCapture(0)
    while (cap.isOpened()):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FRAME_COUNT, 30)
        ret, frame = cap.read()
        h, w = frame.shape[:2]

        frame = frame[100: h-100, 100: w-100, :]
        print(f'h: {h} w: {w}')

        cv2.imshow('video', frame)
        cv2.waitKey(1)


if __name__ == '__main__':
    main()
