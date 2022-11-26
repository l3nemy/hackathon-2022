import cv2


def main():
    cap = cv2.VideoCapture("http://192.168.66.1:9527/videostream.cgi?loginuse=admin&loginpas=admin")
    # while (cap.isOpened()):
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    #cap.set(cv2.CAP_PROP_FRAME_COUNT, 30)
    ret, frame = cap.read()
    h, w = frame.shape[:2]

    frame = frame[10:, 130:w//2, :]
    print(f'h: {h} w: {w}')

    """
    cv2.imshow(
        'video',
        cv2.Sobel(frame, cv2.CV_8U, 0, 1, ksize=1) +
        cv2.Sobel(frame, cv2.CV_8U, 1, 0, ksize=1)
    )
    """

    cv2.imwrite("background.png", frame)


if __name__ == '__main__':
    main()
