import numpy as np
import cv2


def region_of_interest(img, color3=(255, 255, 255), color1=255):  # ROI 셋팅
    vertices = np.array([[(0,270), (0,0 ), (480,0 ), (480,270 )]])

    mask = np.zeros_like(img)  # mask = img와 같은 크기의 빈 이미지

    if len(img.shape) > 2:
        channel_count = img.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    cv2.fillPoly(mask, vertices, ignore_mask_color)

    # 이미지와 color로 채워진 ROI를 합침
    ROI_image = cv2.bitwise_and(img, mask)
    return ROI_image


def imgae_process(img):


    cap_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #ret, thr1 = cv2.threshold(cap_gray, 200, 255, cv2.THRESH_BINARY)

    frame = cv2.medianBlur(cap_gray, 5)


    canny = cv2.Canny(frame, 50, 200)
    roi_img = region_of_interest(canny)
    cv2.imshow("canny",roi_img)
    cv2.imshow("can", frame)
    return canny

line_ = []
line__ = []

def houghP(img):
    lines = cv2.HoughLinesP(img, 0.8, np.pi / 180, 90, minLineLength=50, maxLineGap=30)
    print('t',lines)
    if lines is not None:
        global line_
        global line__
        del line_[:]
        del line__[:]


        for i in lines:
            a = abs(i[0][1] - i[0][3])
            if (a < 15):
                line_.append(i[0])

        print('line_', line_)

        for j in range(0, len(line_) - 1):
            for k in range(0, len(line_) - 1 - j):
                aaa = abs(line_[j][1] - line_[k + 1 + j][1])
                if (aaa < 30 and aaa>15):
                    if (abs(line_[j][1] - line_[k + 1 + j][1]) != 0):
                        line__.append(line_[j])
                        line__.append(line_[k + 1 + j])
        print('p', line__)
        return line__

def draw_lines(img, lines):
    for i in lines:
        cv2.line(img, (i[0], i[1]), (i[2], i[3]), (0, 0, 255), 2)
    return img



cap1 = cv2.VideoCapture(0)
cap1.set(3,480)
cap1.set(4,270)


while True:

    ret1, frame1 = cap1.read()
    #dst = cv2.resize(frame1, dsize=(960, 540), interpolation=cv2.INTER_AREA)
    stop = 1
    img_process = imgae_process(frame1)
    hough = houghP(img_process)
    if hough is not None:
        draw_lines(frame1, hough)
        a = len(hough)
        if (a>3):
            stop = 0
    else:
        stop = 1

    location = (240, 70)
    font = cv2.FONT_HERSHEY_COMPLEX  # normal size serif font
    fontScale = 1.2

    if stop == 0:

        cv2.putText(frame1, 'stop', location, font, fontScale, (255,0,0,), 2)
    else:
        cv2.putText(frame1, 'go', location, font, fontScale, (255, 0, 0,), 2)


    cv2.imshow("VideoFrame", frame1)

    if cv2.waitKey(1) > 0: break

cap1.release()

cv2.waitKey(0)
cv2.destroyAllWindows()