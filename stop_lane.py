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

# 허프라인p 를 이용해 직선의 양끝 좌표를 알아 낸후 좌표들의 차이가 일정 범위 내에 있있을 때 정지선이라 간주한다.
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

    # 검출된 라인의 길이가 3개 보다 많을 때 직선이라 검출
    # 정지선인 경우 stop =0 아닐 경우에는 stop =1 이라 준다.
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
    # stop가 0일때 정지선이라는 것을 의미한다.
    if stop == 0:

        cv2.putText(frame1, 'stop', location, font, fontScale, (255,0,0,), 2)
    else:
        cv2.putText(frame1, 'go', location, font, fontScale, (255, 0, 0,), 2)


    cv2.imshow("VideoFrame", frame1)

    if cv2.waitKey(1) > 0: break

cap1.release()

cv2.waitKey(0)
cv2.destroyAllWindows()