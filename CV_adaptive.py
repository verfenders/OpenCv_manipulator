import cv2
import numpy as np
import serial
import time
from time import sleep
import asyncio
import math

ser = serial.Serial()
ser.baudrate = 9600
ser.port = 'COM3'


async def main(data):

    ser.open()
    ser.write(bytes(data, encoding='utf8'))
    await asyncio.sleep(0.01)
    ser.close()
    print(data)

if __name__ == '__main__':
    cv2.namedWindow("result")
    cap = cv2.VideoCapture(0)
    font = cv2.FONT_HERSHEY_COMPLEX
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)   # float `width`
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    hsv_min = np.array((100, 100, 20), np.uint8)
    hsv_max = np.array((120, 255, 255), np.uint8)

    color_blue = (255, 0, 0)
    color_red = (0, 0, 128)

    while True:
        flag, img = cap.read()
        img = cv2.flip(img, 1)
        try:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            thresh = cv2.inRange(hsv, hsv_min, hsv_max)
            contours0, hierarchy = cv2.findContours(
                thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            for cnt in contours0:
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                center = (int(rect[0][0]), int(rect[0][1]))
                area = int(rect[1][0]*rect[1][1])
                x = center[0]
                y = center[1]
                x1, y1, w, h = cv2.boundingRect(cnt)

                edge1 = np.int0((box[1][0] - box[0][0], box[1][1] - box[0][1]))
                edge2 = np.int0((box[2][0] - box[1][0], box[2][1] - box[1][1]))

                usedEdge = edge1
                if cv2.norm(edge2) > cv2.norm(edge1):
                    usedEdge = edge2
                reference = (1, 0)
                angle = 180.0/math.pi * \
                    math.acos((reference[0]*usedEdge[0] + reference[1] *
                              usedEdge[1]) / (cv2.norm(reference) * cv2.norm(usedEdge)))

                if area > 3000:  # Минимальная площадь для срабатывания
                    cv2.drawContours(img, [box], 0, color_blue, 2)
                    cv2.circle(img, center, 5, color_red, 2)
                    cv2.putText(img, "%d" % int(
                        angle), (center[0]+20, center[1]-20), font, 1, color_red, 2)

                    if (x < width/2-w/5):
                        cv2.putText(img, "Сместитесь вправо! ->", (0, 40),
                                    font, 1, (0, 0, 200), 3)
                        asyncio.run(main('r'))

                    elif (x > width/2+w/5):
                        cv2.putText(img, "Сместитесь влево! <-", (0, 40),
                                    font, 1, (0, 0, 200), 3)
                        asyncio.run(main('l'))

                    elif (y < height/2-h/5):
                        cv2.putText(img, "Сместитесь вниз! \|/", (0, 40),
                                    font, 1, (0, 0, 200), 3)
                        asyncio.run(main('u'))

                    elif (y > height/2+h/5):
                        cv2.putText(img, "Сместитесь вверх! /|\ ", (0, 40),
                                    font, 1, (0, 0, 200), 3)
                        asyncio.run(main('v'))

                    else:
                        cv2.putText(img, "Объект в центре!",
                                    (0, 40), font, 1, (0, 0, 0), 3)

                        if (angle >= 75 or angle <= 15):  # Диапазон углов
                            if area < 20000 and area > 3000:  # Нижний предел площади
                                cv2.drawContours(img, [box], 0, color_blue, 2)
                                cv2.circle(img, center, 5, color_red, 2)
                                cv2.putText(img, "%d" % int(
                                    angle), (center[0]+20, center[1]-20), cv2.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
                                cv2.putText(img, "Объект слишком далеко!", (0, 140),
                                            font, 1, (0, 0, 200), 3)
                                asyncio.run(main('c'))

                            elif area > 1000000:  # Верхний предел площади
                                cv2.drawContours(img, [box], 0, color_blue, 2)
                                cv2.circle(img, center, 5, color_red, 2)
                                cv2.putText(img, "%d" % int(
                                    angle), (center[0]+20, center[1]-20), cv2.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
                                cv2.putText(img, "Объект слишком близко!", (0, 140),
                                            font, 1, (0, 0, 200), 3)
                                asyncio.run(main('f'))
                        else:
                            cv2.putText(img, "Объект под углом! ", (0, 140),
                                        font, 1, (0, 0, 200), 3)
                            asyncio.run(main('t'))
                cv2.imshow('result', img)

        except:
            cap.release()
            raise
        ch = cv2.waitKey(5)
        if ch == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
