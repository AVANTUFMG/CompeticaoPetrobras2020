#!/usr/bin/env python
# -*- coding: cp860 -*-

import sys
import rospy
import cv2
from Position_Command import Position_Info
import numpy as np
import imutils
import sensor_msgs.msg as sensor
import mrs_msgs.msg as mrs_msgs
from cv_bridge import CvBridge


class BlueFoxImage:
    def __init__(
        self,
        param={
            "low_H": 110,
            "high_H": 120,
            "low_S": 0,
            "high_S": 254,
            "low_V": 220,
            "high_V": 230,
        },
    ):
        self.DIGITS_LOOKUP = {
            (0, 0, 1, 1, 0, 1, 0): -1,
            (0, 0, 0, 1, 0, 0, 0): "-",
            (1, 1, 1, 0, 1, 1, 1): 0,
            (0, 0, 1, 0, 0, 1, 0): 1,
            (1, 0, 1, 1, 1, 0, 1): 2,
            (1, 0, 1, 1, 0, 1, 1): 3,
            (0, 1, 1, 1, 0, 1, 0): 4,
            (1, 1, 0, 1, 0, 1, 1): 5,
            (1, 1, 0, 1, 1, 1, 1): 6,
            (1, 0, 1, 0, 0, 1, 0): 7,
            (1, 1, 1, 1, 1, 1, 1): 8,
            (1, 1, 1, 1, 0, 1, 1): 9,
        }
        self.heading_data = mrs_msgs.Float64Stamped()
        self.nav = Position_Info()
        self.camera_setting = sensor.CameraInfo()
        self.camera_frame = sensor.Image()
        self.sub_Camera = rospy.Subscriber(
            "/uav1/bluefox_optflow/camera_info",
            sensor.CameraInfo,
            self.setting_callback,
        )
        self.sub = rospy.Subscriber(
            "/uav1/bluefox_optflow/image_raw", sensor.Image, self.camera_callback
        )
        self.sub_hdg = self.sub = rospy.Subscriber(
            "/uav1/control_manager/heading",
            mrs_msgs.Float64Stamped,
            self.heading_callback,
        )
        while not self.camera_frame.data:
            pass
        while self.camera_setting.K[0] <= 0:
            pass
        self.fx = self.camera_setting.K[0]
        self.fy = self.camera_setting.K[5]
        self.height = self.camera_setting.height
        self.width = self.camera_setting.width
        self.hfov = 2 * np.arctan2(self.width, 2 * self.fx)
        self.vfov = 2 * np.arctan2(self.height, 2 * self.fy)
        self.parameters = param
        self.kernel = np.ones((5, 5), np.uint8)
        self.bridge = CvBridge()
        self.img = ""

    def heading_callback(self, data):
        self.heading_data = data

    def camera_callback(self, data):
        self.camera_frame = data

    def setting_callback(self, data):
        self.camera_setting = data

    def get_frame(self):
        cv_image = ""
        while not self.camera_frame.data:
            pass
        cv_image = self.bridge.imgmsg_to_cv2(
            self.camera_frame, desired_encoding="bgr8"
        )  # precisa ser o encoding padrao de opencv BGR
        return cv_image

    def get_frame_contours(self):
        cv_image = self.get_frame()
        cv_image = cv2.GaussianBlur(cv_image, (5, 5), 0)  # filtro para ruido
        frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        frame_threshold = cv2.inRange(
            frame_HSV,
            (
                self.parameters["low_H"],
                self.parameters["low_S"],
                self.parameters["low_V"],
            ),
            (
                self.parameters["high_H"],
                self.parameters["high_S"],
                self.parameters["high_V"],
            ),
        )  # valores encontrados experimentalmente
        frame_threshold = cv2.morphologyEx(
            frame_threshold, cv2.MORPH_CLOSE, self.kernel
        )
        frame_threshold = cv2.dilate(frame_threshold, self.kernel, iterations=4)
        contours = cv2.findContours(
            frame_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[1]
        cv_image = cv2.drawContours(cv_image, contours, 0, (254, 0, 0), 3)
        self.img = cv_image
        return contours

    def get_pipe_contour(self):
        img = self.get_frame()
        kernel = np.ones((5, 5), np.uint8)
        cv_image = cv2.GaussianBlur(img, (5, 5), 0)
        img_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        img_thresh = cv2.inRange(img_HSV, (12, 0, 220), (24, 255, 255))
        img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
        img_thresh = cv2.dilate(img_thresh, kernel, iterations=2)
        # cv2.imwrite("threshold_pipe.jpg", img_thresh)
        contours = cv2.findContours(
            img_thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )[1]
        return contours, cv_image

    def get_pipe_contour_approx(self):
        cnt, cv_image = self.get_pipe_contour()
        epsilon = 0.005 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        cv_image = cv2.drawContours(cv_image, [approx], -1, (254, 0, 0), 3)
        self.img = cv_image
        # cv2.imwrite("contour_pipe.jpg", cv_image)
        return [approx]

    def get_extreme_points(self):
        cnt = self.get_pipe_contour()[0]
        epsilon = 0.1 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        extreme_points = list()
        extreme_points.append(tuple(approx[0][0]))
        extreme_points.append(tuple(approx[1][0]))
        print(extreme_points)
        return extreme_points

    def get_pipe_distances(self):
        posicoes = list()
        points = self.get_extreme_points()
        pos = self.nav.pos.position
        for c in points:
            dx = -1 * (c[0] - (self.width / 2))
            dy = -1 * (c[1] - (self.height / 2))
            beta = np.arctan2(dx * np.tan(self.hfov / 2), (self.width / 2))
            alpha = np.arctan2(dy * np.tan(self.vfov / 2), (self.height / 2))
            hdg = self.heading_data.value
            x_vec = np.cos(hdg)
            y_vec = np.sin(hdg)
            pos_x = pos.x + np.tan(alpha) * pos.z * x_vec - np.tan(beta) * pos.z * y_vec
            pos_y = pos.y + np.tan(alpha) * pos.z * y_vec + np.tan(beta) * pos.z * x_vec
            posicoes.append((pos_x, pos_y))
        return posicoes

    def get_pipe_frame_contours(self):
        img = self.get_frame()
        kernel = np.ones((5, 5), np.uint8)
        cv_image = cv2.GaussianBlur(img, (5, 5), 0)
        img_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        img_thresh = cv2.inRange(img_HSV, (12, 0, 220), (24, 255, 255))
        img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
        img_thresh = cv2.dilate(img_thresh, kernel, iterations=2)
        # cv2.imwrite("threshold_pipe.jpg", img_thresh)
        contours = cv2.findContours(
            img_thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )[1]
        # Teste Aproximacao
        cnt = contours[0]
        epsilon = 0.005 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        cv_image = cv2.drawContours(cv_image, [approx], -1, (254, 0, 0), 3)
        self.img = cv_image
        # cv2.imwrite("contour_pipe.jpg", cv_image)
        return [approx]

    def get_centroid(self):
        contours = self.get_frame_contours()
        # cv_image = self.img
        centroid = list()
        cx, cy = 0, 0
        for cnt in contours:
            # print(contours[i].shape)
            M = cv2.moments(cnt)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centroid.append((cx, cy))
        self.img = cv2.circle(self.img, (cx, cy), 7, (254, 0, 0), -1)
        return centroid

    def angle_scan(self, centroids):
        # Retorna uma flag, que eh-1 caso nao seja encontrado nenhum objeto
        # e um valor qualquer, caso contrario, que é também o numero de ojetos
        # encontrados, e uma lista com os angulos alpha e beta em relação ao
        # vetor normal à camera; beta é o angulo em x, e alpha é o angulo em y
        # de forma que x e y estão orientados com o sistema de coordenadas
        # do HECTOR quando o drone tem heading = 0
        # A lista é ordenada com o objeto mais proximo do centro da imagem
        # primeiro
        angulos = list()
        temp = list()
        if centroids == []:
            flag = -1
            return flag, angulos
        else:
            flag = len(centroids)
            for c in centroids:
                dx = -1 * (c[0] - (self.width / 2))
                dy = -1 * (c[1] - (self.height / 2))
                beta = np.arctan2(dx * np.tan(self.hfov / 2), (self.width / 2))
                alpha = np.arctan2(dy * np.tan(self.vfov / 2), (self.height / 2))
                angulos.append((beta, alpha))
            for i in range(len(angulos)):
                for j in range(len(angulos) - 1):
                    dist_pvt = np.sqrt(
                        angulos[j][0] * angulos[j][0] + angulos[j][1] * angulos[j][1]
                    )
                    dist_check = np.sqrt(
                        angulos[j + 1][0] * angulos[j + 1][0]
                        + angulos[j + 1][1] * angulos[j + 1][1]
                    )
                    if dist_pvt > dist_check:
                        temp = angulos[j + 1]
                        angulos[j + 1] = angulos[j]
                        angulos[j] = temp
            return flag, angulos

    def get_distances(self, centroids, H):
        posicoes = list()
        pos = self.nav.pos.position
        for c in centroids:
            dx = -1 * (c[0] - (self.width / 2))
            dy = -1 * (c[1] - (self.height / 2))
            beta = np.arctan2(dx * np.tan(self.hfov / 2), (self.width / 2))
            alpha = np.arctan2(dy * np.tan(self.vfov / 2), (self.height / 2))
            hdg = self.heading_data.value
            x_vec = np.cos(hdg)
            y_vec = np.sin(hdg)
            pos_x = (
                pos.x
                + np.tan(alpha) * (pos.z - H) * x_vec
                - np.tan(beta) * (pos.z - H) * y_vec
            )
            pos_y = (
                pos.y
                + np.tan(alpha) * (pos.z - H) * y_vec
                + np.tan(beta) * (pos.z - H) * x_vec
            )
            posicoes.append((pos_x, pos_y))
        return posicoes

    def get_signal_centroids(self, img_thresh, kernel, cv_image):
        img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)

        img_thresh = cv2.dilate(img_thresh, kernel, iterations=4)

        # cv2.imwrite("Output.jpg", img_thresh)

        contours = cv2.findContours(
            img_thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )[1]

        centroids_list = list()
        for c in contours:

            cv_image = cv2.drawContours(cv_image, [c], -1, (254, 0, 0), 3)

            M = cv2.moments(c)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centroids_list.append((cx, cy))
            cv_image = cv2.circle(cv_image, (cx, cy), 2, (254, 0, 0), -1)

        # cv2.imshow("Identificado", cv_image)
        # cv2.waitKey(3000)
        # cv2.destroyAllWindows()

        return centroids_list

    def get_signal_red(self):
        img = self.get_frame()
        kernel = np.ones((5, 5), np.uint8)
        cv_image = cv2.GaussianBlur(img, (5, 5), 0)
        img_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Threshold para o sinal vermelho
        img_thresh = cv2.inRange(img_HSV, (0, 0, 220), (15, 255, 255))

        centroids_list = self.get_signal_centroids(img_thresh, kernel, cv_image)
        return centroids_list

    def get_signal_green(self):
        img = self.get_frame()
        kernel = np.ones((5, 5), np.uint8)
        cv_image = cv2.GaussianBlur(img, (5, 5), 0)
        img_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Threshold para o sinal verde
        img_thresh = cv2.inRange(img_HSV, (56, 0, 220), (69, 255, 255))

        centroids_list = self.get_signal_centroids(img_thresh, kernel, cv_image)
        return centroids_list

    def plot_img(self):
        print(self.get_centroid())
        # cv2.imwrite("Centroide.jpg", self.img)

    def get_distances_platforms(self):
        # metodo para achar as posicoes das plataformas
        return self.get_distances(self.get_centroid(), 0)

    def get_distances_red_signals(self):
        return self.get_distances(self.get_signal_red(), 0.3)

    def get_distances_green_signals(self):
        return self.get_distances(self.get_signal_green(), 0.3)

    def angle_scan_red_signals(self):
        return self.get_distances(self.get_signal_red())

    def angle_scan_green_signals(self):
        return self.angle_scan(self.get_signal_green())

    # TERCEIRA FASE ---------------------------------------------------------------

    def get_panel_numbers(self):
        # retorna flag, area, e os numeros
        # nao deve ser usada
        img = self.get_frame()
        kernel = np.ones((5, 5), np.uint8)
        cv_image = cv2.GaussianBlur(img, (5, 5), 0)
        img_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        kernel = np.ones((5, 5), np.uint8)
        kernel2 = np.ones((3, 3), np.uint8)

        img_thresh = cv2.inRange(img_HSV, (0, 0, 0), (179, 255, 30))
        img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
        img_thresh = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)
        img_thresh = cv2.dilate(img_thresh, kernel, iterations=0)

        cnts = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[
            1
        ]  # mudei de 0 para 1

        if len(cnts) <= 0:
            flag = False
            final_numbers = []
            area = 0
            return flag, area, final_numbers
        flag = True

        mask = np.zeros(img_gray.shape, dtype="uint8")

        # seleciona maior contorno
        areas = [cv2.contourArea(c) for c in cnts]
        max_index = np.argmax(areas)
        cnt = cnts[max_index]

        rotrect = cv2.minAreaRect(cnt)
        ang = rotrect[len(rotrect) - 1]
        box = cv2.boxPoints(rotrect)
        box = np.int0(box)

        (x, y, w, h) = cv2.boundingRect(cnt)
        rect = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]])
        cv2.fillPoly(mask, [rect], 255)

        imageROI = img[y : y + h, x : x + w]
        maskROI = mask[y : y + h, x : x + w]

        imageROI = cv2.bitwise_and(imageROI, imageROI, mask=maskROI)
        # imageROI_BGR = imageROI
        imageROI = cv2.cvtColor(imageROI, cv2.COLOR_BGR2GRAY)

        imageROI = cv2.GaussianBlur(imageROI, (5, 5), 2)

        th3 = cv2.adaptiveThreshold(
            imageROI, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, -3
        )
        th3 = cv2.morphologyEx(th3, cv2.MORPH_OPEN, kernel2)
        th3 = imutils.rotate_bound(th3, ang)

        (h, w) = th3.shape

        newth = cv2.cvtColor(th3, cv2.COLOR_GRAY2BGR)

        newth = imutils.rotate_bound(newth, -ang)
        imageROI = imutils.rotate_bound(imageROI, -ang)
        # cv2.imwrite("imageROI.jpg", imageROI)
        for i in range(5):
            _, new_cnts, hierar = cv2.findContours(
                th3, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
            )  # acrescentei _ para o unpack

            circle_cnts = list()
            for i in range(len(new_cnts)):
                (x, y), radius = cv2.minEnclosingCircle(new_cnts[i])
                area = cv2.contourArea(new_cnts[i])
                if area > 0:
                    if (np.abs((radius * radius * np.pi) - area) / area < 0.45) and (
                        hierar[0][i][2] == -1
                    ):
                        circle_cnts.append(new_cnts[i])
                else:
                    pass
            # print(len(circle_cnts))
            avg_x = 0.0
            avg_y = 0.0
            for i in range(len(circle_cnts)):
                M = cv2.moments(circle_cnts[i])
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                avg_x += cx
                avg_y += cy
            avg_x = avg_x / 4
            avg_y = avg_y / 4
            if avg_x < (2 * w / 3):
                th3 = imutils.rotate_bound(th3, 90)
                newth = imutils.rotate_bound(newth, 90)
                imageROI = imutils.rotate_bound(imageROI, 90)
            else:
                break
        # cv2.imwrite("test.jpg", newth)
        x = int(0.08 * newth.shape[0])
        y = int(0)
        # y = int(0.08*newth.shape[1])
        h = int(newth.shape[0] - 1)
        # h = int(0.84*newth.shape[0])
        w = int(0.84 * newth.shape[1])

        cropped = imageROI[y : y + h, x : x + w]

        (h, w) = cropped.shape
        ret, cropped = cv2.threshold(
            cropped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

        # cv2.imwrite("teste.jpg", cropped)
        sqr1 = cropped[0 : np.int0(h / 2), 0 : np.int0(w / 3)]
        sqr2 = cropped[0 : np.int0(h / 2), (np.int0(w / 3) + 1) : np.int0(2 * w / 3)]
        sqr3 = cropped[0 : np.int0(h / 2), (np.int0(2 * w / 3) + 1) : np.int0(w)]
        sqr4 = cropped[np.int0(h / 2) : h, 0 : np.int0(w / 3)]
        sqr5 = cropped[np.int0(h / 2) : h, (np.int0(w / 3) + 1) : (np.int0(2 * w / 3))]
        sqr6 = cropped[np.int0(h / 2) : h, (np.int0(2 * w / 3) + 1) : np.int0(w)]

        ret3, sqr1 = cv2.threshold(sqr1, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        ret3, sqr2 = cv2.threshold(sqr2, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        ret3, sqr3 = cv2.threshold(sqr3, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        ret3, sqr4 = cv2.threshold(sqr4, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        ret3, sqr5 = cv2.threshold(sqr5, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        ret3, sqr6 = cv2.threshold(sqr6, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # cv2.imwrite("sqr1.jpg", sqr1)
        # cv2.imwrite("sqr2.jpg", sqr2)
        # cv2.imwrite("sqr3.jpg", sqr3)
        # cv2.imwrite("sqr4.jpg", sqr4)
        # cv2.imwrite("sqr5.jpg", sqr5)
        # cv2.imwrite("sqr6.jpg", sqr6)

        # ----------------------------------------------------------------------------------
        digits_thresh = []
        digits = []

        digits_thresh.append(sqr1)
        digits_thresh.append(sqr2)
        digits_thresh.append(sqr4)
        digits_thresh.append(sqr5)

        for digit in digits_thresh:

            img = digit

            cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            if len(cnts) == 0:
                digits.append("")
                continue

            areas = [cv2.contourArea(c) for c in cnts]
            max_index = np.argmax(areas)
            cnt = cnts[max_index]

            (x, y, w, h) = cv2.boundingRect(cnt)

            if (float(w) / float(h)) < 0.45:
                w = h / 2
                x = x - (3 * w) / 4
            if (float(h) / float(w)) < 0.5:
                h = w * 2
                y = y - h / 2 + 0.075 * h
                h = int(h)
                y = int(y)
            x = int(x)
            w = int(w)
            y = int(y)
            h = int(h)
            img = img[y : (y + h), x : (x + w)]
            # cv2.imwrite("teste.jpg", img)

            # img = cv2.rectangle(img, (x, y), (x+w, y+h), (125), 2)

            (h, w) = img.shape
            (dW, dH) = (int(w * 0.25), int(h * 0.15))
            dHC = int(h * 0.05)

            segments = [
                ((0, 0), (w, dH)),  # top
                ((0, 0), (dW, h // 2)),  # top-left
                ((w - dW, 0), (w, h // 2)),  # top-right
                ((0, (h // 2) - dHC), (w, (h // 2) + dHC)),  # center
                ((0, h // 2), (dW, h)),  # bottom-left
                ((w - dW, h // 2), (w, h)),  # bottom-right
                ((0, h - dH), (w, h)),  # bottom
            ]
            on = [0] * len(segments)

            for (i, ((xA, yA), (xB, yB))) in enumerate(segments):

                segROI = img[yA:yB, xA:xB]

                total = cv2.countNonZero(segROI)

                # print(total)
                area = (xB - xA) * (yB - yA)
                if float(area) == 0:
                    return True, area, []
                if total / float(area) > 0.45:
                    on[i] = 1
            # print(on)
            try:
                digit = self.DIGITS_LOOKUP[tuple(on)]
            except KeyError:
                digits.append(0)
            else:
                digits.append(digit)

        # print(digits)
        numbers = [(str(digits[0]) + str(digits[1])), (str(digits[2]) + str(digits[3]))]
        final_numbers = [int(numbers[0]), int(numbers[1])]
        return flag, area, final_numbers

    def get_panel(self):
        # retorna flag e lista, se a flag der FALSE, entao nao ha painel ali
        # se a flag for TRUE então a lista irá conter os dois numeros
        flag, area, numbers = self.get_panel_numbers()
        if flag is False:
            return flag, numbers
        else:
            if area == 0.0:
                while area == 0.0:
                    flag, area, numbers = self.get_panel_numbers()
                return flag, numbers
            return flag, numbers

    def exist_panel(self):
        return self.get_panel()[0]

    def get_numbers(self):
        return self.get_panel()[1]
