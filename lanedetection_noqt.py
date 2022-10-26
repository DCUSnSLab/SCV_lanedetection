import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
from sensor_msgs.msg import CompressedImage


class lanedetection_noqt():
    def __init__(self):
        print('init')
        rospy.init_node('gray')
        self.prevTime = 0
        self.selecting_sub_image = "compressed"  # you can choose image type "compressed", "raw"

        self._sub = rospy.Subscriber('/zed2/zed_node/left/image_rect_color/compressed', CompressedImage, self.callback, queue_size=1)

        self.bridge = CvBridge()
        print('init finished')
        while not rospy.is_shutdown():
            rospy.spin()

    def callback(self, image_msg):
        # try:
        if self.selecting_sub_image == "compressed":
            # converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.COLOR_BGR2RGB)
            #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        elif self.selecting_sub_image == "raw":
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")


        #cv_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        size = cv_image.nbytes
        # curTime = time.time()
        # sec = curTime - self.prevTime
        # self.prevTime = curTime
        # fps = 1 / (sec)
        # tstamp = float(image_msg.header.stamp.to_sec())
        # #print(tstamp, type(tstamp))
        # str = "FPS : %0.1f %0.3f" % (fps, tstamp)
        # cv2.putText(cv_image, str, (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))

        kernel_size = 5
        mag_thresh = (30, 100)
        r_thresh = (235, 255)
        s_thresh = (165, 255)
        b_thresh = (160, 255)
        g_thresh = (210, 255)
        combined_binary = self.get_bin_img(cv_image, kernel_size=kernel_size, sobel_thresh=mag_thresh,
                                           r_thresh=r_thresh,
                                           s_thresh=s_thresh, b_thresh=b_thresh, g_thresh=g_thresh)

        cv2.imshow("videoFrame", combined_binary)
        cv2.waitKey(3)
        #ax2.imshow(warped, cmap='gray')
        #convertToQtFormat = QImage(cv_image.data, w, h, cv_image.strides[0], QImage.Format_RGB888)
        #convertToQtFormat = QImage(combined_binary.data, w, h, combined_binary.strides[0], QImage.Format_Grayscale8)
        #p = convertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
        #self.signal.emit(convertToQtFormat)

        #cv2.imshow('cv_gray', cv_image), cv2.waitKey(1)
        # except Exception as e:
        #     print(e)

    def get_rgb_thresh_img(self, img, channel='R', thresh=(0, 255)):
        img1 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        if channel == 'R':
            bin_img = img1[:, :, 0]
        if channel == 'G':
            bin_img = img1[:, :, 1]
        if channel == 'B':
            bin_img = img1[:, :, 2]

        binary_img = np.zeros_like(bin_img).astype(np.uint8)
        binary_img[(bin_img >= thresh[0]) & (bin_img < thresh[1])] = 1

        return binary_img

    def get_lab_bthresh_img(self, img, thresh=(0, 255)):
        lab_img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        B = lab_img[:, :, 2]

        bin_op = np.zeros_like(B).astype(np.uint8)
        bin_op[(B >= thresh[0]) & (B < thresh[1])] = 1

        return bin_op

    def get_lab_athresh_img(self, img, thresh=(0, 255)):
        lab_img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        A = lab_img[:, :, 1]

        bin_op = np.zeros_like(A).astype(np.uint8)
        bin_op[(A >= thresh[0]) & (A < thresh[1])] = 1

        return bin_op

    def get_hls_sthresh_img(self, img, thresh=(0, 255)):
        hls_img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        S = hls_img[:, :, 2]

        binary_output = np.zeros_like(S).astype(np.uint8)
        binary_output[(S >= thresh[0]) & (S < thresh[1])] = 1

        return binary_output

    def get_bin_img(self, img, kernel_size=3, sobel_dirn='X', sobel_thresh=(0, 255), r_thresh=(0, 255),
                    s_thresh=(0, 255), b_thresh=(0, 255), g_thresh=(0, 255)):
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS).astype(np.float32)
        h_channel = hls[:, :, 0]
        l_channel = hls[:, :, 1]
        s_channel = hls[:, :, 2]

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if sobel_dirn == 'X':
            sobel = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=kernel_size)
        else:
            sobel = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=kernel_size)

        abs_sobel = np.absolute(sobel)
        scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel))

        sbinary = np.zeros_like(scaled_sobel)
        sbinary[(scaled_sobel >= sobel_thresh[0]) & (scaled_sobel <= sobel_thresh[1])] = 1

        combined = np.zeros_like(sbinary)
        combined[(sbinary == 1)] = 1

        # Threshold R color channel
        r_binary = self.get_rgb_thresh_img(img, thresh=r_thresh)

        # Threshhold G color channel
        g_binary = self.get_rgb_thresh_img(img, thresh=g_thresh, channel='G')

        # Threshhold B in LAB
        b_binary = self.get_lab_bthresh_img(img, thresh=b_thresh)

        # Threshold color channel
        s_binary = self.get_hls_sthresh_img(img, thresh=s_thresh)

        # If two of the three are activated, activate in the binary image
        combined_binary = np.zeros_like(combined)
        combined_binary[(r_binary == 1) | (combined == 1) | (s_binary == 1) | (b_binary == 1) | (g_binary == 1)] = 255
        #combined_binary = gray
        return combined_binary
        # return gray

    def transform_image(self, img, offset=250, src=None, dst=None):
        img_size = (img.shape[1], img.shape[0])

        out_img_orig = np.copy(img)

        leftupper = (585, 460)
        rightupper = (705, 460)
        leftlower = (210, img.shape[0])
        rightlower = (1080, img.shape[0])

        warped_leftupper = (offset, 0)
        warped_rightupper = (offset, img.shape[0])
        warped_leftlower = (img.shape[1] - offset, 0)
        warped_rightlower = (img.shape[1] - offset, img.shape[0])

        color_r = [0, 0, 255]
        color_g = [0, 255, 0]
        line_width = 5

        if src is not None:
            src = src
        else:
            src = np.float32([leftupper, leftlower, rightupper, rightlower])

        if dst is not None:
            dst = dst
        else:
            dst = np.float32([warped_leftupper, warped_rightupper, warped_leftlower, warped_rightlower])

        cv2.line(out_img_orig, leftlower, leftupper, color_r, line_width)
        cv2.line(out_img_orig, leftlower, rightlower, color_r, line_width * 2)
        cv2.line(out_img_orig, rightupper, rightlower, color_r, line_width)
        cv2.line(out_img_orig, rightupper, leftupper, color_g, line_width)

        # calculate the perspective transform matrix
        M = cv2.getPerspectiveTransform(src, dst)
        minv = cv2.getPerspectiveTransform(dst, src)

        # Warp the image
        warped = cv2.warpPerspective(img, M, img_size, flags=cv2.WARP_FILL_OUTLIERS + cv2.INTER_CUBIC)
        out_warped_img = np.copy(warped)

        cv2.line(out_warped_img, warped_rightupper, warped_leftupper, color_r, line_width)
        cv2.line(out_warped_img, warped_rightupper, warped_rightlower, color_r, line_width * 2)
        cv2.line(out_warped_img, warped_leftlower, warped_rightlower, color_r, line_width)
        cv2.line(out_warped_img, warped_leftlower, warped_leftupper, color_g, line_width)

        return warped, M, minv, out_img_orig, out_warped_img

if __name__ == '__main__':
    ld = lanedetection_noqt()
