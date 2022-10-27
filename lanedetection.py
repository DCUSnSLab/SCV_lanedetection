import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import time
from PyQt5.QtWidgets import  QWidget, QLabel, QApplication
from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap

class cvThread(QThread):
    signal = pyqtSignal(QImage)

    def __init__(self, qt):
        super(cvThread, self).__init__(parent=qt)
        print('init')
        rospy.init_node('gray')
        self.prevTime = 0
        self.selecting_sub_image = "compressed"  # you can choose image type "compressed", "raw"
        self.isSim = True
        self.wrapCaliDone = False

        if self.selecting_sub_image == "compressed":
            if self.isSim :
                self._sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.callback, queue_size=1)
            else:
                self._sub = rospy.Subscriber('/zed2/zed_node/left/image_rect_color/compressed', CompressedImage, self.callback, queue_size=1)

        else:
            self._sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback, queue_size=1)

        self.bridge = CvBridge()
        self.lane = LaneDet(max_counter=5)
        print('init finished')

    def doWrapCalibration(self, cv_image):
        leftupper = (0, 0)
        rightupper = (0, 0)
        leftlower = (0, 0)
        rightlower = (0, 0)
        warped_leftupper = (0, 0)
        warped_rightupper = (0, 0)
        warped_leftlower = (0, 0)
        warped_rightlower = (0, 0)


        if self.isSim:
            offset = 250
            leftupper = (285, 250)
            rightupper = (355, 250)
            leftlower = (-135, cv_image.shape[0])
            rightlower = (795, cv_image.shape[0])

            warped_leftupper = (offset, 0)
            warped_rightupper = (offset, cv_image.shape[0])
            warped_leftlower = (cv_image.shape[1] - offset, 0)
            warped_rightlower = (cv_image.shape[1] - offset, cv_image.shape[0])
        else:
            offset = 250
            leftupper = (265, 200)
            rightupper = (395, 200)
            leftlower = (-135, cv_image.shape[0])
            rightlower = (795, cv_image.shape[0])

            warped_leftupper = (offset, 0)
            warped_rightupper = (offset, cv_image.shape[0])
            warped_leftlower = (cv_image.shape[1] - offset, 0)
            warped_rightlower = (cv_image.shape[1] - offset, cv_image.shape[0])


        return (np.float32([leftupper, leftlower, rightupper, rightlower]), np.float32([warped_leftupper, warped_rightupper, warped_leftlower, warped_rightlower]))

    def callback(self, image_msg):
        # try:
        if self.selecting_sub_image == "compressed":
            # converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.COLOR_BGR2RGB)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        elif self.selecting_sub_image == "raw":
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        if self.wrapCaliDone is False:
            wrap_origin, wrap = self.doWrapCalibration(cv_image)
            self.lane.set_presp_indices(wrap_origin, wrap)
            self.wrapCaliDone = True

        # b_out = cuv_img #cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)

        out_img = self.lane.process_image(cv_image)
        h, w, ch = out_img.shape


        # size = cv_image.nbytes
        curTime = time.time()
        sec = curTime - self.prevTime
        self.prevTime = curTime
        fps = 1 / (sec)
        tstamp = float(image_msg.header.stamp.to_sec())
        #print(tstamp.to_sec(), type(tstamp))
        str = "FPS : %0.1f %0.3f" % (fps, tstamp)
        cv2.putText(out_img, str, (800, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0))

        convertToQtFormat = QImage(out_img.data, w, h, out_img.strides[0], QImage.Format_RGB888)
        p = convertToQtFormat.scaled(1280, 960, Qt.KeepAspectRatio)
        self.signal.emit(p)

        #cv2.imshow('cv_gray', cv_image), cv2.waitKey(1)
        # except Exception as e:
        #     print(e)

    def run(self):
        rospy.spin()



class LaneDet():
    def __init__(self, max_counter):
        self.current_fit_left = None
        self.best_fit_left = None
        self.history_left = [np.array([False])]
        self.current_fit_right = None
        self.best_fit_right = None
        self.history_right = [np.array([False])]
        self.counter = 0
        self.max_counter = 1
        self.src = None
        self.dst = None

    def set_presp_indices(self, src, dest):
        self.src = src
        self.dst = dest

    def reset(self):
        self.current_fit_left = None
        self.best_fit_left = None
        self.history_left = [np.array([False])]
        self.current_fit_right = None
        self.best_fit_right = None
        self.history_right = [np.array([False])]
        self.counter = 0

    def update_fit(self, left_fit, right_fit):
        if self.counter > self.max_counter:
            self.reset()
        else:
            self.current_fit_left = left_fit
            self.current_fit_right = right_fit
            self.history_left.append(left_fit)
            self.history_right.append(right_fit)
            self.history_left = self.history_left[-self.max_counter:] if len(
                self.history_left) > self.max_counter else self.history_left
            self.history_right = self.history_right[-self.max_counter:] if len(
                self.history_right) > self.max_counter else self.history_right
            self.best_fit_left = np.mean(self.history_left, axis=0)
            self.best_fit_right = np.mean(self.history_right, axis=0)

    def process_image(self, image):
        img = image#self.undistort_no_read(image, objpoints, imgpoints)

        kernel_size = 5
        mag_thresh = (30, 100)
        r_thresh = (235, 255)
        s_thresh = (165, 255)
        b_thresh = (160, 255)
        g_thresh = (210, 255)
        combined_binary, combined_binary255 = self.get_bin_img(img, kernel_size=kernel_size, sobel_thresh=mag_thresh,
                                      r_thresh=r_thresh, s_thresh=s_thresh, b_thresh=b_thresh, g_thresh=g_thresh)

        if self.src is not None or self.dst is not None:
            warped, warp_matrix, unwarp_matrix, out_img_orig, out_warped_img = self.transform_image(combined_binary,
                                                                                               src=self.src,
                                                                                               dst=self.dst)
        else:
            warped, warp_matrix, unwarp_matrix, out_img_orig, out_warped_img = self.transform_image(combined_binary)

        #get origin output
        cvwraped, cvM, cvminv, cv_out_img_orig, cv_out_warped_img = self.transform_image(image)

        xmtr_per_pixel = 3.7 / 800
        ymtr_per_pixel = 30 / 720
        cuv_img = None
        if True or self.best_fit_left is None and self.best_fit_right is None:
            left_fit, right_fit, left_fitx, right_fitx, left_lane_indices, right_lane_indices, cuv_img = self.fit_polynomial\
                (warped, nwindows=15, margin=30, show=False)
        else:
            left_fit, right_fit, left_lane_indices, right_lane_indices = self.search_around_poly(warped, self.best_fit_left,
                                                                                            self.best_fit_right,
                                                                                            xmtr_per_pixel,
                                                                                            ymtr_per_pixel)
        # To debug Find our lane pixels first
        deb_leftx, deb_lefty, deb_rightx, deb_righty, deb_left_lane_indices, deb_right_lane_indices, f_line_img \
            = self.find_lines(warped, nwindows=15, margin=30, minpix=50)

        self.counter += 1

        birdeye_debug = self.draw_birdeye_debug(warped, left_fit, right_fit, deb_leftx, deb_lefty, deb_rightx, deb_righty,
                                                deb_left_lane_indices, deb_right_lane_indices, unwarp_matrix)
        lane_img = self.draw_lines(img, left_fit, right_fit, unwarp_matrix)
        out_img = self.show_curvatures(lane_img, left_fit, right_fit, xmtr_per_pixel, ymtr_per_pixel)

        self.update_fit(left_fit, right_fit)

        cv_out_img_orig = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
        # merge to horizontal
        numpy_horizontal1 = np.concatenate((out_img, f_line_img), axis=1)
        # merge to horizontal
        numpy_horizontal2 = np.concatenate((self.draw_wrapinfo(image), birdeye_debug), axis=1)
        num_mergy = np.concatenate((numpy_horizontal1, numpy_horizontal2), axis=0)
        return num_mergy#out_img

    def draw_wrapinfo(self, img):
        out_img = np.copy(img)
        if self.src is not None and self.dst is not None:
            color_r = [0, 0, 255]
            color_g = [0, 255, 0]
            line_width = 5
            src = tuple(map(tuple, self.src))
            cv2.line(out_img, src[1], src[0], color_r, line_width)
            cv2.line(out_img, src[1], src[3], color_r, line_width * 2)
            cv2.line(out_img, src[2], src[3], color_r, line_width)
            cv2.line(out_img, src[2], src[0], color_g, line_width)

        return out_img

    def draw_birdeye_debug(self, unwarp_img, left_fit, right_fit, leftx, lefty, rightx, righty, left_lane_inds, right_lane_inds, minv):
        ploty = np.linspace(0, unwarp_img.shape[0] - 1, unwarp_img.shape[0])
        unwarp_img = np.where(unwarp_img == 1, 255, unwarp_img)
        unwarp_img = cv2.cvtColor(unwarp_img, cv2.COLOR_GRAY2BGR)

        # Colors in the left and right lane regions
        unwarp_img[lefty, leftx] = [255, 0, 0]
        unwarp_img[righty, rightx] = [0, 0, 255]

        # Find left and right points.
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

        for y, data in enumerate(left_fitx):
            left_val = unwarp_img.shape[1] - 1 if int(left_fitx[y]) >= unwarp_img.shape[1] else int(left_fitx[y])
            left_val = 0 if left_val < 0 else left_val
            right_val = unwarp_img.shape[1] - 1 if int(right_fitx[y]) >= unwarp_img.shape[1] else int(right_fitx[y])
            right_val = 0 if right_val < 0 else right_val
            # print(left_val, right_val)
            unwarp_img[y, left_val] = [255, 234, 0]
            unwarp_img[y, right_val] = [255, 234, 0]

        return unwarp_img

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
        combined_binary255 = np.zeros_like(combined)
        combined_binary[(r_binary == 1) | (combined == 1) | (s_binary == 1) | (b_binary == 1) | (g_binary == 1)] = 1
        combined_binary255[(r_binary == 1) | (combined == 1) | (s_binary == 1) | (b_binary == 1) | (g_binary == 1)] = 255

        return combined_binary, combined_binary255
        # return gray

    def transform_image(self, img, offset=250, src=None, dst=None):
        img_size = (img.shape[1], img.shape[0])

        out_img_orig = np.copy(img)

        leftupper = (265, 200)
        rightupper = (395, 200)
        leftlower = (-135, img.shape[0])
        rightlower = (795, img.shape[0])

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

    def find_lines(self, warped_img, nwindows=9, margin=80, minpix=40):

        # Take a histogram of the bottom half of the image
        histogram = np.sum(warped_img[warped_img.shape[0] // 2:, :], axis=0)

        # Create an output image to draw on and visualize the result
        out_img = np.dstack((warped_img, warped_img, warped_img)) * 255

        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0] // 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Set height of windows - based on nwindows above and image shape
        window_height = np.int(warped_img.shape[0] // nwindows)

        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = warped_img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # Current positions to be updated later for each window in nwindows
        leftx_current = leftx_base
        rightx_current = rightx_base

        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = warped_img.shape[0] - (window + 1) * window_height
            win_y_high = warped_img.shape[0] - window * window_height

            ### Find the four below boundaries of the window ###
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)

            ### Identify the nonzero pixels in x and y within the window ###
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & \
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & \
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            ### If you found > minpix pixels, recenter next window ###
            ### (`right` or `leftx_current`) on their mean position ###
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices (previously was a list of lists of pixels)
        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError:
            # Avoids an error if the above is not implemented fully
            pass

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        return leftx, lefty, rightx, righty, left_lane_inds, right_lane_inds, out_img

    def fit_polynomial(self, binary_warped, nwindows=5, margin=50, minpix=50, show=True):
        # Find our lane pixels first
        leftx, lefty, rightx, righty, left_lane_inds, right_lane_inds, out_img \
            = self.find_lines(binary_warped, nwindows=nwindows, margin=margin, minpix=minpix)

        left_fit = np.array([0, 0, 0]) if lefty.size == 0 and leftx.size == 0 else np.polyfit(lefty, leftx, 2)
        right_fit = np.array([0, 0, 0]) if rightx.size == 0 and righty.size == 0 else np.polyfit(righty, rightx, 2)


        # Generate x and y values for plotting
        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        try:
            left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
            right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
        except TypeError:
            # Avoids an error if `left` and `right_fit` are still none or incorrect
            print('The function failed to fit a line!')
            left_fitx = 1 * ploty ** 2 + 1 * ploty
            right_fitx = 1 * ploty ** 2 + 1 * ploty

        # Colors in the left and right lane regions
        out_img[lefty, leftx] = [255, 0, 0]
        out_img[righty, rightx] = [0, 0, 255]

        for y, data in enumerate(left_fitx):
            left_val = out_img.shape[1]-1 if int(left_fitx[y]) >= out_img.shape[1] else int(left_fitx[y])
            left_val = 0 if left_val < 0 else left_val
            right_val = out_img.shape[1]-1 if int(right_fitx[y]) >= out_img.shape[1] else int(right_fitx[y])
            right_val = 0 if right_val < 0 else right_val
            #print(left_val, right_val)
            out_img[y, left_val] = [255, 234, 0]
            out_img[y, right_val] = [255, 234, 0]
        # Plots the left and right polynomials on the lane lines
        # if show == True:
        #     plt.plot(left_fitx, ploty, color='yellow')
        #     plt.plot(right_fitx, ploty, color='yellow')

        return left_fit, right_fit, left_fitx, right_fitx, left_lane_inds, right_lane_inds, out_img

    def search_around_poly(self, binary_warped, left_fit, right_fit, ymtr_per_pixel, xmtr_per_pixel, margin=80):
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_lane_inds = ((nonzerox > (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy +
                                       left_fit[2] - margin)) & (nonzerox < (left_fit[0] * (nonzeroy ** 2) +
                                                                             left_fit[1] * nonzeroy + left_fit[
                                                                                 2] + margin)))
        right_lane_inds = ((nonzerox > (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy +
                                        right_fit[2] - margin)) & (nonzerox < (right_fit[0] * (nonzeroy ** 2) +
                                                                               right_fit[1] * nonzeroy + right_fit[
                                                                                   2] + margin)))

        # Again, extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # Fit a second order polynomial to each using `np.polyfit`
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        # Fit second order polynomial to for for points on real world
        left_lane_indices = np.polyfit(lefty * ymtr_per_pixel, leftx * xmtr_per_pixel, 2)
        right_lane_indices = np.polyfit(righty * ymtr_per_pixel, rightx * xmtr_per_pixel, 2)

        return left_fit, right_fit, left_lane_indices, right_lane_indices

    def radius_curvature(self, img, left_fit, right_fit, xmtr_per_pixel, ymtr_per_pixel):
        ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
        y_eval = np.max(ploty)

        left_fit_cr = np.polyfit(ploty * ymtr_per_pixel, left_fitx * xmtr_per_pixel, 2)
        right_fit_cr = np.polyfit(ploty * ymtr_per_pixel, right_fitx * xmtr_per_pixel, 2)

        # find radii of curvature
        left_rad = ((1 + (2 * left_fit_cr[0] * y_eval * ymtr_per_pixel + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * left_fit_cr[0])
        right_rad = ((1 + (2 * right_fit_cr[0] * y_eval * ymtr_per_pixel + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * right_fit_cr[0])

        return (left_rad, right_rad)

    def dist_from_center(self, img, left_fit, right_fit, xmtr_per_pixel, ymtr_per_pixel):
        ## Image mid horizontal position
        # xmax = img.shape[1]*xmtr_per_pixel
        ymax = img.shape[0] * ymtr_per_pixel

        center = img.shape[1] / 2

        lineLeft = left_fit[0] * ymax ** 2 + left_fit[1] * ymax + left_fit[2]
        lineRight = right_fit[0] * ymax ** 2 + right_fit[1] * ymax + right_fit[2]

        mid = lineLeft + (lineRight - lineLeft) / 2
        dist = (mid - center) * xmtr_per_pixel
        if dist >= 0.:
            message = 'Vehicle location: {:.2f} m right'.format(dist)
        else:
            message = 'Vehicle location: {:.2f} m left'.format(abs(dist))

        return message

    def draw_lines(self, img, left_fit, right_fit, minv):
        ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])
        color_warp = np.zeros_like(img).astype(np.uint8)

        # Find left and right points.
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

        # Warp the blank back to original image space using inverse perspective matrix
        unwarp_img = cv2.warpPerspective(color_warp, minv, (img.shape[1], img.shape[0]),
                                         flags=cv2.WARP_FILL_OUTLIERS + cv2.INTER_CUBIC)

        return cv2.addWeighted(img, 1, unwarp_img, 0.3, 0)

    def show_curvatures(self, img, leftx, rightx, xmtr_per_pixel, ymtr_per_pixel):
        (left_curvature, right_curvature) = self.radius_curvature(img, leftx, rightx, xmtr_per_pixel, ymtr_per_pixel)
        dist_txt = self.dist_from_center(img, leftx, rightx, xmtr_per_pixel, ymtr_per_pixel)

        out_img = np.copy(img)
        avg_rad = round(np.mean([left_curvature, right_curvature]), 0)
        cv2.putText(out_img, 'Average lane curvature: {:.2f} m'.format(avg_rad),
                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(out_img, dist_txt, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        return out_img

class App(QWidget):
    def __init__(self):
        super().__init__()
        self.title = 'PyQt5 Video'
        self.left = 100
        self.top = 100
        self.width = 1920
        self.height = 1080
        self.wsize = 1280
        self.hsize = 960
        self.initUI()

    @pyqtSlot(QImage)
    def setImage(self, image):
        self.label.setPixmap(QPixmap.fromImage(image))

    def initUI(self):

        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.resize(self.wsize, self.hsize)
        # create a label
        self.label = QLabel(self)
        self.label.resize(self.wsize, self.hsize)

        self.show()
        time.sleep(1)
        th = cvThread(self)
        th.signal.connect(self.setImage)
        # th.changePixmap.connect(self.setImage)
        th.start()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())