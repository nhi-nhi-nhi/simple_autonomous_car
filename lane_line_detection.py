import cv2
import numpy as np


def find_lane_lines(img):
    """
    Detecting road markings
    This function will take a color image, in BGR color system,
    Returns a filtered image of road markings
    """

    # Convert to gray scale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply a Gaussian filter to remove noise
    # You can experiment with other filters here.
    img_gauss = cv2.GaussianBlur(gray, (11, 11), 0)

    # Apply Canny edge detection
    thresh_low = 150
    thresh_high = 200
    img_canny = cv2.Canny(img_gauss, thresh_low, thresh_high)

    # Return image
    return img_canny


def birdview_transform(img):
    """Apply bird-view transform to the image
    """
    IMAGE_H = 480
    IMAGE_W = 640
    src = np.float32([[0, IMAGE_H], [640, IMAGE_H], [0, IMAGE_H * 0.4], [IMAGE_W, IMAGE_H * 0.4]])
    dst = np.float32([[240, IMAGE_H], [640 - 240, IMAGE_H], [-160, 0], [IMAGE_W+160, 0]])
    M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
    warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H)) # Image warping
    return warped_img


def calculate_control_signal(self, draw=None):
        """Calculate the control signal based on the detected lane lines."""
        self.left_point, self.right_point, self.have_left, self.have_right, self.len_line = calculate_left_right_point(self.img, draw=draw)

        im_center = self.img.shape[1] // 2
        if self.left_point != -1 and self.right_point != -1:
            middle_point = (self.right_point + self.left_point) // 2
            x_offset = im_center - middle_point
            self.steering_angle = -self.pid_controller.update(x_offset)

            # Adjust throttle based on steering angle
            self.throttle = self.min_throttle if abs(self.steering_angle) > 0.58 else self.default_throttle
        else:
            self.steering_angle = 0

def find_left_right_points(image, draw=None):
    """Find left and right points of lane
    """

    im_height, im_width = image.shape[:2]

    # Consider the position 70% from the top of the image
    interested_line_y = int(im_height * 0.9)
    if draw is not None:
        cv2.line(draw, (0, interested_line_y),
                 (im_width, interested_line_y), (0, 0, 255), 2)
    interested_line = image[interested_line_y, :]

    # Detect left/right points
    len_line = 0
    left_point = -1
    right_point = -1
    lane_width = 100
    center = im_width // 2
    haveLeft = 0
    haveRight = 0

    # Traverse the two sides, find the first non-zero value pixels, and
    # consider them as the position of the left and right lines
    for x in range(center, 0, -1):
        if interested_line[x] > 0:
            haveLeft = 1
            left_point = x
            break
    for x in range(center + 1, im_width):
        if interested_line[x] > 0:
            haveRight = 1
            right_point = x
            break

    if (haveLeft != 0 and haveRight !=0):
            len_line = 2
    else:
            len_line = 1

    # Predict right point when only see the left point
    if left_point != -1 and right_point == -1:
        right_point = left_point + lane_width

    # Predict left point when only see the right point
    if right_point != -1 and left_point == -1:
        left_point = right_point - lane_width


    # Draw two points on the image
    if draw is not None:
        if left_point != -1:
            draw = cv2.circle(
                draw, (left_point, interested_line_y), 7, (255, 255, 0), -1)
        if right_point != -1:
            draw = cv2.circle(
                draw, (right_point, interested_line_y), 7, (0, 255, 0), -1)

    return left_point, right_point, haveLeft, haveRight, len_line


def calculate_left_right_point(img, draw=None):
    """Calculate the left and right lane points using lane line detection."""
    img_lines = find_lane_lines(img)
    img_birdview = birdview_transform(img_lines)
    draw[:, :] = birdview_transform(draw)
    return find_left_right_points(img_birdview, draw=draw)
