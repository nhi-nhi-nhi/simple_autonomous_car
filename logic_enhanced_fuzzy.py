import time
from lane_line_detection import *
from PID_Fuzzy import Fuzzy_PID

class CarLogic:
    def __init__(self):
        self.last_sign_time = 0
        self.last_sign_detection = ''
        self.last_sign = ''
        self.turning_time = 0
        self.min_throttle = 0.6
        self.default_throttle = 1
        self.turn_duration = 5
        self.throttle = self.default_throttle
        self.steering_angle = 0
        self.max_turn_time = 7
        self.min_turn_time = 0.1
        self.stop_time = 4
        self.see_sign_time = 0
        self.max_see_sign_time = 7
        self.stop = False
        self.straight_sign_time = 0 
        self.straight_sign_duration = 2

        # Initialize the Fuzzy PID controller
        self.pid_controller = Fuzzy_PID(Pmax=9, Pmin=0, Imax=0.8, Imin=0, Dmax=1.2, Dmin=0)
        # self.pid_controller.setKp(0.5, 0.05)
        # self.pid_controller.setKi(0, 0)
        # self.pid_controller.setKd(0, 0)
        self.pid_controller.setKp(3.5, 0)
        self.pid_controller.setKi(0.01, 0)
        self.pid_controller.setKd(0.2, 0)

        self.pid_controller.setSampleTime(0.01) 
        self.pid_controller.setSetPoint(0.0)  

    def calculate_control_signal(self, draw=None):
        """Calculate the control signal based on the detected lane lines."""
        self.left_point, self.right_point, self.have_left, self.have_right, self.len_line = calculate_left_right_point(self.img, draw=draw)

        im_center = self.img.shape[1] // 2  # Get the image center
        if self.left_point != -1 and self.right_point != -1:
            middle_point = (self.right_point + self.left_point) // 2  # Calculate middle of the lane
            x_offset = im_center - middle_point  # Calculate the offset from the lane center
            self.steering_angle = -self.pid_controller.update(x_offset)  # Use Fuzzy PID controller for steering angle

            # Adjust throttle based on the steering angle
            self.throttle = self.min_throttle if abs(self.steering_angle) > 0.58 else self.default_throttle
        else:
            self.steering_angle = 0  # No steering when no lane is detected

    def detect_sign(self, signs):
        """Detect traffic signs and update the last detected sign."""
        for sign in signs:
            class_name, _, _, _, _ = sign
            if class_name in ['left', 'right', 'no_left', 'no_right', 'straight', 'stop']:
                self.last_sign_detection = class_name
                self.see_sign_time = time.time()

    def handle_sign_detection(self):
        """Handle behavior based on the last detected sign."""
        if self.last_sign_detection == 'straight':
            self.straight_sign_time = time.time()

            if time.time() - self.straight_sign_time <= self.straight_sign_duration:

                if not self.have_left or not self.have_right:

                    self.throttle = self.default_throttle
                    self.steering_angle = 0 
                    self.set_turning_time()
                    
                # else:
                #     self.steering_angle = self.pid_controller.update(0)  
                #     self.throttle = self.default_throttle 
            else:
                self.straight_sign_time = 0
            
        elif self.steering_angle != 0 and self.last_sign_detection and not self.turning_time:
            self.throttle = self.min_throttle

        if self.last_sign_detection and not self.turning_time and (not self.have_left or not self.have_right):
            self.steering_angle = 0

        self.set_turning_time()

    def set_turning_time(self):
        """Set the turning time based on the last detected sign."""
        if not self.steering_angle and not self.turning_time:
            if self.last_sign_detection == 'right' and not self.have_right:
                self.start_turning('right')
            elif self.last_sign_detection == 'left' and not self.have_left:
                self.start_turning('left')
            elif self.last_sign_detection == 'no_right' and not self.have_left:
                self.start_turning('no_right')
            elif self.last_sign_detection == 'no_left' and not self.have_right:
                self.start_turning('no_left')
            elif self.last_sign_detection == 'straight':
                self.steering_angle = 0
                self.start_turning('straight')
            elif self.last_sign_detection == 'stop':
                print('stop')
                self.stop = True
                self.start_turning('stop', self.stop_time)

    def start_turning(self, direction, duration=None):
        """Start the turning process."""
        if duration is None: 
            self.turning_time = self.max_turn_time
        else:
            self.throttle = 0
            self.turning_time = duration
        self.last_sign_time = time.time()
        print(f"turn {direction}")

    def execute_turning(self):
        """Execute the turning process based on the sign and lane detection."""
        if self.turning_time and (0 <= (time.time() - self.last_sign_time) <= self.turning_time):
            self.perform_turn()
            if self.len_line == 2 and (time.time() - self.last_sign_time) >= self.min_turn_time:
                self.reset_turning_state()
        elif self.turning_time and (self.len_line == 2):
            self.reset_turning_state()

    def perform_turn(self):
        """Adjust throttle and steering angle during a turn."""
        if self.last_sign_detection == 'left':
            self.steering_angle = -1
        elif self.last_sign_detection == 'right':
            self.steering_angle = 1
        elif self.last_sign_detection == 'no_left':
            self.steering_angle = abs(self.steering_angle)
        elif self.last_sign_detection == 'no_right':
            self.steering_angle = -abs(self.steering_angle)
        elif self.last_sign_detection == 'straight':
            self.steering_angle = 0
        elif self.last_sign_detection == 'stop':
            self.throttle = 0
            self.steering_angle = 0

    def reset_turning_state(self):
        """Reset the state after completing a turn or stopping."""
        self.turning_time = 0
        self.last_sign = self.last_sign_detection
        self.last_sign_detection = ''
        self.see_sign_time = 0
        self.last_sign_time = 0
        print("turning state reset")

    def reset_if_no_turn(self):
        """Reset the state if no turn was made after seeing a sign."""
        if self.last_sign_detection and not self.turning_time and (time.time() - self.see_sign_time) >= self.max_see_sign_time:
            self.reset_turning_state()
            print("turning state reset")

    def decision_control(self, image, signs, draw=None):
        """Main control loop to process the image and signs."""
        self.img = image
        self.im_height, self.im_width = self.img.shape[:2]

        self.calculate_control_signal(draw=draw)
        self.detect_sign(signs)
        self.handle_sign_detection()
        self.execute_turning()
        self.reset_if_no_turn()

        if not self.have_left and not self.have_right and not self.last_sign_detection:
            self.steering_angle = 1
            self.throttle = 0.3

        if self.stop:
            self.throttle = 0
            print('car stopped')

        return self.throttle, self.steering_angle
