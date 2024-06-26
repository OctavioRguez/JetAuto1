#!/usr/bin/python
import os
import socket
import time
import numpy as np
import copy
import argparse
import cv2 as cv
from pupil_apriltags import Detector
import pygame

# Window Dimensions
WINDOW_WIDTH = 960
WINDOW_HEIGHT = 540

# Colors
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

class control:
    def __init__(self) -> None:
        # Declare socket for TCP client connection
        self.s = socket.socket()
        # self.s.connect(('192.168.149.77', 80))

        # Variables for camera position feedback
        self.start = False
        self.camera_parameters()
        self.centers = {"1":[0.0, 0.0], "2":[0.0, 0.0], "3":[0.0, 0.0]} # Tags (x, y) position
        self.angles = {"1":0.0, "2":0.0, "3":0.0}

        # Robot 1, Robot 2 and Virtual leader states
        self.x1, self.y1, self.theta1 = 3, 0, 0
        self.x2, self.y2, self.theta2 = 1, 0, 0
        self.xv, self.yv, self.thetav = 2, 0, 0

        # Control constants
        self.kp = {"kx":0.5, "ky":0.5, "kr":0.5}
        self.k = 1
        self.vmax, self.wmax = 0.3, 0.3

        # Desired states (references)
        self.d_d = 0.5
        self.xd, self.yd, self.thetad = 4, 4, -np.pi/2

        # Flag for start the simulation
        self.simulate = False

    # Function for start camera with desired parameters
    def camera_parameters(self) -> None:
        # Camera and detection parameters
        parser = argparse.ArgumentParser()
        parser.add_argument("--device", type=int, default=0)
        parser.add_argument("--width", help='cap width', type=int, default=WINDOW_WIDTH)
        parser.add_argument("--height", help='cap height', type=int, default=WINDOW_HEIGHT)
        parser.add_argument("--families", type=str, default='tag36h11')
        parser.add_argument("--nthreads", type=int, default=1)
        parser.add_argument("--quad_decimate", type=float, default=2.0)
        parser.add_argument("--quad_sigma", type=float, default=0.0)
        parser.add_argument("--refine_edges", type=int, default=1)
        parser.add_argument("--decode_sharpening", type=float, default=0.25)
        parser.add_argument("--debug", type=int, default=0)
        args = parser.parse_args()

        cap_device = args.device
        cap_width = args.width
        cap_height = args.height

        # Send parameters into openCV camera object
        self.cap = cv.VideoCapture(cap_device)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, cap_width)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, cap_height)

        # Initialize detector object
        self.at_detector = Detector(
            families=args.families,
            nthreads=args.nthreads,
            quad_decimate=args.quad_decimate,
            quad_sigma=args.quad_sigma,
            refine_edges=args.refine_edges,
            decode_sharpening=args.decode_sharpening,
            debug=args.debug
        )

    # Function for detecting tags, getting (x, y) coordinates and angles
    def camera(self) -> None:
        # Read current frame and deep copy the original frame
        image = self.cap.read()[1]
        height, width = image.shape[:2]
        debug_image = copy.deepcopy(image)

        # Apply gray filter and detect tags
        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(image)

        # Get angle and position for each tag
        for tag in tags:
            center, id, corners = tag.center, tag.tag_id, tag.corners

            # X and y of the center
            center = (int(center[0]), int(center[1]))
            # Get angle for current tag
            corner1, corner2 = (int(corners[0][0]), int(corners[0][1])), (int(corners[1][0]), int(corners[1][1]))
            CA, CO = corner2[0]-corner1[0], corner1[1]-corner2[1] # Catetos
            angulo = np.arctan2(CO, CA)

            # Save states feedback from current tag
            pixelsToMeters = 5.5 / (100*np.sqrt(CA**2 + CO**2))
            self.centers[str(id)] = [pixelsToMeters*(center[0] - width//2), -pixelsToMeters*(center[1] - height//2)]
            self.angles[str(id)] = angulo
            cv.circle(debug_image, (center[0], center[1]), 3, RED, 2) # Center
            cv.putText(debug_image, f"X = {self.centers[str(id)][0]:.3f}", (center[0]+10, center[1]+15), 
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, BLUE, 2, cv.LINE_AA) # X
            cv.putText(debug_image, f"Y = {self.centers[str(id)][1]:.3f}", (center[0]+10, center[1]), 
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, BLUE, 2, cv.LINE_AA) # Y
            cv.putText(debug_image, f"Theta = {angulo:.3f}", (center[0]+10, center[1]-15), 
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, BLUE, 2, cv.LINE_AA) # Angle
        cv.imshow("Camera", debug_image) # Display frame

        # Control states feedback 
        # self.x1, self.y1, self.theta1 = self.centers["1"][0], self.centers["1"][1], self.angles["1"]
        # self.x2, self.y2, self.theta2 = self.centers["2"][0], self.centers["2"][1], self.angles["2"]
        # self.xd, self.yd, self.thetad = self.centers["3"][0], self.centers["3"][1], self.angles["3"]
        # if not self.start:
        #     self.xv, self.yv = (self.x1 + self.x2)/2, (self.y1 + self.y2)/2

        # Print control states
        # print(f"X1 = {self.x1:.3f},  Y1 = {self.y1:.3f},  Theta1 = {self.theta1:.3f}\n")
        # print(f"X2 = {self.x2:.3f},  Y2 = {self.y2:.3f},  Theta2 = {self.theta2:.3f}\n")
        # print(f"Xv = {self.xv:.3f},  Yv = {self.yv:.3f},  Thetav = {self.thetav:.3f}\n")
        # print(f"Xd = {self.xd:.3f},  Yd = {self.yd:.3f},  Thetad = {self.thetad:.3f}\n")

    # Function for sending data to ESP32 (by TCP)
    def send_to_esp32(self, message : str) -> None:
        data = bytes(message, 'utf-8') # Data to send
        self.s.sendall(data)

    # Function for controlling the current states according to desired states
    def control(self, x : float, y : float, theta : float, xd : float, yd : float, thetad : float, dt : float, virtual : bool, addVel) -> list:
        # Increase lineal velocity for followers (for being able to keep up)
        if (not virtual and (abs(x - xd) > 0.2 or abs(y - yd) > 0.2)):
            vmax = self.vmax * 3
            kx = self.kp["kx"] * 3
            ky = self.kp["ky"] * 3
        else:
            vmax = self.vmax
            kx = self.kp["kx"]
            ky = self.kp["ky"]
    
        # Matrix calculations for lineal control
        A = -np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
        B, C = np.array([[kx, 0], [0, ky]]), np.array([x - xd, y - yd])
        Vec = np.dot(np.dot(A, B), C) # Vector velocities (vx, vy)

        # Saturate linear control
        if np.linalg.norm(Vec) > vmax:
            Vec = Vec*vmax/np.linalg.norm(Vec)
        [vx, vy] = Vec + addVel

        # Angular control
        thetae = theta-thetad
        # Keep the error between (-pi, pi)
        if thetae > np.pi:
            thetae -= 2*np.pi
        elif thetae < -np.pi:
           thetae += 2*np.pi
        
        # Increase angular velocity for followers
        if (not virtual and abs(thetae) > np.pi/6):
            wmax = self.wmax * 2
            kr = self.kp["kr"] * 2
        else:
            wmax = self.wmax
            kr = self.kp["kr"]
        # Saturate angular control with sigmoid
        w = wmax*np.tanh(-kr*thetae/wmax)

        # Get velocities on inertial frame
        xp = vx*np.cos(theta) - vy*np.sin(theta)
        yp = vx*np.sin(theta) + vy*np.cos(theta)
        thetap = w

        # Estimate current states (open loop)
        x += xp*dt
        y +=  yp*dt
        theta +=  thetap*dt

        return [vx, vy, w, x, y, theta] # Return velocities and states

    # Function for integrating all the calculations for the physical robots and virtual leader
    def startControl(self, dt : float) -> None:
        # Virtual leader
        [vx, vy, w, self.xv, self.yv, self.thetav] = self.control(self.xv, self.yv, self.thetav, self.xd, self.yd, self.thetad, dt, True, np.array([0, 0]))

        # Get desired positions for the physical robots according to virtual leader
        xd1, yd1 = self.xv + self.d_d*np.cos(self.thetav), self.yv + self.d_d*np.sin(self.thetav)
        xd2, yd2 = self.xv + self.d_d*np.cos(np.pi + self.thetav), self.yv + self.d_d*np.sin(np.pi + self.thetav)

        # Control for avoiding colisions between the 2 robots
        dist = np.sqrt((self.x1-self.x2)**2 + (self.y1-self.y2)**2)
        mag = self.k*(dist-2*self.d_d)
        dir1 = np.array([self.x1-self.x2,self.y1-self.y2])/dist
        dir2 = np.array([self.x2-self.x1,self.y2-self.y1])/dist
        vel1 = np.dot(-np.array([[np.cos(self.theta1), np.sin(self.theta1)], [-np.sin(self.theta1), np.cos(self.theta1)]]), mag*dir1)
        vel2 = np.dot(-np.array([[np.cos(self.theta2), np.sin(self.theta2)], [-np.sin(self.theta2), np.cos(self.theta2)]]), mag*dir2)

        # Robot 1
        [vx1, vy1, w1, self.x1, self.y1, self.theta1] = self.control(self.x1, self.y1, self.theta1, xd1, yd1, self.thetav, dt, False, vel1)

        # Robot 2
        [vx2, vy2, w2, self.x2, self.y2, self.theta2] = self.control(self.x2, self.y2, self.theta2, xd2, yd2, self.thetav, dt, False, vel2)

        # Prepare message with velocities for sending to ESP32
        data = f'{vx1:.3f},{vy1:.3f},{w1:.3f}|{vx2:.3f},{vy2:.3f},{w2:.3f}\n'
        # self.send_to_esp32(data)

    def simulation(self) -> None:
        if not self.simulate:
            pygame.init() # Initiate the service

            # Offset for simulating the 4 cuadrants of a cartesian plane
            self.offset_x = WINDOW_WIDTH // 2
            self.offset_y = WINDOW_HEIGHT // 2
            self.scale_factor = 50 # Scaling the real states to match the pixels

            # Create the screen
            self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
            pygame.display.set_caption("Omnidirectional Vehicle Control") # Title
            self.clock = pygame.time.Clock() # Clock for frames
            self.simulate = True

        # For closing the simulation
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        # Clear the screen
        self.screen.fill(WHITE)

        # Draw the desired position
        pygame.draw.circle(self.screen, RED, (self.scale_factor*self.xd + self.offset_x, -self.scale_factor*self.yd + self.offset_y), 8)
        # Draw the vitual leader
        pygame.draw.circle(self.screen, BLUE, (self.scale_factor*self.xv + self.offset_x, -self.scale_factor*self.yv + self.offset_y), 5)
        # Draw the vehicle 1
        pygame.draw.circle(self.screen, GREEN, (self.scale_factor*self.x1 + self.offset_x, -self.scale_factor*self.y1 + self.offset_y), 4)
        # Draw the vehicle 2
        pygame.draw.circle(self.screen, GREEN, (self.scale_factor*self.x2 + self.offset_x, -self.scale_factor*self.y2 + self.offset_y), 4)

        # Update the screen
        pygame.display.flip()
        # Cap the frame rate
        self.clock.tick(60)

    # Function for stopping all active services on the code
    def stop(self) -> None:
        self.cap.release() # Camera
        cv.destroyAllWindows() # OpenCV figures
        self.s.close() # TCP connection
        pygame.quit() # Close simulation
        exit() # Code execution

if __name__ == '__main__':
    print("The Velocity Controller is Running")

    flag = None # Flag for calculating dt
    classObject = control()
    while True:
        try:
            # Get dt
            if (flag == None):
                flag = True
                currTime = time.time()
            elif (flag):
                flag = False
                dt = time.time() - currTime
            else:
                # Control (Manual start)
                # while not classObject.start:
                #     classObject.camera() # Start camera for calibrating
                #     if cv.waitKey(1) == ord('q'):
                #         # Wait for 'q' key, (manual activation)
                #         classObject.start = True
                #     time.sleep(0.02)
                #     os.system('cls')

                # Start control
                classObject.camera()
                # classObject.startControl(dt)
                # classObject.simulation()
                if cv.waitKey(1) == 27:
                    # Esc key for finish
                    break
            time.sleep(0.02)
            os.system('cls') # Clear console
        # Manage exceptions
        except ValueError:
            print(ValueError)
            classObject.stop()