import cv2 
import numpy as np
import matplotlib.pyplot as plt
import time

class LimbSim:
    def __init__(self):
        self.max_angle = 170 #deg
        self.min_angle = 30 #deg

        self.upper_length = 400/1000 #m
        self.lower_length = 400/1000 #m

        self.upper_mount_offset = 150/1000 #m
        self.lower_mount_offset = 100/1000 #m

        self.r_spool = 2/100 #m

        self.time_step = 1/30 #s

        self.frame = [800,600] #[height, width]
        self.shoulder = [50,50]
        self.elbow = np.add(self.shoulder, [0,self.upper_length*1000])
        self.wrist = np.add(self.elbow, [0,self.lower_length*1000])
        self.upper_mount = np.add(self.elbow, [0,-self.upper_mount_offset*1000])
        self.lower_mount = np.add(self.elbow, [0,self.lower_mount_offset*1000])

        self.cable = abs(self.upper_mount_offset - self.lower_mount_offset) #m
        self.cable_angle = 0 #rad
        self.theta = np.pi/2 #rad

        self.extend = True
        self.cable_color = [255,0,0]
        self.target_angvel = 0.01 #rad/s

        self.start_time = time.time() #s
        self.prev_time = self.start_time #s

        self.m_upper = 2 #kg
        self.m_lower = 2 #kg
        self.m_payload = 1 #kg cite: EXOTIC
        self.I_lower = (1/3)*self.m_lower*np.square(self.lower_length) #kg*mm2
        self.grav = 9.82 #m/s^2

        self.F_lower_g = 0 #N
        self.F_lower = 0 #N
        self.F_payload_g = 0 #N
        self.F_payload = 0 #N

        self.F_cable_90deg = 0 #N
        self.F_cable_0deg = 0 #N
        self.F_cable = 0 #N
        self.joint_torque = 0 #Nm
        self.motor_torque = 0 #Nm
    

        #                      0          1
        self.mode_select = ["length", "velocity"]
        self.mode = self.mode_select[1]

    def cable2Angle(self, cable):
        return np.arccos(np.divide((np.square(self.upper_mount_offset) + np.square(self.lower_mount_offset) - np.square(cable)), (2*self.upper_mount_offset*self.lower_mount_offset)))

    def angle2Cable(self, angle):
        return np.sqrt(np.square(self.upper_mount_offset)+np.square(self.lower_mount_offset) - 2*self.upper_mount_offset*self.lower_mount_offset*np.cos(angle))

    def gravity(self, angle, cable):
        # Vertical gravity force on lower arm
        self.F_lower_g = [0,self.m_lower*self.grav]
        # Tangential gravity force on lower arm
        self.F_lower = [(np.cos(angle)*self.lower_mount_offset/self.lower_mount_offset)*self.F_lower_g[0], (np.sin(angle)*self.lower_mount_offset/self.lower_mount_offset)*self.F_lower_g[1]]
        
        # Vertical gravity force on the payload
        self.F_payload_g = [0,self.m_payload*self.grav]
        # Tangential gravity force on the payload
        self.F_payload = [(np.cos(angle)*self.lower_length/self.lower_length)*self.F_payload_g[0], (np.sin(angle)*self.lower_length/self.lower_length)*self.F_payload_g[1]]

        # Use the gravity forces to calulate the resulting torque on the joint
        self.joint_torque = self.F_lower[1]*self.lower_length/2 + self.F_payload[1]*self.lower_length #Nm
        
        # Use joint torque to calculate the necessary tangential force for oppose the gravity torque
        self.F_cable_90deg = self.joint_torque/self.lower_mount_offset

        # Calculate the angle of the cable from the arm
        self.cable_angle = np.arccos((self.lower_mount_offset*self.lower_mount_offset + cable*cable - self.upper_mount_offset*self.upper_mount_offset )/( 2*self.lower_mount_offset*cable))

        # Calculate the force along the length of the arm
        self.F_cable_0deg = abs(np.tan(abs(np.pi/2-self.cable_angle))*self.F_cable_90deg)

        # Don't ask why the number of pulleys is decided here
        pulley_wheels = 1
        
        # Calculate the force along the pulling cable based on the the tangential forces
        F_cable = np.sqrt(np.square(self.F_cable_90deg)+np.square(self.F_cable_0deg))/pulley_wheels

        #Calculate the motor torque
        self.motor_torque = F_cable*self.r_spool

        return F_cable


    def length(self):
        if self.theta > np.deg2rad(self.max_angle):
            self.extend = False
            print("Retract")
        if self.theta < np.deg2rad(self.min_angle):
            self.extend = True
            print("Extend")

        if self.extend:
            self.cable +=0.5
        else:
            self.cable -=0.5
        
        self.theta = self.cable2Angle(self.cable)
    
    def angVel(self, aVel):
        if self.extend:
            self.theta = self.theta + aVel
        else:
            self.theta = self.theta - aVel
        
        self.cable = self.angle2Cable(self.theta)

        if self.theta > np.deg2rad(self.max_angle):
            self.extend = False
            print("Retract")
        if self.theta < np.deg2rad(self.min_angle):
            self.extend = True
            print("Extend")

    def torque_control(self, target):
        pass


    def plot(self):
        min = abs(self.upper_mount_offset - self.lower_mount_offset)
        max = self.upper_mount_offset + self.lower_mount_offset

        x = np.linspace(min, max, int((max-min)*5000))
        y = self.cable2Angle(x)
        y2 = self.gravity(y,x)

        # fig, x1 = plt.subplots()

        # x1.plot([min,max], [0,180], linewidth=1.0, linestyle='dashed')
        # x1.axhline(self.min_angle, linewidth=1.0, linestyle='dotted')
        # x1.axhline(self.max_angle, linewidth=1.0, linestyle='dotted')

        # x1.plot(x, np.rad2deg(y), linewidth=2.0)

        # x1.set(xlim=(min, max), ylim=(0, 180))

        # # plt.show()



        # fig, x2 = plt.subplots()

        # x2.axhline(y2[-2], linewidth=1.0, linestyle='dotted')

        # x2.plot(np.rad2deg(y), y2, linewidth=2.0)

        # x2.set(xlim=(0, 180))



        fig, x3 = plt.subplots()

        y3 = y2*self.r_spool

        x3.axhline(y3[-2], linewidth=1.0, linestyle='dotted')
        x3.axhline(y3[-2]/2, linewidth=1.0, linestyle='dotted')
        x3.plot(np.rad2deg(y), y3, linewidth=2.0)

        x3.set(xlim=(self.min_angle, self.max_angle))

        plt.show()

    def draw(self):
        start_time = time.time()
        prev_time = start_time
        while True:
            if time.time() - prev_time  >= self.time_step:
                prev_time = time.time()
                
                self.blank = np.zeros(self.frame+[3])

                if self.mode == "length":
                    self.length()
                elif self.mode == "velocity":
                    self.angVel(self.target_angvel)
            
                self.F_cable = self.gravity(self.theta, self.cable)

                self.lower_mount = np.add(self.elbow, [np.cos(self.theta-np.pi/2)*self.lower_mount_offset*1000, np.sin(self.theta-np.pi/2)*self.lower_mount_offset*1000])
                self.int_lower_mount = self.lower_mount.astype(int)

                self.lower_mid = np.add((np.add(self.lower_mount,-self.elbow)/self.lower_mount_offset) * self.lower_length/2, self.elbow)
                self.lower_mid = self.lower_mid.astype(int)

                self.wrist = np.add((np.add(self.lower_mount,-self.elbow)/self.lower_mount_offset) * self.upper_length, self.elbow)
                self.wrist = self.wrist.astype(int)

                
                
                if self.extend == True:
                    self.cable_color = [255,0,0]
                else:
                    self.cable_color = [0,0,255]

                # Draw boring joints
                cv2.circle(self.blank, self.shoulder, 10, [255,255,255], 4)
                cv2.circle(self.blank, self.wrist, 10, [255,255,255], 4)
                cv2.circle(self.blank, self.upper_mount.astype(int), 10, self.cable_color, 4)
                cv2.circle(self.blank, self.int_lower_mount, 10, self.cable_color, 4)

                # Draw limbs
                cv2.line(self.blank, self.shoulder, self.elbow.astype(int), [255,255,255], 4) 
                cv2.line(self.blank, self.elbow.astype(int), self.wrist, [255,255,255], 4) 

                # Draw cable 
                cv2.line(self.blank, self.upper_mount.astype(int), self.int_lower_mount, self.cable_color, 4)

                f_mult = 1
                tau_mult = 1

                # Draw gravity
                cv2.line(self.blank, self.lower_mid, np.add(self.lower_mid, [int(self.F_lower_g[0]*f_mult), int(self.F_lower_g[1]*f_mult)]), [0,255,0], 1)
                cv2.line(self.blank, self.wrist, np.add(self.wrist, [int(self.F_payload_g[0]*f_mult), int(self.F_payload_g[1]*f_mult)]), [0,255,0], 1)
                
                # Draw gravity decompositions
                # cv2.line(self.blank, self.lower_mid, np.add(self.lower_mid, [int(np.sin(-self.theta)*F_lower_draw[0]),int(np.cos(-self.theta)*F_lower_draw[0])]), [0,255,0], 2)
                F_lower_draw = np.multiply(self.F_lower,f_mult)
                cv2.line(self.blank, self.lower_mid, np.add(self.lower_mid, [int(np.cos(self.theta)*F_lower_draw[1]),int(np.sin(self.theta)*F_lower_draw[1])]), [0,255,0], 1)
                F_payload_draw = np.multiply(self.F_payload,f_mult)
                cv2.line(self.blank, self.wrist, np.add(self.wrist, [int(np.cos(self.theta)*F_payload_draw[1]),int(np.sin(self.theta)*F_payload_draw[1])]), [0,255,0], 1)


                cable_force_90deg = [np.cos(self.theta-np.pi)*self.F_cable_90deg, np.sin(self.theta-np.pi)*self.F_cable_90deg]
                cable_force_90deg_draw = np.add(np.multiply(cable_force_90deg,f_mult), self.int_lower_mount)
                cv2.line(self.blank, self.int_lower_mount, cable_force_90deg_draw.astype(int), [0,255,0], 1)
                

                cable_force = np.add(self.int_lower_mount, np.multiply(np.divide(np.add(self.upper_mount, -self.int_lower_mount), self.cable*1000),self.F_cable*f_mult))

                cv2.line(self.blank, self.int_lower_mount, cable_force.astype(int), [0,255,0], 2)

                # cv2.line(self.blank, cable_force_90deg_draw.astype(int), cable_force_draw.astype(int), [0,255,0], 2)
                
                # Draw joint torque
                cv2.circle(self.blank, self.elbow.astype(int), int(np.multiply(self.joint_torque, tau_mult)), [255,255,255], 4)

                # Debugging text
                cv2.putText(self.blank, "Joint angle:   " + str(int(np.rad2deg(self.theta))) + " deg", [100,40], cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
                cv2.putText(self.blank, "Cable length: " + str(int(self.cable*1000)) + " mm", [100,80], cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
                cv2.putText(self.blank, "Cable force:  " + str(int(self.F_cable)) + " N", [100,120], cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
                cv2.putText(self.blank, "Motor torque: " + str(float(int(self.motor_torque*100))/100) + " Nm", [100,160], cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
                

                cv2.imshow("Elbow joint", self.blank) # Displays the image

                if cv2.waitKey(20) & 0xFF == ord(' '): # Breaks the loop when space is pressed
                    break
                
sim = LimbSim()

sim.draw()
sim.plot()