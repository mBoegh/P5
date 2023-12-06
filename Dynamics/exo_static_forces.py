import cv2 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import time

max_angle = 180-10 #deg
min_angle = 30     #deg

upper_length = 220/1000 #m
lower_length = 220/1000 #m

upper_mount = 100/1000 #m
lower_mount = 100/1000 #m

spool_radius = 25/1000 #m

time_step = 1/30 #s

frame = [900,900] #[height, width]
shoulder_pos = [50, int(frame[1]*0.25)]
# elbow_pos = np.add(shoulder, [0,upper_length*1000])
# wrist_pos = np.add(elbow, [0,lower_length*1000])
# upper_mount_pos = np.add(elbow, [0,-upper_mount_offset*1000])
# lower_mount_pos = np.add(elbow, [0,lower_mount_offset*1000])

# cable = abs(upper_mount_offset - lower_mount_offset) #m

cable_angle = 0 #rad
shoulder_ang = np.deg2rad(0) #rad
elbow_ang = np.deg2rad(180-135)    #rad


start_time = time.time() #s
prev_time = start_time #s

upper_mass = 2 #kg
lower_mass = 2 #kg
payload_mass = 1 #kg cite: EXOTIC
upper_inertia = (1/3)*upper_mass*np.square(upper_length) #kg*mm2
lower_inertia = (1/3)*lower_mass*np.square(lower_length) #kg*mm2
grav = 9.82 #m/s^2

upper_gravity = [0,upper_mass*grav] #N
lower_gravity = [0,lower_mass*grav] #N
lower_force = 0 #N
payload_gravity = [0,payload_mass*grav] #N
payload_force = 0 #N

cable_force_tangent = 0 #N
cable_force_radial = 0 #N
cable_force = 0 #N
elbow_torque = 0 #Nm
motor_torque = 0 #Nm

pulley_wheels = 1
gearing = 6


#                 0          1
mode_select = ["cable", "angle"]

mode = mode_select[ 1 ]

def position(type, amount):
    cable_length = 0
    cable_angle = 0
    
    if type == "cable":
        cable_length = amount
        cable_angle = np.sqrt(np.square(upper_mount)+np.square(lower_mount) - 2*upper_mount*lower_mount*np.cos(amount))

    if type == "angle":
        cable_angle = amount
        cable_length = np.arccos(np.divide((np.square(upper_mount) + np.square(lower_mount) - np.square(amount)), (2*upper_mount*lower_mount)))

    return cable_length, cable_angle


def dynamics(shoulder, elbow):
    # Tangential gravity force on lower arm
    lower_force = [(np.cos(-shoulder+elbow)*lower_length*0.5/lower_length*0.5)*lower_gravity[0], (np.sin(-shoulder+elbow)*lower_length*0.5/lower_length*0.5)*lower_gravity[1]]

    # Tangential gravity force on the payload
    payload_force = [(np.cos(-shoulder+elbow)*lower_length/lower_length)*payload_gravity[0], (np.sin(-shoulder+elbow)*lower_length/lower_length)*payload_gravity[1]]

    # Use the gravity forces to calulate the resulting torque on the joint
    joint_torque = lower_force[1]*lower_length/2 + payload_force[1]*lower_length #Nm
    # print(joint_torque)

    # Use joint torque to calculate the necessary tangential force for oppose the gravity torque
    cable_force_tangent = joint_torque/lower_mount
    # print(cable_force_tangent)


    cable_length = np.sqrt(np.square(upper_mount)+np.square(lower_mount)-2*upper_mount*lower_mount*np.cos(elbow))
    # print(cable_length)
    cable_angle = np.arccos(np.divide((np.square(cable_length) + np.square(lower_mount) - np.square(upper_mount)), (2*cable_length*lower_mount)))
    # print(cable_angle)

    # Calculate the force along the length of the arm
    cable_force_radial = abs(np.tan(abs(np.pi/2-cable_angle))*cable_force_tangent)
    # print(cable_force_radial)

    # Calculate the force along the pulling cable based on the the tangential forces
    F_cable = np.sqrt(np.square(cable_force_tangent)+np.square(cable_force_radial))/pulley_wheels
    # print(F_cable)

    #Calculate the motor torque
    motor_torque = (F_cable*spool_radius)/gearing
    # print(motor_torque)

    return motor_torque, joint_torque


def draw():                
    blank = np.zeros(frame+[3])

    upper_mount_pos = np.add(shoulder_pos, [np.sin(shoulder_ang)*(upper_length-upper_mount)*1000, np.cos(shoulder_ang)*(upper_length-upper_mount)*1000])
    int_upper_mount_pos = upper_mount_pos.astype(int)

    elbow_pos = np.add(shoulder_pos, [np.sin(shoulder_ang)*upper_length*1000, np.cos(shoulder_ang)*upper_length*1000])
    int_elbow_pos = elbow_pos.astype(int)

    lower_mount_pos = np.add(int_elbow_pos, [np.cos(-shoulder_ang+elbow_ang-np.pi/2)*lower_mount*1000, np.sin(-shoulder_ang+elbow_ang-np.pi/2)*lower_mount*1000])
    int_lower_mount_pos = lower_mount_pos.astype(int)

    wrist_pos = np.add(elbow_pos, [np.cos(-shoulder_ang+elbow_ang-np.pi/2)*lower_length*1000, np.sin(-shoulder_ang+elbow_ang-np.pi/2)*lower_length*1000])
    int_wrist_pos = wrist_pos.astype(int)

    upper_mid_pos = np.add(shoulder_pos, [np.sin(shoulder_ang)*(upper_length/2)*1000, np.cos(shoulder_ang)*(upper_length/2)*1000])
    int_upper_mid_pos = upper_mid_pos.astype(int)

    lower_mid = np.add(elbow_pos, [np.cos(-shoulder_ang+elbow_ang-np.pi/2)*lower_length*0.5*1000, np.sin(-shoulder_ang+elbow_ang-np.pi/2)*lower_length*0.5*1000]) #np.add((np.add(lower_mount,-elbow_pos)/lower_mount) * lower_length/2, elbow_pos)
    int_lower_mid_pos = lower_mid.astype(int)


    # # Draw boring joints
    cv2.circle(blank, shoulder_pos, 10, [255,255,255], 4)
    cv2.circle(blank, int_wrist_pos, 10, [255,255,255], 4)
    cv2.circle(blank, int_upper_mount_pos, 10, [255,255,255], 4)
    cv2.circle(blank, int_lower_mount_pos, 10, [255,255,255], 4)

    # # Draw limbs
    cv2.line(blank, shoulder_pos, int_elbow_pos, [255,255,255], 4) 
    cv2.line(blank, int_elbow_pos, int_wrist_pos, [255,255,255], 4) 

    # # Draw cable 
    cv2.line(blank, int_upper_mount_pos, int_lower_mount_pos, [255,255,255], 4)

    f_mult = 5
    tau_mult = 5

    # # Draw gravity
    cv2.line(blank, int_lower_mid_pos, np.add(int_lower_mid_pos, [int(lower_gravity[0]*f_mult), int(lower_gravity[1]*f_mult)]), [0,255,0], 1)
    cv2.line(blank, int_wrist_pos, np.add(int_wrist_pos, [int(payload_gravity[0]*f_mult), int(payload_gravity[1]*f_mult)]), [0,255,0], 1)
    
    # # Draw gravity decompositions
    # # cv2.line(blank, lower_mid, np.add(lower_mid, [int(np.sin(-theta)*F_lower_draw[0]),int(np.cos(-theta)*F_lower_draw[0])]), [0,255,0], 2)
    # F_lower_draw = np.multiply(F_lower,f_mult)
    # cv2.line(blank, lower_mid, np.add(lower_mid, [int(np.cos(theta)*F_lower_draw[1]),int(np.sin(theta)*F_lower_draw[1])]), [0,255,0], 1)
    # F_payload_draw = np.multiply(F_payload,f_mult)
    # cv2.line(blank, wrist, np.add(wrist, [int(np.cos(theta)*F_payload_draw[1]),int(np.sin(theta)*F_payload_draw[1])]), [0,255,0], 1)


    # cable_force_90deg = [np.cos(theta-np.pi)*F_cable_90deg, np.sin(theta-np.pi)*F_cable_90deg]
    # cable_force_90deg_draw = np.add(np.multiply(cable_force_90deg,f_mult), int_lower_mount)
    # cv2.line(blank, int_lower_mount, cable_force_90deg_draw.astype(int), [0,255,0], 1)
    

    # cable_force = np.add(int_lower_mount, np.multiply(np.divide(np.add(upper_mount, -int_lower_mount), cable*1000),F_cable*f_mult))

    # cv2.line(blank, int_lower_mount, cable_force.astype(int), [0,255,0], 2)

    # # cv2.line(blank, cable_force_90deg_draw.astype(int), cable_force_draw.astype(int), [0,255,0], 2)
    
    # # Draw joint torque
    if joint_torque > 0:
        cv2.circle(blank, elbow_pos.astype(int), int(np.multiply(joint_torque, tau_mult)), [255,255,255], 4)
    else:
        cv2.circle(blank, elbow_pos.astype(int), 10, [0,0,255], 4)

    # # Debugging text
    # cv2.putText(blank, "Joint angle:   " + str(int(np.rad2deg(theta))) + " deg", [100,40], cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
    # cv2.putText(blank, "Cable length: " + str(int(cable*1000)) + " mm", [100,80], cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
    # cv2.putText(blank, "Cable force:  " + str(int(F_cable)) + " N", [100,120], cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(blank, "Motor torque: " + str(float(int(motor_torque*100))/100) + " Nm", [50,50], cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
    

    cv2.imshow("Elbow joint", blank) # Displays the image
    cv2.waitKey(0)


def plot(what):
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

    # Make data.
    X = np.deg2rad(np.arange(0, 180, 1)) # Shoulder joint
    Y = np.deg2rad(np.arange(30, 160, 1)) # Elbow joint
    X, Y = np.meshgrid(X, Y)

    M, J = dynamics(X,Y)

    if True:
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                if np.deg2rad(180)-X[i,j]+Y[i,j] < np.deg2rad(180):
                    M[i,j] = 0
                    J[i,j] = 0
                if M[i,j] > 7:
                    M[i,j] = 7
    

    if what == "Joint":
        surf = ax.plot_surface(np.rad2deg(X), np.rad2deg(Y), J, cmap=cm.coolwarm,
                        linewidth=0, antialiased=False)
    elif what == "Motor":
        # ax.plot(np.linspace(0,180,180),np.linspace(30,170,180),np.linspace(0,8.54,180), 'ro', alpha=0.5)
        surf = ax.plot_surface(np.rad2deg(X), np.rad2deg(Y), M, cmap=cm.coolwarm,
                        linewidth=0, antialiased=False)
        # ax.plot(np.linspace(0,180,180), np.linspace(30,170,180), np.linspace(0,8.54,180), label='parametric curve')
        # ax.plot([90] * len(X), np.rad2deg(X[90,:]), M[90,:], label='parametric curve')
        # ax = plt.axes(projection='3d')


        # Attempt at drawing a line on the graph
        # #ax.scatter(xx, yy, zz, c='r', marker='o')
        # ax.plot(np.linspace(0,180,180), np.linspace(30,170,180), np.linspace(0,8.54,180), 'ro', alpha=0.5) # note the 'ro' (no '-') and the alpha

        # ax.plot_surface(np.rad2deg(X), np.rad2deg(Y), M, rstride=10, cstride=10,
        #                 cmap='viridis', edgecolor='none')


        # Plot with cool graphs on the sides
        # ax.plot_surface(np.rad2deg(X), np.rad2deg(Y), M, edgecolor='royalblue', lw=0.5, rstride=8, cstride=8,
        #                 alpha=0.3)
        # Plot projections of the contours for each dimension.  By choosing offsets
        # that match the appropriate axes limits, the projected contours will sit on
        # the 'walls' of the graph.
        # # ax.contour(np.rad2deg(X), np.rad2deg(Y), M, zdir='z', offset=0, cmap='coolwarm')
        # ax.contour(np.rad2deg(X), np.rad2deg(Y), M, zdir='x', offset=-15, cmap='coolwarm')
        # ax.contour(np.rad2deg(X), np.rad2deg(Y), M, zdir='y', offset=180, cmap='coolwarm')

    else:
        print("Wrong plot parameter")
        return

    ax.set_xlabel("Shoulder Joint Angle [deg]")
    ax.set_ylabel("Elbow Joint Angle [deg]")
    ax.set_zlabel("Motor torque [Nm]")


    # Limit the allowed decimal places to two
    ax.zaxis.set_major_formatter('{x:.02f}')

    # Add a color bar which maps values to colors.
    fig.colorbar(surf, shrink=0.5, aspect=10, location='left')

    plt.show()

# cable_length, cable_angle = position("angle", elbow_ang)
motor_torque, joint_torque = dynamics(shoulder_ang, elbow_ang)

# draw()

plot("Motor")


