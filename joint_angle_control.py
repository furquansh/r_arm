#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math

def talker():

    #Initialiizing Publishers
    Joint_me = rospy.Publisher('/rrbot/joint_me_position_controller/command',Float64, queue_size=10)
    Joint_1 = rospy.Publisher('/rrbot/joint1_position_controller/command',Float64, queue_size=10)
    Joint_2 = rospy.Publisher('/rrbot/joint2_position_controller/command',Float64, queue_size=10)
    Joint_3 = rospy.Publisher('/rrbot/joint3_position_controller/command',Float64, queue_size=10)
    Joint_4 = rospy.Publisher('/rrbot/joint4_position_controller/command',Float64, queue_size=10)
    Joint_G1 = rospy.Publisher('/rrbot/jointG1_position_controller/command',Float64, queue_size=10)
    Joint_G2 = rospy.Publisher('/rrbot/jointG2_position_controller/command',Float64, queue_size=10)


    
    rospy.init_node('joint_state_pub' , anonymous=False)
    joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        print("All the theta values to be entered in degrees")

        theta_1 = float(input("{:22s}".format("Enter theta_1: ")))
        theta_2 = float(input("{:22s}".format("Enter theta_2: ")))
        theta_3 =  float(input("{:22s}".format("Enter theta_3: ")))
        theta_4 =  float(input("{:22s}".format("Enter theta_4: ")))
        theta_5 =  float(input("{:22s}".format("Enter theta_5: ")))
        theta_6 =  float(input("{:22s}".format("Enter theta_6: ")))
        theta_7 =  float(input("{:22s}".format("Enter theta_7: ")))


        if -180.0 <= theta_1<= 180.0 and -180.0 <= theta_2 <= 180.0 and -180.0 <= theta_3 <= 180.0 and -180.0 <= theta_4 <= 180.0 and  -180.0 <= theta_5<= 180.0 and -180.0 <= theta_6<= 180.0 and -180.0 <= theta_7<= 180.0: 
            joint[0] = (theta_1)*math.pi/180
            joint[1] = (theta_2)*math.pi/180
            joint[2] = (theta_3)*math.pi/180
            joint[3] = (theta_4)*math.pi/180
            joint[4] = (theta_5)*math.pi/180
            joint[5] = (theta_6)*math.pi/180
            joint[6] = (theta_7)*math.pi/180

            rospy.loginfo("\ntheta_base = %f\ntheta_shoulder = %f\ntheta_elbow = %f\ntheta_claw = %f", joint[0], joint[1], joint[2], joint[3])
            Joint_me.publish(joint[0])
            Joint_1.publish(joint[1])
            Joint_2.publish(joint[2])
            Joint_3.publish(joint[3])
            Joint_4.publish(joint[4])
            Joint_G1.publish(joint[5])
            Joint_G2.publish(joint[6])

        else:
            print ("Enter angles in range 0 to 180")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass