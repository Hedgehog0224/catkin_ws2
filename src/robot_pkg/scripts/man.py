#!/usr/bin/python3
from robot_pkg.msg import Servodata
import rospy

class manual():
    def __init__(self):
        rospy.init_node('manual')
        self.predData = [0,0,0,0,0,0]
        self.pub = rospy.Publisher('manipulatorData', Servodata, queue_size=10)
        self.msg = Servodata()


    def pubData(self):
        # self.r = float(input("r:"))
        # self.t = float(input("thet:"))
        # self.f = float(input("fi:"))
        # self.msg.servo0 = int(self.r*100)
        # self.msg.servo1 = int(self.t*100)
        # self.msg.servo2 = int(self.f*100)
        # self.msg.servo3 = int(0*100)
        # self.msg.servo4 = int(0*100)
        # self.msg.servo5 = int(0*100)
        self.s1 = float(input("s1:"))
        self.s2 = float(input("s2:"))
        self.s3 = float(input("s3:"))
        self.s4 = float(input("s4:"))
        self.s5 = float(input("s5:"))
        self.s6 = float(input("s6:"))

        self.msg.servo0 = int(self.s1)
        self.msg.servo1 = int(self.s2)
        self.msg.servo2 = int(self.s3)
        self.msg.servo3 = int(self.s4)
        self.msg.servo4 = int(self.s5)
        self.msg.servo5 = int(self.s6)
        # try:
        self.pub.publish(self.msg)
        # except:
        #     print("ERROR")
        self.predData = [self.msg.servo0, self.msg.servo1, self.msg.servo2, self.msg.servo3, self.msg.servo4, self.msg.servo5]
        rospy.sleep(0.4)

def main():
    Ob = manual()
    while not rospy.is_shutdown():
        Ob.pubData()

if __name__=="__main__":
    main()