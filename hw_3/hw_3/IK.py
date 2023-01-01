import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose

import math
import numpy as np

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'topic',
            self.listener_callback,
            1)
        self.subscription = self.create_subscription(
            Pose,
            'topic_inv',
            self.listener_callback_inv,
            1)
        self.subscription  # prevent unused variable warning
        

    def listener_callback(self, msg):
        
        q1=msg.data[0] 
        q2=msg.data[1]
        q3=msg.data[2]
        l1=1 #assuming link length to be of 1 unit
        l2=1 #assuming link length to be of 1 unit
        l3=1 #assuming link length to be of 1 unit
        
        A1 = np.array([[math.cos(q1),0, (-math.sin(q1)),0],[math.sin(q1),0,math.cos(q1),0],[0,-1,0,l1],[0,0,0,1]])
        A2 = np.array([[math.cos(q2),(-math.sin(q2)),0,(l2*math.cos(q2))],[math.sin(q2),math.cos(q2),0,(l2*math.sin(q2))],[0,0,1,0],[0,0,0,1]])
        A3 = np.array([[math.cos(q3),(-math.sin(q3)),0,(l3*math.cos(q3))],[math.sin(q3),math.cos(q3),0,(l3*math.sin(q3))],[0,0,1,0],[0,0,0,1]])
        
        
        t = A1.dot(A2)
        T = t.dot(A3)

        print("The pose of the end-effector is:")
        print(T)
        
    def listener_callback_inv(self, msg_inv):
        
        xc = msg_inv.position.x 
        yc = msg_inv.position.y
        zc = msg_inv.position.z
        l1 = 1.0	#assuming l1 link length 
        l2 = 1.0	#assuming l2 link length 
        l3 = 1.0	#assuming l3 link length 
        print("The link lengths are l1: ",l1, ", l2: ",l2,", l3: ",l3)
        theta1 = np.arctan2([yc],[xc]) #joint1 variable
        s = np.absolute(l1 - zc) #difference between zc and link1
        r = np.sqrt(np.square(xc)+np.square(yc)) #projection of links 2 and 3 on x-y plane
        c = np.sqrt(np.square(r)+np.square(s)) #hypotenuse of triangle formed by r and s
        D = (np.square(l2)+np.square(l3)-np.square(c))/(2*l2*l3) #cos(gamma)
        gamma = np.arctan2([np.sqrt(1-np.square(D))], [D])
        #gamma = np.arccos(D)
        d = (np.square(l2)+np.square(c)-np.square(l3))/(2*l2*c) #cos(psi)
        psi = np.arctan2([np.sqrt(1-np.square(d))], [d])
        #psi = np.arccos(d)
        alpha = np.arctan2([s],[r])
        theta3 = np.radians([180])-gamma #joint3 variable
        theta2 = alpha - psi #join2 variable
        
        print ("xc = ",xc,"yc = ",yc,"zc = ",zc)
        print("The angles are:")
        print("Theta 1 = ",theta1)
        print("Theta 2 = ",theta2)
        print("Theta 3 = ",theta3)
        print("c:",c)
        print("d:",d)
        print("D:",D)
        print("alpha:",alpha)
        print("psi:",psi)


                     

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)


    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 
    main() 
