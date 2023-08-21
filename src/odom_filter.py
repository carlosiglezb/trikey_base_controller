#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Vector3, Quaternion, TransformStamped, Transform
from nav_msgs.msg import Odometry
from numpy import zeros, asarray, float64
import tf2_ros

velocity_linear = Vector3(0,0,0)
velocity_angular = Vector3(0,0,0)
position = Point(0,0,0)
orientation = Quaternion(0,0,0,0)

#geometry_msgs.msg convertors
def vector3_convertor(v3):
    output = zeros((3,1))
    output[0], output[1], output[2] = v3.x, v3.y, v3.z
    return asarray(output, dtype = float64)

def point_convertor(pnt):
    output = zeros((3,1))
    output[0], output[1], output[2] = pnt.x, pnt.y, pnt.z
    return asarray(output, dtype = float64)

def quaternion_convertor(quat):
    output = zeros((4,1))
    output[0], output[1], output[2], output[3]= quat.x, quat.y, quat.z, quat.w
    return asarray(output, dtype = float64)

#tf publisher
def filtered_odom_tf_br(vel_linear, orientation):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "filtered_odom"
    t.child_frame_id = "base_link"
    t.transform.translation = vel_linear
    t.transform.rotation = orientation
    
    br.sendTransform(t)
    

class odom_filter:
    def __init__(self):
        self.v3_lin, self.v3_ang= zeros((3,1)), zeros((3,1))
        self.point, self.quaternion = zeros((3,1)), zeros((4,1))
    
    def lpf(self, v3_lin_in, v3_ang_in, point_in, quat_in):
        #converting from geometry_msgs.msg to matrix
        v3_lin_conv,v3_ang_conv = vector3_convertor(v3_lin_in),vector3_convertor(v3_ang_in)
        point_conv, quat_conv = point_convertor(point_in), quaternion_convertor(quat_in)
        
        #filtering through 1st-order low pass filter
        alpha = 0.001       
        new_v3_lin = alpha*v3_lin_conv + (1-alpha)*self.v3_lin
        new_v3_ang = alpha*v3_ang_conv + (1-alpha)*self.v3_ang
        new_pnt = alpha*point_conv + (1-alpha)*self.point
        new_quat  = alpha*quat_conv + (1-alpha)*self.quaternion
        
        #converting from matrix to geometry_msgs.msg formats
        out_v3_lin, out_v3_ang = Vector3(float(new_v3_lin[0]),float(new_v3_lin[1]),float(new_v3_lin[2])), Vector3(float(new_v3_ang[0]),float(new_v3_ang[1]),float(new_v3_ang[2]))
        out_pnt, out_quat = Point(float(new_pnt[0]),float(new_pnt[1]),float(new_pnt[2])), Quaternion(float(new_quat[0]),float(new_quat[1]),float(new_quat[2]),float(new_quat[3]))
        
        #updating current inputs to old inputs
        self.v3_lin, self.v3_ang = new_v3_lin, new_v3_ang
        self.point, self.quaternion = new_pnt, new_quat
        
        return  out_v3_lin, out_v3_ang, out_pnt, out_quat

def callback(odom: Odometry):
    global velocity_linear, velocity_angular, position, orientation
    velocity_linear, velocity_angular, position, orientation = odom.twist.twist.linear, odom.twist.twist.angular, odom.pose.pose.position, odom.pose.pose.orientation

if __name__ == '__main__':
    rospy.init_node("odom_filter")
    output = rospy.Publisher("/filtered_odom", Odometry, queue_size=10)
    input = rospy.Subscriber("/odom", Odometry, callback)

    rate = rospy.Rate(10)
    filter = odom_filter()
    msg = Odometry()

    
    while not rospy.is_shutdown():
        msg.twist.twist.linear, msg.twist.twist.angular, msg.pose.pose.position, msg.pose.pose.orientation = filter.lpf(velocity_linear, velocity_angular, position, orientation)
        filtered_odom_tf_br(msg.twist.twist.linear, msg.pose.pose.orientation)
        output.publish(msg)
        rate.sleep()