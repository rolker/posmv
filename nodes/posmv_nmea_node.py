#!/usr/bin/env python

import serial
import socket
import rospy
import rosbag
from  sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import TimeReference
from marine_msgs.msg import NavEulerStamped
import datetime

def posmv_nmea_listener():
    position_pub = rospy.Publisher('/base/position',NavSatFix,queue_size=10)
    timeref_pub = rospy.Publisher('/base/time_reference',TimeReference,queue_size=10)
    orientation_pub = rospy.Publisher('/base/orientation',NavEulerStamped,queue_size=10)
    rospy.init_node('posmv_nmea')
    input_type = rospy.get_param('/posmv_nmea/input_type')
    input_address = rospy.get_param('/posmv_nmea/input')
    input_speed = rospy.get_param('/posmv_nmea/input_speed')
    output_port = int(rospy.get_param('/posmv_nmea/output',0))
    output_address = rospy.get_param('/posmv_nmea/output_address','<broadcast>')
    
    if input_type == 'serial':
        serial_in = serial.Serial(input_address, int(input_speed))
    
    if output_port > 0:
        udp_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        udp_out.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    else:
        udp_out = None
            
    timestamp = datetime.datetime.utcfromtimestamp(rospy.Time.now().to_time()).isoformat()
    bag = rosbag.Bag('nodes/posmv_nmea_'+('-'.join(timestamp.split(':')))+'.bag', 'w', rosbag.Compression.BZ2)

    while not rospy.is_shutdown():
        if input_type == 'serial':
            nmea_in = serial_in.readline()
            print nmea_in
            if udp_out is not None:
                udp_out.sendto(nmea_in, (output_address,output_port))


if __name__ == '__main__':
    try:
        posmv_nmea_listener()
    except rospy.ROSInterruptException:
        pass


