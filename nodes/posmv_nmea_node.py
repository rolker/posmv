#!/usr/bin/env python3

import serial
import socket
import rospy
from  sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import TimeReference
from marine_msgs.msg import NavEulerStamped
import datetime
import calendar

def posmv_nmea_listener():
    rospy.init_node('posmv_nmea')
    position_pub = rospy.Publisher('~position',NavSatFix,queue_size=10)
    timeref_pub = rospy.Publisher('~time_reference',TimeReference,queue_size=10)
    orientation_pub = rospy.Publisher('~orientation',NavEulerStamped,queue_size=10)
    input_type = rospy.get_param('~input_type')
    input_address = rospy.get_param('~input','')
    input_speed = rospy.get_param('/~input_speed',0)
    input_port = int(rospy.get_param('~input_port',0))
    output_port = int(rospy.get_param('~output',0))
    output_address = rospy.get_param('~output_address','<broadcast>')
    
    if input_type == 'serial':
        serial_in = serial.Serial(input_address, int(input_speed))
    else:
        udp_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_in.bind(('',input_port))
    
    if output_port > 0:
        udp_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        udp_out.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    else:
        udp_out = None
            
    while not rospy.is_shutdown():
        if input_type == 'serial':
            nmea_ins = (serial_in.readline(),)
            #print nmea_in
            if udp_out is not None:
                for nmea_in in nmea_ins:
                    udp_out.sendto(nmea_in, (output_address,output_port))
        else:
            nmea_in,addr = udp_in.recvfrom(2048)
            print (addr, nmea_in)
            nmea_ins = nmea_in.split('\n')
        now = rospy.get_rostime()
        for nmea_in in nmea_ins:
            try:
                nmea_parts = nmea_in.decode('utf-8').strip().split(',')
                if len(nmea_parts):
                    #print nmea_parts
                    try:
                        if nmea_parts[0][3:] == 'ZDA' and len(nmea_parts) >= 5:
                            tref = TimeReference()
                            tref.header.stamp = now
                            hour = int(nmea_parts[1][0:2])
                            minute = int(nmea_parts[1][2:4])
                            second = int(nmea_parts[1][4:6])
                            ms = int(float(nmea_parts[1][6:])*1000000)
                            day = int(nmea_parts[2])
                            month = int(nmea_parts[3])
                            year = int(nmea_parts[4])
                            zda = datetime.datetime(year,month,day,hour,minute,second,ms)
                            tref.time_ref = rospy.Time(calendar.timegm(zda.timetuple()),zda.microsecond*1000)
                            tref.source = 'posmv'
                            timeref_pub.publish(tref)
                        if nmea_parts[0][3:] == 'GGA' and len(nmea_parts) >= 10:
                            latitude = int(nmea_parts[2][0:2])+float(nmea_parts[2][2:])/60.0
                            if nmea_parts[3] == 'S':
                                latitude = -latitude
                            longitude = int(nmea_parts[4][0:3])+float(nmea_parts[4][3:])/60.0
                            if nmea_parts[5] == 'W':
                                longitude = -longitude
                            altitude = float(nmea_parts[9])
                            nsf = NavSatFix()
                            nsf.header.stamp = now
                            nsf.header.frame_id = 'posmv_operator'
                            nsf.latitude = latitude
                            nsf.longitude = longitude
                            nsf.altitude = altitude
                            position_pub.publish(nsf)
                        if nmea_parts[0][3:] == 'HDT' and len(nmea_parts) >= 2:
                            heading = float(nmea_parts[1])
                            nes = NavEulerStamped()
                            nes.header.stamp = now
                            nes.header.frame_id = 'posmv_operator'
                            nes.orientation.heading = heading
                            orientation_pub.publish(nes)
                    except ValueError:
                        pass
            except UnicodeDecodeError:
                pass


if __name__ == '__main__':
    try:
        posmv_nmea_listener()
    except rospy.ROSInterruptException:
        pass


