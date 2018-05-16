#!/usr/bin/env python

import posmv
import rospy
import rosbag
from  sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import TimeReference
from marine_msgs.msg import NavEulerStamped
import datetime
import calendar

#
# from: https://gist.github.com/jeremiahajohnson/eca97484db88bcf6b124
#
def weeksecondstoutc(gpsweek,gpsseconds,leapseconds):
    datetimeformat = "%Y-%m-%d %H:%M:%S"
    epoch = datetime.datetime.strptime("1980-01-06 00:00:00",datetimeformat)
    elapsed = datetime.timedelta(days=(gpsweek*7),seconds=(gpsseconds+leapseconds))
    return epoch + elapsed
    #return datetime.datetime.strftime(epoch + elapsed,datetimeformat)

def decode_time(d, gps_week, offset):
    if gps_week is None:
        return None
    t1_type = d['time_types']&0x0f
    t2_type = d['time_types']/0x0f
    if t1_type == 2:
        return weeksecondstoutc(gps_week,d['time1'],0)
    elif t1_type == 1:
        return weeksecondstoutc(gps_week,d['time1'],offset)
    elif t2_type == 2:
        return weeksecondstoutc(gps_week,d['time2'],0)
    elif t2_type == 1:
        return weeksecondstoutc(gps_weel,d['time2'],offset)
    return None


def posmv_listener():
    position_pub = rospy.Publisher('/posmv/position',NavSatFix,queue_size=10)
    timeref_pub = rospy.Publisher('/posmv/time_reference',TimeReference,queue_size=10)
    orientation_pub = rospy.Publisher('/posmv/orientation',NavEulerStamped,queue_size=10)
    rospy.init_node('posmv')
    
    pos = posmv.Posmv()

    gps_week = None
    gps_utc_offset = None

    timestamp = datetime.datetime.utcfromtimestamp(rospy.Time.now().to_time()).isoformat()
    bag = rosbag.Bag('nodes/posmv_'+('-'.join(timestamp.split(':')))+'.bag', 'w', rosbag.Compression.BZ2)
    while not rospy.is_shutdown():
        data = pos.read((1,3))
        #print data
        for d in data:
           if d['group_id'] == 1:
              now = rospy.get_rostime()
              pos_now = decode_time(d, gps_week, gps_utc_offset)
              if pos_now is not None:
                  tref = TimeReference()
                  tref.header.stamp = now
                  tref.time_ref = rospy.Time(calendar.timegm(pos_now.timetuple()),pos_now.microsecond*1000)
                  tref.source = 'posmv'
                  timeref_pub.publish(tref)
                  bag.write('/posmv/time_reference', tref)
                  nsf = NavSatFix()
                  nsf.header.stamp = now
                  nsf.header.frame_id = 'posmv'
                  nsf.latitude = d['latitude']
                  nsf.longitude = d['longitude']
                  nsf.altitude = d['altitude']
                  position_pub.publish(nsf)
                  bag.write('/posmv/position', nsf)
                  nes = NavEulerStamped()
                  nes.header.stamp = now
                  nes.header.frame_id = 'posmv'
                  nes.orientation.roll = d['vessel_roll']
                  nes.orientation.pitch = d['vessel_pitch']
                  nes.orientation.heading = d['vessel_heading']
                  orientation_pub.publish(nes)
                  bag.write('/posmv/orientation', nes)
           if d['group_id'] == 3:
              gps_week = d['gps_utc_week_number']
              gps_utc_offset = d['gps_utc_time_offset']
              #print 'gps week:', gps_week, 'offset:', gps_utc_offset, 'time types:', d['time_types']
              #print decode_time(d,gps_week,gps_utc_offset)

if __name__ == '__main__':
    try:
        posmv_listener()
    except rospy.ROSInterruptException:
        pass

