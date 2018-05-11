#!/usr/bin/env python

import posmv
import rospy
from  sensor_msgs.msg import NavSatFix

#
# from: https://gist.github.com/jeremiahajohnson/eca97484db88bcf6b124
#
def weeksecondstoutc(gpsweek,gpsseconds,leapseconds):
    import datetime, calendar
    datetimeformat = "%Y-%m-%d %H:%M:%S"
    epoch = datetime.datetime.strptime("1980-01-06 00:00:00",datetimeformat)
    elapsed = datetime.timedelta(days=(gpsweek*7),seconds=(gpsseconds+leapseconds))
    return datetime.datetime.strftime(epoch + elapsed,datetimeformat)

def posmv_listener():
    pub = rospy.Publisher('/posmv/position',NavSatFix,queue_size=10)
    rospy.init_node('posmv')
    
    pos = posmv.Posmv()

    gps_week = None
    gps_utc_offset = None
    
    while not rospy.is_shutdown():
        data = pos.read((1,3))
        #print data
        for d in data:
           if d['group_id'] == 1:
              nsf = NavSatFix()
              nsf.latitude = d['latitude']
              nsf.longitude = d['longitude']
              nsf.altitude = d['altitude']
              pub.publish(nsf)
           if d['group_id'] == 3:
              gps_week = d['gps_utc_week_number']
              gps_utc_offset = d['gps_utc_time_offset']
              print 'gps week:', gps_week, 'offset:', gps_utc_offset

if __name__ == '__main__':
    try:
        posmv_listener()
    except rospy.ROSInterruptException:
        pass

