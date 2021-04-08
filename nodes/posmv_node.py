#!/usr/bin/env python

from __future__ import division
from past.utils import old_div
from posmv import posmv
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from sensor_msgs.msg import TimeReference
from geometry_msgs.msg import TwistWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
import datetime
import calendar
import tf.transformations
import math

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
  t2_type = old_div(d['time_types'],0x0f)
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
  rospy.init_node('posmv')
    
  listen_address = rospy.get_param('~listen_address', '')
  listen_port = rospy.get_param('~listen_port', 5602)
    
  posmv_frame = rospy.get_param('~posmv_frame', 'posmv')
    
  position_pub = rospy.Publisher('posmv/position',NavSatFix,queue_size=10)
  timeref_pub = rospy.Publisher('posmv/time_reference',TimeReference,queue_size=10)
  orientation_pub = rospy.Publisher('posmv/orientation',Imu,queue_size=10)
  velocity_pub = rospy.Publisher('posmv/velocity',TwistWithCovarianceStamped,queue_size=10)
  diagnostic_pub = rospy.Publisher('/diagnostics',DiagnosticArray,queue_size=10)
    
  pos = posmv.Posmv()

  gps_week = None
  gps_utc_offset = None
    
  group_2 = None

  last_data_time = None
  last_group_1_diag = None
  last_group_1_diag_time = None
  last_group_2_diag = None
  last_group_2_diag_time = None
  last_diag_time = None



  while not rospy.is_shutdown():
    data = pos.read((1,2,3))
    now = rospy.get_rostime()
    if len(data):
        last_data_time = now
    #print (data)
    for d in data:
        if d['group_id'] == 1:
            pos_now = decode_time(d, gps_week, gps_utc_offset)
            if pos_now is not None:
                tref = TimeReference()
                tref.header.stamp = now
                tref.time_ref = rospy.Time(calendar.timegm(pos_now.timetuple()),pos_now.microsecond*1000)
                tref.source = posmv_frame
                timeref_pub.publish(tref)
                
            nsf = NavSatFix()
            nsf.header.stamp = now
            nsf.header.frame_id = posmv_frame
            nsf.latitude = d['latitude']
            nsf.longitude = d['longitude']
            nsf.altitude = d['altitude']
            if group_2 is not None:
              nsf.position_covariance[0] = group_2['east_position_rms_error']**2
              nsf.position_covariance[4] = group_2['north_position_rms_error']**2
              nsf.position_covariance[8] = group_2['down_position_rms_error']**2
              nsf.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            position_pub.publish(nsf)

            imu = Imu()                  
            imu.header.stamp = now
            imu.header.frame_id = posmv_frame
            q = tf.transformations.quaternion_from_euler(math.radians(90.0-d['vessel_heading']), math.radians(-d['vessel_pitch']), math.radians(d['vessel_roll']), 'rzyx')
            
            imu.orientation.x = q[0]
            imu.orientation.y = q[1]
            imu.orientation.z = q[2]
            imu.orientation.w = q[3]
            if group_2 is not None:
              imu.orientation_covariance[0] = math.radians(group_2['roll_rms_error']**2)
              imu.orientation_covariance[4] = math.radians(group_2['pitch_rms_error']**2)
              imu.orientation_covariance[8] = math.radians(group_2['heading_rms_error']**2)
            imu.angular_velocity.x = math.radians(d['vessel_angular_rate_about_longitudinal_axis'])
            imu.angular_velocity.y = math.radians(-d['vessel_angular_rate_about_transverse_axis'])
            imu.angular_velocity.z = math.radians(-d['vessel_angular_rate_about_down_axis'])
            imu.linear_acceleration.x = d['vessel_longitudinal_acceleration']
            imu.linear_acceleration.y = d['vessel_transverse_acceleration']
            imu.linear_acceleration.z = -d['vessel_down_acceleration']
            
            #todo: measure and calculate covariance
            # From Val on teams:
            # This is a bit of a guess, but try (4 \sigma^2) / dt^2, where sigma^2 is the variance of the angle (rms^2) and dt is the time interval.
            # This is based on a commonly used "discrete noise process model" used to model the uncertainty for the model part of a Kalman Filter. I think it should be a reasonable estimate. (ref: Estimation with Applications to Tracking and Navigation")￼
            # It might be good to annotate that in a comment. Section 3.3.3, page 274. Authors are Bar-Shalom, Li and Kirubarajan

            orientation_pub.publish(imu)

            twcs = TwistWithCovarianceStamped()
            twcs.header.stamp = now
            twcs.header.frame_id = posmv_frame
            twcs.twist.twist.linear.x = d['east_velocity']
            twcs.twist.twist.linear.y = d['north_velocity']
            twcs.twist.twist.linear.z = -d['down_velocity']
            if group_2 is not None:
              twcs.twist.covariance[0] = group_2['east_velocity_rms_error']**2
              twcs.twist.covariance[7] = group_2['north_velocity_rms_error']**2
              twcs.twist.covariance[14] = group_2['down_velocity_rms_error']**2
            twcs.twist.twist.angular = imu.angular_velocity
            velocity_pub.publish(twcs)

            ds = DiagnosticStatus()
            ds.name = 'posmv/alignment_status'
            ds.hardware_id = str(pos.address)
            ds.values.append(KeyValue('code',str(d['alignment_status'])))

            status_table = (('Full navigation','User accuracies are met'),
                            ('Fine alignment is active','RMS heading error is less than 15 degrees'),
                            ('GC CHI 2','alignment with GPS, RMS heading error is greater than 15 degrees'),
                            ('PC CHI 2','alignment without GPS, RMS heading error is greater than 15 degrees'),
                            ('GC CHI 1','alignment with GPS, RMS heading error is greater than 45 degrees'),
                            ('PC CHI 1','alignment without GPS, RMS heading error is greater than 45 degrees'),
                            ('Coarse leveling is active',''),
                            ('Initial solution assigned',''),
                            ('No valid solution','')
            )

            if d['alignment_status'] >= 0 and d['alignment_status'] < len(status_table):
              ds.message = status_table[d['alignment_status']][0]
              ds.values.append(KeyValue('description', status_table[d['alignment_status']][0]))
              ds.values.append(KeyValue('note',status_table[d['alignment_status']][1]))
            else:
              ds.message = "Invalid alignment status"

            if d['alignment_status'] == 0:
              ds.level = DiagnosticStatus.OK
            elif d['alignment_status'] > 0 and d['alignment_status'] <= 3:
              ds.level = DiagnosticStatus.WARN
            elif d['alignment_status'] > 3:
              ds.level = DiagnosticStatus.ERROR

            last_group_1_diag = ds
            last_group_1_diag_time = now
                
        if d['group_id'] == 2:                  
            group_2 = d
            ds = DiagnosticStatus()
            ds.name = 'posmv/performance'
            ds.message = 'Vessel Navigation Performance Metrics'
            ds.hardware_id = str(pos.address)
            for k in ('north_position_rms_error', 'east_position_rms_error', 'down_position_rms_error', 'north_velocity_rms_error', 'east_velocity_rms_error', 'down_velocity_rms_error', 'roll_rms_error', 'pitch_rms_error',  'heading_rms_error', 'error_ellipsoid_semi-major', 'error_ellipsoid_semi-minor', 'error_ellipsoid_orientation'):
              ds.values.append(KeyValue(k,str(d[k])))
            last_group_2_diag = ds
            last_group_2_diag_time = now

        if d['group_id'] == 3:
            gps_week = d['gps_utc_week_number']
            gps_utc_offset = d['gps_utc_time_offset']
            #print 'gps week:', gps_week, 'offset:', gps_utc_offset, 'time types:', d['time_types']
            #print decode_time(d,gps_week,gps_utc_offset)

    if last_diag_time is None or now - last_diag_time > rospy.Duration(1.0):
      last_diag_time = now
      diag_array = DiagnosticArray()
      diag_array.header.stamp = rospy.Time.now()

      ds = DiagnosticStatus()
      ds.name = 'posmv/network'
      if last_data_time is None:
        ds.level = DiagnosticStatus.ERROR
        ds.message = 'Network Timeout'
        ds.values.append(KeyValue('time_since_last_data','never'))
      else:
        age = now - last_data_time
        if age > rospy.Duration(5):
          ds.level = DiagnosticStatus.ERROR
          ds.message = 'Network Timeout'
        elif age > rospy.Duration(1):
          ds.level = DiagnosticStatus.WARN
          ds.message = 'Network Lagging'

        else:
          ds.level = DiagnosticStatus.OK
          ds.message = 'Network OK'

        ds.values.append(KeyValue('time_since_last_data',str(age.to_sec())))
      diag_array.status.append(ds)

      if last_group_1_diag is not None and now - last_group_1_diag_time < rospy.Duration(10.0):
        diag_array.status.append(last_group_1_diag)
      if last_group_2_diag is not None and now - last_group_2_diag_time < rospy.Duration(10.0):
        diag_array.status.append(last_group_2_diag)
      diagnostic_pub.publish(diag_array)

if __name__ == '__main__':
    try:
        posmv_listener()
    except rospy.ROSInterruptException:
        pass

