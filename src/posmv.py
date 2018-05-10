#!/usr/bin/env python

import socket
import struct

# messages configured on CW4: 1,2,3,4,5,7,9,10,99,102,110-113,10001,10007-10009,10011-10012

class Posmv:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('',5602))
        self.data_buffer = ''
        
    def read(self):
        ret = []
        data,address = self.sock.recvfrom(4096)
        print address
        self.data_buffer += data
        while '$GRP' in self.data_buffer:
            i = self.data_buffer.find('$GRP')
            if i < 0:
                break
            self.data_buffer = self.data_buffer[i:]
            if len(self.data_buffer) < 8:
                break
            grp = {}
            grp['group_id'],byte_count = struct.unpack('<HH',self.data_buffer[4:8])
            #if grp['group_id'] != 4:
            #    print 'group:',grp['group_id'],'size:',byte_count
                
            if len(self.data_buffer) < byte_count+8:
                break

            if byte_count >= 26:
                
                grp['time1'],grp['time2'],grp['distance_tag'],grp['time_types'],grp['distance_type']=struct.unpack('<dddBB',self.data_buffer[8:34])

                if grp['group_id'] == 1:
                    if len(self.data_buffer) > 34+101:
                        grp['latitude'],grp['longitude'],grp['altitude'],grp['north_velocity'],grp['east_velocity'],grp['down_velocity'],grp['vessel_roll'],grp['vessel_pitch'],grp['vessel_heading'],grp['vessel_wander_angle'],grp['vessel_track_angle'],grp['vessel_speed'],grp['vessel_angular_rate_about_longitudinal_axis'],grp['vessel_angular_rate_about_transverse_axis'],grp['vessel_angular_rate_about_down_axis'],grp['vessel_longitudinal_acceleration'],grp['vessel_transverse_acceleration'],grp['vessel_down_acceleration'],grp['alignment_status'] = struct.unpack('<dddfffddddffffffffB',self.data_buffer[34:34+101])
                elif grp['group_id'] == 2:
                    if len(self.data_buffer) > 34+48:
                        grp['north_position_rms_error'],grp['east_position_rms_error'],grp['down_position_rms_error'],grp['north_velocity_rms_error'],grp['east_velocity_rms_error'],grp['down_velocity_rms_error'],grp['roll_rms_error'],grp['pitch_rms_error'],grp['heading_rms_error'],grp['error_ellipsoid_semi-major'],grp['error_ellipsoid_semi-minor'],grp['error_ellipsoid_orientation'] = struct.unpack('<12f',self.data_buffer[34:34+48])
                elif grp['group_id'] == 3:
                    if len(self.data_buffer) > 34+4:
                        grp['navigation_solution_status'],grp['number_of_sv_tracked'],channel_status_byte_count = struct.unpack('<BBH',self.data_buffer[34:34+4])
                        if len(self.data_buffer) > 34+4+channel_status_byte_count+40:
                            grp['channel_status'] = []
                            for i in range(0,channel_status_byte_count,20):
                                sv = {}
                                sv['sv_prn'],sv['channel_tracking_status'],sv['sv_azimuth'],sv['sv_elevation'],sv['sv_l1_snr'],sv['sv_l2_snr'] = struct.unpack('<HHffff',self.data_buffer[34+4+i:34+4+i+20])
                                grp['channel_status'].append(sv)
                            grp['hdop'],grp['vdop'],grp['dgps_correction_latency'],grp['dgps_reference_id'],grp['gps_utc_week_number'],grp['gps_utc_time_offset'],grp['gnss_navigation_message_latency'],grp['geoidal_separation'],grp['gnss_receiver_type'],grp['gnss_status'] = struct.unpack('<fffHLdffHL', self.data_buffer[34+4+channel_status_byte_count:34+4+channel_status_byte_count+40])
                elif grp['group_id'] == 4:
                    if len(self.data_buffer) > 34+29:
                        grp['imu_data'] = self.data_buffer[34:34+29]
                    #print 'skipping 4...'
                elif grp['group_id'] in (5,6):
                    if len(self.data_buffer) > 34+4:
                        grp['event_pulse_number'], = struct.unpack('<L',self.data_buffer[34:34+4])
                elif grp['group_id'] == 7:
                    if len(self.data_buffer) > 34+5:
                        grp['pps_count'],grp['time_synchronization_status'] = struct.unpack('<LB',self.data_buffer[34:34+5])
                elif grp['group_id'] == 9:
                    if len(self.data_buffer) > 34+40:
                        grp['number_of_satellites'],grp['a_priori_pdop'],grp['computed_antenna_separation'],grp['solution_status'],grp['prn_assignement'],grp['cycle_slip_flag'],grp['gams_heading'],grp['gams_heading_rms_error'] = struct.unpack('<BffB12sHdd',self.data_buffer[34:34+40])
                elif grp['group_id'] == 10:
                    if len(self.data_buffer) > 34+30:
                        grp['general_status_a'],grp['general_status_b'],grp['general_status_c'],grp['fdir_level_1_status'],grp['fdir_level_1_imu_failures'],grp['fdir_level_2_status'],grp['fdir_level_3_status'],grp['fdir_level_4_status'],grp['fdir_level_5_status'],grp['extended_status'] = struct.unpack('<LLLLHHHHHL',self.data_buffer[34:34+30])
                elif grp['group_id'] == 99:
                    if len(self.data_buffer) > 34+380:
                        grp['system_version'],grp['primary_gps_version'],grp['secondary_gps_version'],grp['total_hours'],grp['number_of_runs'],grp['average_length_of_run'],grp['longest_run'],grp['current_run'],grp['options'] = struct.unpack('<120s80s80sfLfff80s',self.data_buffer[34:34+380])
                        for k in ('system_version','primary_gps_version','secondary_gps_version','options'):
                            grp[k] = grp[k].strip('\x00')
                elif grp['group_id'] == 10001:
                    if len(self.data_buffer) > 34+8:
                        grp['gps_receiver_type'],var_msg_byte_count = struct.unpack('<H4xH',self.data_buffer[34:34+8])
                        if len(self.data_buffer) > 34+8+var_msg_byte_count:
                            grp['gps_receiver_raw_data'] = self.data_buffer[34+8:34+8+var_msg_byte_count]
                    
                else:
                    print grp['group_id'],'not handled    !!!!'
                ret.append(grp)
            self.data_buffer = self.data_buffer[byte_count+8:]
            
        
        return ret
        
if __name__ == '__main__':
    p = Posmv()
    while True:
        for g in p.read():
            print g
        
