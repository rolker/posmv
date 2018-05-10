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
        data,address = self.sock.recvfrom(4096)
        print address
        self.data_buffer += data
        ret = []
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

                if grp['group_id'] == 4:
                    if len(self.data_buffer) > 34+29:
                        grp['imu_data'] = self.data_buffer[34:34+29]
                    #print 'skipping 4...'
                    
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
        