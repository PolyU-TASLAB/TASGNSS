#!/usr/bin/env python3

import rospy
import sys
import os
import time
from ros_wrapper.msg import GnssObservation, GnssEphemMsg, GnssProcessedData
import tasgnss
import pyrtklib as prl

class RosWrapperNode:
    def __init__(self):
        rospy.init_node('ros_wrapper_node', anonymous=True)
        
        # Publishers
        self.obs_pub = rospy.Publisher('gnss_observations', GnssObservation, queue_size=10)
        self.ephem_pub = rospy.Publisher('gnss_ephemeris', GnssEphemMsg, queue_size=10)
        self.processed_data_pub = rospy.Publisher('gnss_processed_data', GnssProcessedData, queue_size=10)
        
        # Initialize tasgnss
        self.nav = None  # Will be set when reading data
        
        rospy.loginfo("ROS Wrapper Node initialized")

    def process_and_publish(self, obs_file, nav_file, start_utc=0, end_utc=0):
        """
        Process GNSS data and publish ROS messages
        """
        try:
            # Read observation and navigation data
            obs, nav, sta = tasgnss.read_obs(obs_file, nav_file)
            self.nav = nav
            
            
            # Split observations by epoch
            obss = tasgnss.split_obs(obs,ref_obs=False)
            if start_utc > 0 and end_utc > 0:
                rospy.loginfo(f"Filtering observations from UTC {start_utc} to {end_utc}")
                obss = tasgnss.filter_obs(obss, start_utc, end_utc)

            rate = rospy.Rate(1)  # 1 Hz
            
            for i, o in enumerate(obss):
                if rospy.is_shutdown():
                    break
                    
                # Process observations for this epoch
                result = tasgnss.preprocess_obs(o, nav, use_cache=False)
                
                # Check if result is valid (array with at least 7 elements)
                if isinstance(result, list) and len(result) >= 7:
                    # Extract data from the result array
                    # ret_data = [None,None,None,None,data,cdata,raw_data]
                    data = result[4]  # data
                    cdata = result[5]  # cdata (solve_data)
                    raw_data = result[6]  # raw_data
                    
                    # Check if data is not empty
                    if len(data) > 0:
                        # Publish observation messages
                        self.publish_observations(data, raw_data, o.data[0].time)
                        
                        # Publish ephemeris messages (for satellites in this epoch)
                        self.publish_ephemeris(data, o.data[0].time)
                        
                        # Publish processed data messages
                        self.publish_processed_data(cdata, o.data[0].time)
                
                rate.sleep()
                
        except Exception as e:
            rospy.logerr(f"Error processing data: {str(e)}")

    def publish_observations(self, data, raw_data, time):
        """
        Publish GNSS observation messages
        """
        for i, obs_data in enumerate(data):
            msg = GnssObservation()
            
            # Extract data from the processed observation
            sat, sname, s_sys, satpos, sdt, corrected_p, sagnac, iono_error, trop_error, snr, var, az, el, dop = obs_data
            
            # Fill message fields
            # Handle sname which might be bytes
            if isinstance(sname, bytes):
                msg.prn = sname.decode('utf-8')
            else:
                msg.prn = sname
                
            msg.week = time.time // 604800  # Approximate GPS week
            msg.tow = time.time % 604800    # Approximate GPS TOW
            
            # Signal strengths (SNR) - use raw_data
            snr_values = raw_data['SNR']
            msg.SNR = [float(s) for s in snr_values if s > 0]
            
            # Loss of lock indicators (LLI) - use raw_data
            lli_values = raw_data['LLI']
            msg.LLI = [int(l) for l in lli_values]
            
            # Code indicators - use raw_data
            code_values = raw_data['code']
            # Convert code values to string representation based on RTKLIB definitions
            code_map = {
                0: "NONE",   # obs code: none or unknown
                1: "L1C",    # obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS)
                2: "L1P",    # obs code: L1P,G1P,B1P (GPS,GLO,BDS)
                3: "L1W",    # obs code: L1 Z-track (GPS)
                4: "L1Y",    # obs code: L1Y        (GPS)
                5: "L1M",    # obs code: L1M        (GPS)
                6: "L1N",    # obs code: L1codeless,B1codeless (GPS,BDS)
                7: "L1S",    # obs code: L1C(D)     (GPS,QZS)
                8: "L1L",    # obs code: L1C(P)     (GPS,QZS)
                9: "L1E",    # (not used)
                10: "L1A",   # obs code: E1A,B1A    (GAL,BDS)
                11: "L1B",   # obs code: E1B        (GAL)
                12: "L1X",   # obs code: E1B+C,L1C(D+P),B1D+P (GAL,QZS,BDS)
                13: "L1Z",   # obs code: E1A+B+C,L1S (GAL,QZS)
                14: "L2C",   # obs code: L2C/A,G1C/A (GPS,GLO)
                15: "L2D",   # obs code: L2 L1C/A-(P2-P1) (GPS)
                16: "L2S",   # obs code: L2C(M)     (GPS,QZS)
                17: "L2L",   # obs code: L2C(L)     (GPS,QZS)
                18: "L2X",   # obs code: L2C(M+L),B1_2I+Q (GPS,QZS,BDS)
                19: "L2P",   # obs code: L2P,G2P    (GPS,GLO)
                20: "L2W",   # obs code: L2 Z-track (GPS)
                21: "L2Y",   # obs code: L2Y        (GPS)
                22: "L2M",   # obs code: L2M        (GPS)
                23: "L2N",   # obs code: L2codeless (GPS)
                24: "L5I",   # obs code: L5I,E5aI   (GPS,GAL,QZS,SBS)
                25: "L5Q",   # obs code: L5Q,E5aQ   (GPS,GAL,QZS,SBS)
                26: "L5X",   # obs code: L5I+Q,E5aI+Q,L5B+C,B2aD+P (GPS,GAL,QZS,IRN,SBS,BDS)
                27: "L7I",   # obs code: E5bI,B2bI  (GAL,BDS)
                28: "L7Q",   # obs code: E5bQ,B2bQ  (GAL,BDS)
                29: "L7X",   # obs code: E5bI+Q,B2bI+Q (GAL,BDS)
                30: "L6A",   # obs code: E6A,B3A    (GAL,BDS)
                31: "L6B",   # obs code: E6B        (GAL)
                32: "L6C",   # obs code: E6C        (GAL)
                33: "L6X",   # obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,BDS)
                34: "L6Z",   # obs code: E6A+B+C,L6D+E (GAL,QZS)
                35: "L6S",   # obs code: L6S        (QZS)
                36: "L6L",   # obs code: L6L        (QZS)
                37: "L8I",   # obs code: E5abI      (GAL)
                38: "L8Q",   # obs code: E5abQ      (GAL)
                39: "L8X",   # obs code: E5abI+Q,B2abD+P (GAL,BDS)
                40: "L2I",   # obs code: B1_2I      (BDS)
                41: "L2Q",   # obs code: B1_2Q      (BDS)
                42: "L6I",   # obs code: B3I        (BDS)
                43: "L6Q",   # obs code: B3Q        (BDS)
                44: "L3I",   # obs code: G3I        (GLO)
                45: "L3Q",   # obs code: G3Q        (GLO)
                46: "L3X",   # obs code: G3I+Q      (GLO)
                47: "L1I",   # obs code: B1I        (BDS) (obsolute)
                48: "L1Q",   # obs code: B1Q        (BDS) (obsolute)
                49: "L5A",   # obs code: L5A SPS    (IRN)
                50: "L5B",   # obs code: L5B RS(D)  (IRN)
                51: "L5C",   # obs code: L5C RS(P)  (IRN)
                52: "L9A",   # obs code: SA SPS     (IRN)
                53: "L9B",   # obs code: SB RS(D)   (IRN)
                54: "L9C",   # obs code: SC RS(P)   (IRN)
                55: "L9X",   # obs code: SB+C       (IRN)
                56: "L1D",   # obs code: B1D        (BDS)
                57: "L5D",   # obs code: L5D(L5S),B2aD (QZS,BDS)
                58: "L5P",   # obs code: L5P(L5S),B2aP (QZS,BDS)
                59: "L5Z",   # obs code: L5D+P(L5S) (QZS)
                60: "L6E",   # obs code: L6E        (QZS)
                61: "L7D",   # obs code: B2bD       (BDS)
                62: "L7P",   # obs code: B2bP       (BDS)
                63: "L7Z",   # obs code: B2bD+P     (BDS)
                64: "L8D",   # obs code: B2abD      (BDS)
                65: "L8P",   # obs code: B2abP      (BDS)
                66: "L4A",   # obs code: G1aL1OCd   (GLO)
                67: "L4B",   # obs code: G1aL1OCd   (GLO)
                68: "L4X"    # obs code: G1al1OCd+p (GLO)
            }
            msg.code = [code_map.get(int(c), str(int(c))) for c in code_values if int(c) in code_map]
            
            # Carrier phase cycles - use raw_data
            l_values = raw_data['L']
            msg.L = [float(l) for l in l_values if l != 0.0]
            
            # Pseudoranges - use processed data
            msg.P = [float(corrected_p)]
            
            # Dopplers - use processed data
            msg.D = [float(dop)]
            
            # Publish message
            self.obs_pub.publish(msg)
            rospy.loginfo(f"Published observation for satellite {msg.prn}")

    def publish_ephemeris(self, data, time):
        """
        Publish GNSS ephemeris messages
        """
        for i, obs_data in enumerate(data):
            sat, sname, s_sys, satpos, sdt, corrected_p, sagnac, iono_error, trop_error, snr, var, az, el, dop = obs_data
            
            # Find ephemeris data for this satellite
            eph = None
            for j in range(self.nav.n):
                if self.nav.eph[j].sat == sat:
                    eph = self.nav.eph[j]
                    break
            
            if eph is not None:
                msg = GnssEphemMsg()
                
                # Fill message fields
                msg.sat = sat
                # Note: GnssTimeMsg is not defined in the provided message definition
                # We'll need to create a separate message for time or modify the existing one
                # For now, we'll skip ttr, toe, toc fields or use placeholder values
                msg.toe_tow = eph.toes
                msg.week = eph.week
                msg.iode = eph.iode
                msg.iodc = eph.iodc
                msg.health = eph.svh
                msg.code = eph.code
                msg.ura = eph.sva
                msg.A = eph.A
                msg.e = eph.e
                msg.i0 = eph.i0
                msg.omg = eph.omg
                msg.OMG0 = eph.OMG0
                msg.M0 = eph.M0
                msg.delta_n = eph.deln
                msg.OMG_dot = eph.OMGd
                msg.i_dot = eph.idot
                msg.cuc = eph.cuc
                msg.cus = eph.cus
                msg.crc = eph.crc
                msg.crs = eph.crs
                msg.cic = eph.cic
                msg.cis = eph.cis
                msg.af0 = eph.f0
                msg.af1 = eph.f1
                msg.af2 = eph.f2
                msg.tgd0 = eph.tgd[0] if len(eph.tgd) > 0 else 0.0
                msg.tgd1 = eph.tgd[1] if len(eph.tgd) > 1 else 0.0
                msg.A_dot = eph.Adot if hasattr(eph, 'Adot') else 0.0
                msg.n_dot = eph.ndot if hasattr(eph, 'ndot') else 0.0
                
                # Publish message
                self.ephem_pub.publish(msg)
                rospy.loginfo(f"Published ephemeris for satellite {sname}")

    def publish_processed_data(self, cdata, time):
        """
        Publish processed GNSS data
        """
        msg = GnssProcessedData()
        
        # Extract PRN names from cdata (assuming they are available in the original data)
        # Since cdata doesn't contain PRN names directly, we'll need to get them from the context
        # For now, we'll use empty strings as placeholders
        msg.prn = cdata['sys']
        
        # Flatten satpos array (Nx3) to 1D array
        if 'satpos' in cdata and len(cdata['satpos']) > 0:
            msg.satpos = cdata['satpos'].flatten().tolist()
        else:
            msg.satpos = []
            
        # Flatten and convert other arrays
        if 'pr' in cdata and len(cdata['pr']) > 0:
            msg.pr = cdata['pr'].flatten().tolist()
        else:
            msg.pr = []
            
        if 'dop' in cdata and len(cdata['dop']) > 0:
            msg.dop = cdata['dop'].flatten().tolist()
        else:
            msg.dop = []
            
        if 'sdt' in cdata and len(cdata['sdt']) > 0:
            msg.sdt = cdata['sdt'].flatten().tolist()
        else:
            msg.sdt = []
            
        if 'sagnac' in cdata and len(cdata['sagnac']) > 0:
            msg.sagnac = cdata['sagnac'].tolist()
        else:
            msg.sagnac = []
            
        if 'I' in cdata and len(cdata['I']) > 0:
            msg.I = cdata['I'].tolist()
        else:
            msg.I = []
            
        if 'T' in cdata and len(cdata['T']) > 0:
            msg.T = cdata['T'].tolist()
        else:
            msg.T = []
            
        if 'sys' in cdata:
            msg.sys = cdata['sys']
        else:
            msg.sys = []
            
        if 'var' in cdata and len(cdata['var']) > 0:
            msg.var = cdata['var'].tolist()
        else:
            msg.var = []
        
        # Publish message
        self.processed_data_pub.publish(msg)
        rospy.loginfo(f"Published processed data for {len(msg.prn)} satellites")

if __name__ == '__main__':
    try:
        node = RosWrapperNode()
        
        # Get file paths from parameters or use defaults
        obs_file = rospy.get_param('~obs_file', 'data/20210610/COM38_210610_025603.obs')
        nav_file = rospy.get_param('~nav_file', 'data/20210610/sta/hksc161d.21*')
        
        # Get UTC time range parameters
        start_utc = rospy.get_param('~start_utc', 0)
        end_utc = rospy.get_param('~end_utc', 0)
        
        rospy.loginfo(f"Processing observation file: {obs_file}")
        rospy.loginfo(f"Processing navigation file: {nav_file}")
        rospy.loginfo(f"UTC time range: {start_utc} to {end_utc}")
        
        node.process_and_publish(obs_file, nav_file, start_utc, end_utc)
        
    except rospy.ROSInterruptException:
        pass