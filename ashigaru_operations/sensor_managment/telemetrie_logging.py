#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, NavSatFix, Imu
from mavros_msgs.msg import State, BatteryStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import json
import csv
from datetime import datetime
import os

class TelemetryLogger:
    def __init__(self):
        rospy.init_node('telemetry_logger')

        # Erstelle Aufnahmeordner
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_dir = os.path.join(os.path.expanduser('~'),
                                   'drone_ai_project/recordings', timestamp)
        os.makedirs(self.log_dir, exist_ok=True)

        # Öffne Log-Dateien
        self.flight_data = open(os.path.join(self.log_dir, 'flight_data.csv'), 'w')
        self.flight_csv = csv.writer(self.flight_data)
        self.flight_csv.writerow(['timestamp', 'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                                'vx', 'vy', 'vz', 'battery', 'mode'])

        # Subscriber für verschiedene Telemetriedaten
        self.pose_sub = rospy.Subscriber('mavros/local_position/pose',
                                        PoseStamped, self.pose_callback)
        self.vel_sub = rospy.Subscriber('mavros/local_position/velocity_local',
                                       TwistStamped, self.velocity_callback)
        self.state_sub = rospy.Subscriber('mavros/state',
                                         State, self.state_callback)
        self.battery_sub = rospy.Subscriber('mavros/battery',
                                          BatteryStatus, self.battery_callback)
        self.gps_sub = rospy.Subscriber('mavros/global_position/global',
                                       NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber('mavros/imu/data',
                                       Imu, self.imu_callback)

        # Aktuelle Werte
        self.current_pose = None
        self.current_vel = None
        self.current_state = None
        self.current_battery = None
        self.current_gps = None
        self.current_imu = None

    def pose_callback(self, msg):
        self.current_pose = msg
        self.write_telemetry()

    def velocity_callback(self, msg):
        self.current_vel = msg

    def state_callback(self, msg):
        self.current_state = msg

    def battery_callback(self, msg):
        self.current_battery = msg

    def gps_callback(self, msg):
        self.current_gps = msg

    def imu_callback(self, msg):
        self.current_imu = msg

    def write_telemetry(self):
        if all([self.current_pose, self.current_vel,
                self.current_state, self.current_battery]):
            # Schreibe alle Daten in CSV
            self.flight_csv.writerow([
                rospy.Time.now(),
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z,
                # Hier würden noch die Orientierungswerte aus den Quaternionen kommen
                self.current_vel.twist.linear.x,
                self.current_vel.twist.linear.y,
                self.current_vel.twist.linear.z,
                self.current_battery.percentage,
                self.current_state.mode
            ])
            self.flight_data.flush()  # Sicheres Schreiben

    def shutdown(self):
        """Cleanup beim Beenden"""
        self.flight_data.close()
        rospy.loginfo("Telemetrie-Logger beendet")

def main():
    try:
        logger = TelemetryLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'logger' in locals():
            logger.shutdown()

if __name__ == "__main__":
    main()
