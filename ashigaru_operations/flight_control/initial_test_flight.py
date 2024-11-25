#!/usr/bin/env python3

import rospy
import mavros
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from geometry_msgs.msg import PoseStamped
import time
import os
import subprocess


class DroneTest:
    def __init__(self):
        rospy.init_node('drone_test_node')

        self.current_state = State()

        # Warte auf die Services
        rospy.loginfo("Warte auf Services...")
        try:
            rospy.wait_for_service("/mavros/cmd/arming", timeout=10)
            rospy.wait_for_service("/mavros/set_mode", timeout=10)
        except rospy.ROSException:
            rospy.logerr("Services nicht verfügbar!")
            return

        # Starte Logger
        try:
            script_dir = os.path.dirname(os.path.realpath(__file__))
            log_script = os.path.join(script_dir, 'telemetrie_logging.py')
            self.logger_process = subprocess.Popen(['python3', log_script])
            rospy.loginfo("Telemetrie-Logger gestartet")
        except Exception as e:
            rospy.logerr(f"Fehler beim Starten des Loggers: {e}")

        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.pose = PoseStamped()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 2

    def cleanup(self):
        """Aufräumen beim Beenden"""
        if hasattr(self, 'logger_process'):
            self.logger_process.terminate()
            rospy.loginfo("Telemetrie-Logger beendet")

    def state_cb(self, msg):
        self.current_state = msg

    def wait_for_connection(self):
        rospy.loginfo("Warte auf MAVROS Verbindung...")
        for i in range(50):  # Timeout nach 10 Sekunden
            if self.current_state.connected:
                rospy.loginfo("MAVROS verbunden!")
                return True
            rospy.sleep(0.2)
        rospy.logerr("Keine Verbindung zu MAVROS!")
        return False

    def arm_drone(self):
        rospy.loginfo("Versuche Drohne zu armen...")
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        for i in range(5):  # 5 Versuche
            if self.arming_client.call(arm_cmd).success:
                rospy.loginfo("Drohne erfolgreich gearmt!")
                return True
            rospy.sleep(1)
        rospy.logerr("Arming fehlgeschlagen!")
        return False

    def set_offboard_mode(self):
        rospy.loginfo("Wechsle in OFFBOARD Mode...")
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        # Sende einige Setpoints bevor Switch to OFFBOARD
        for i in range(50):
            self.local_pos_pub.publish(self.pose)
            rospy.sleep(0.1)

        for i in range(5):  # 5 Versuche
            if self.set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD Mode aktiviert!")
                return True
            rospy.sleep(1)
        rospy.logerr("OFFBOARD Mode Aktivierung fehlgeschlagen!")
        return False

    def test_flight(self):
        try:
            if not self.wait_for_connection():
                return

            if not self.set_offboard_mode():
                return

            if not self.arm_drone():
                return

            rospy.loginfo("Starte Testflug...")

            # Steige auf
            rospy.loginfo("Steige auf...")
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time) < rospy.Duration(5.0):
                self.local_pos_pub.publish(self.pose)
                rospy.sleep(0.1)

            # Fliege Quadrat
            waypoints = [
                (2, 0, 2),
                (2, 2, 2),
                (0, 2, 2),
                (0, 0, 2)
            ]

            for wp in waypoints:
                rospy.loginfo(f"Fliege zu Position: {wp}")
                self.pose.pose.position.x = wp[0]
                self.pose.pose.position.y = wp[1]
                self.pose.pose.position.z = wp[2]

                start_time = rospy.Time.now()
                while (rospy.Time.now() - start_time) < rospy.Duration(5.0):
                    self.local_pos_pub.publish(self.pose)
                    rospy.sleep(0.1)

            # Landung
            rospy.loginfo("Kehre zur Landung zurück")
            self.pose.pose.position.x = 0
            self.pose.pose.position.y = 0
            self.pose.pose.position.z = 0

            for i in range(50):
                self.local_pos_pub.publish(self.pose)
                rospy.sleep(0.1)

            land_set_mode = SetModeRequest()
            land_set_mode.custom_mode = 'AUTO.LAND'
            if self.set_mode_client.call(land_set_mode).mode_sent:
                rospy.loginfo("Landung eingeleitet")
        finally:
            self.cleanup()


if __name__ == "__main__":
    try:
        drone = DroneTest()
        drone.test_flight()
    except rospy.ROSInterruptException:
        if hasattr(drone, 'cleanup'):
            drone.cleanup()
