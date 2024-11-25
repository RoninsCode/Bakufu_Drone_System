#!/usr/bin/env python3
#im Verzeichnis: Bakufu_Drone_System/ashigaru_operations/flight_control

import rospy
import mavros
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from geometry_msgs.msg import PoseStamped
import time
import os
import subprocess

class AshigaruFlightController:
    """
    Ashigaru Flight Control System
    Hauptklasse für die grundlegende Flugsteuerung und Mission Control.
    """
    def __init__(self):
        rospy.init_node('ashigaru_flight_control')

        # State Management
        self.current_state = State()
        self._initialize_services()
        self._initialize_logger()
        self._setup_ros_interface()
        self._initialize_flight_parameters()

    def _initialize_services(self):
        """Initialisiert die benötigten ROS-Services"""
        try:
            rospy.loginfo("Initialisiere Ashigaru Services...")
            rospy.wait_for_service("/mavros/cmd/arming", timeout=10)
            rospy.wait_for_service("/mavros/set_mode", timeout=10)
            rospy.loginfo("Services erfolgreich initialisiert")
        except rospy.ROSException as e:
            rospy.logerr(f"Service-Initialisierung fehlgeschlagen: {e}")
            raise

    def _initialize_logger(self):
        """Startet das Telemetrie-Logging-System"""
        try:
            script_dir = os.path.dirname(os.path.realpath(__file__))
            log_script = os.path.join(script_dir, 'telemetrie_logging.py')
            self.logger_process = subprocess.Popen(['python3', log_script])
            rospy.loginfo("Telemetrie-System aktiviert")
        except Exception as e:
            rospy.logerr(f"Telemetrie-System-Fehler: {e}")
            raise

    def _setup_ros_interface(self):
        """Initialisiert die ROS-Kommunikationsschnittstellen"""
        self.state_sub = rospy.Subscriber(
            "mavros/state",
            State,
            self._state_callback
        )
        self.local_pos_pub = rospy.Publisher(
            "mavros/setpoint_position/local",
            PoseStamped,
            queue_size=10
        )
        self.arming_client = rospy.ServiceProxy(
            "mavros/cmd/arming",
            CommandBool
        )
        self.set_mode_client = rospy.ServiceProxy(
            "mavros/set_mode",
            SetMode
        )

    def _initialize_flight_parameters(self):
        """Initialisiert die Flugparameter"""
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 2

    def cleanup(self):
        """System-Cleanup und Ressourcenfreigabe"""
        if hasattr(self, 'logger_process'):
            self.logger_process.terminate()
            rospy.loginfo("Telemetrie-System deaktiviert")

    def _state_callback(self, msg):
        """Callback für Statusaktualisierungen"""
        self.current_state = msg

    def establish_connection(self):
        """
        Stellt die MAVROS-Verbindung her
        Returns:
            bool: True bei erfolgreicher Verbindung
        """
        rospy.loginfo("Initialisiere MAVROS-Verbindung...")
        for _ in range(50):
            if self.current_state.connected:
                rospy.loginfo("MAVROS-Verbindung hergestellt")
                return True
            rospy.sleep(0.2)
        rospy.logerr("MAVROS-Verbindung fehlgeschlagen")
        return False

    def arm_system(self):
        """
        Aktiviert die Systeme der Drohne
        Returns:
            bool: True bei erfolgreichem Arming
        """
        rospy.loginfo("Systemaktivierung...")
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        for _ in range(5):
            if self.arming_client.call(arm_cmd).success:
                rospy.loginfo("System erfolgreich aktiviert")
                return True
            rospy.sleep(1)
        rospy.logerr("Systemaktivierung fehlgeschlagen")
        return False

    def engage_offboard_control(self):
        """
        Aktiviert den OFFBOARD-Kontrollmodus
        Returns:
            bool: True bei erfolgreicher Aktivierung
        """
        rospy.loginfo("Aktiviere OFFBOARD-Kontrolle...")
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        # Setpoint-Vorbereitung
        for _ in range(50):
            self.local_pos_pub.publish(self.pose)
            rospy.sleep(0.1)

        for _ in range(5):
            if self.set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD-Kontrolle aktiviert")
                return True
            rospy.sleep(1)
        rospy.logerr("OFFBOARD-Aktivierung fehlgeschlagen")
        return False

    def execute_mission(self):
        """
        Führt die Hauptflugroutine aus
        Returns:
            bool: True bei erfolgreicher Mission
        """
        try:
            if not all([
                self.establish_connection(),
                self.engage_offboard_control(),
                self.arm_system()
            ]):
                return False

            rospy.loginfo("Starte Ashigaru-Testflug...")

            # Aufstiegsphase
            self._execute_ascent()

            # Wegpunkt-Navigation
            self._execute_waypoint_navigation()

            # Landephase
            self._execute_landing()

            return True

        except Exception as e:
            rospy.logerr(f"Missionsfehler: {e}")
            return False
        finally:
            self.cleanup()

    def _execute_ascent(self):
        """Führt die Aufstiegsphase aus"""
        rospy.loginfo("Steigphase eingeleitet...")
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < rospy.Duration(5.0):
            self.local_pos_pub.publish(self.pose)
            rospy.sleep(0.1)

    def _execute_waypoint_navigation(self):
        """Führt die Wegpunkt-Navigation aus"""
        waypoints = [
            (2, 0, 2),  # Vorwärts
            (2, 2, 2),  # Rechts
            (0, 2, 2),  # Rückwärts
            (0, 0, 2)   # Links
        ]

        for wp in waypoints:
            rospy.loginfo(f"Navigation zu Wegpunkt: {wp}")
            self.pose.pose.position.x = wp[0]
            self.pose.pose.position.y = wp[1]
            self.pose.pose.position.z = wp[2]

            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time) < rospy.Duration(5.0):
                self.local_pos_pub.publish(self.pose)
                rospy.sleep(0.1)

    def _execute_landing(self):
        """Führt die Landephase aus"""
        rospy.loginfo("Leite Landung ein...")
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0

        for _ in range(50):
            self.local_pos_pub.publish(self.pose)
            rospy.sleep(0.1)

        land_set_mode = SetModeRequest()
        land_set_mode.custom_mode = 'AUTO.LAND'
        if self.set_mode_client.call(land_set_mode).mode_sent:
            rospy.loginfo("Landephase aktiviert")


if __name__ == "__main__":
    try:
        controller = AshigaruFlightController()
        controller.execute_mission()
    except rospy.ROSInterruptException:
        if hasattr(controller, 'cleanup'):
            controller.cleanup()
