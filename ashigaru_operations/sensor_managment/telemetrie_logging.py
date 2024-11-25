#!/usr/bin/env python3
#Verzeichnis Bakufu_Drone_System/ashigaru_operations/sensor_managment

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class AshigaruTelemetryLogger:
    """
    Ashigaru Telemetrie-System
    Verantwortlich für die Aufzeichnung und Speicherung von Flugdaten und Videostreams.
    """
    def __init__(self):
        self._initialize_ros_node()
        self._setup_recording_directory()
        self._initialize_cv_bridge()
        self._initialize_video_writer()
        self._setup_subscribers()
        self._initialize_state_variables()

    def _initialize_ros_node(self):
        """Initialisiert den ROS-Node"""
        try:
            rospy.init_node('ashigaru_telemetry', anonymous=True)
            rospy.loginfo("Ashigaru Telemetrie-System initialisiert")
        except Exception as e:
            rospy.logerr(f"ROS-Node-Initialisierung fehlgeschlagen: {e}")
            raise

    def _setup_recording_directory(self):
        """Erstellt und konfiguriert das Aufzeichnungsverzeichnis"""
        try:
            self.recording_dir = os.path.join(
                os.path.expanduser('~'),
                'bakufu_drone_system/telemetry_data',
                datetime.now().strftime('%Y%m%d_%H%M%S')
            )
            os.makedirs(self.recording_dir, exist_ok=True)
            rospy.loginfo(f"Telemetrie-Verzeichnis erstellt: {self.recording_dir}")
        except Exception as e:
            rospy.logerr(f"Verzeichniserstellung fehlgeschlagen: {e}")
            raise

    def _initialize_cv_bridge(self):
        """Initialisiert die OpenCV Bridge"""
        try:
            self.bridge = CvBridge()
            rospy.loginfo("CV Bridge initialisiert")
        except Exception as e:
            rospy.logerr(f"CV Bridge Initialisierung fehlgeschlagen: {e}")
            raise

    def _initialize_video_writer(self):
        """Initialisiert den Video Writer"""
        self.video_file = os.path.join(self.recording_dir, 'mission_recording.avi')
        self.video_writer = None
        self.frame_size = None

    def _setup_subscribers(self):
        """Richtet die ROS-Subscriber ein"""
        try:
            rospy.loginfo("Überprüfe verfügbare Topics...")
            topics = rospy.get_published_topics()
            for topic in topics:
                rospy.loginfo(f"Gefunden: {topic}")

            self.image_sub = rospy.Subscriber(
                "/iris/camera/image_raw",
                Image,
                self._image_callback
            )
            rospy.loginfo("Bildempfang aktiviert")
        except Exception as e:
            rospy.logerr(f"Subscriber-Setup fehlgeschlagen: {e}")
            raise

    def _initialize_state_variables(self):
        """Initialisiert die Statusvariablen"""
        self.frame_count = 0
        self.received_first_frame = False
        self.is_recording = True

    def _image_callback(self, msg):
        """
        Callback für eingehende Bildframes
        Args:
            msg: ROS Image Message
        """
        try:
            if not self.received_first_frame:
                rospy.loginfo("Erstes Bild empfangen")
                self.received_first_frame = True

            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self._process_frame(cv_image)

        except Exception as e:
            rospy.logerr(f"Bildverarbeitung fehlgeschlagen: {e}")
            self._log_frame_error(msg)

    def _process_frame(self, cv_image):
        """
        Verarbeitet und speichert einen Bildframe
        Args:
            cv_image: OpenCV Bildframe
        """
        if self.video_writer is None:
            self._initialize_frame_writer(cv_image)

        if self.video_writer and self.is_recording:
            self.video_writer.write(cv_image)
            self.frame_count += 1
            self._log_frame_status()

    def _initialize_frame_writer(self, first_frame):
        """
        Initialisiert den Frame Writer mit dem ersten Frame
        Args:
            first_frame: Erster OpenCV Bildframe
        """
        try:
            self.frame_size = (first_frame.shape[1], first_frame.shape[0])
            rospy.loginfo(f"Initialisiere VideoWriter mit Format: {self.frame_size}")

            self.video_writer = cv2.VideoWriter(
                self.video_file,
                cv2.VideoWriter_fourcc(*'XVID'),
                30,  # FPS
                self.frame_size,
                isColor=True
            )

            if not self.video_writer.isOpened():
                raise Exception("VideoWriter konnte nicht geöffnet werden")

            rospy.loginfo(f"Videoaufzeichnung gestartet: {self.video_file}")

        except Exception as e:
            rospy.logerr(f"VideoWriter Initialisierung fehlgeschlagen: {e}")
            raise

    def _log_frame_status(self):
        """Loggt den Status der Frameaufzeichnung"""
        if self.frame_count % 30 == 0:  # Log alle 30 Frames
            rospy.loginfo(f"Aufgezeichnete Frames: {self.frame_count}")

    def _log_frame_error(self, msg):
        """
        Loggt Details bei Frame-Fehlern
        Args:
            msg: Problematische ROS-Message
        """
        rospy.logerr(f"Frame Info - Typ: {type(msg)}")
        rospy.logerr(f"Message Details: {msg._type}")

    def run(self):
        """Hauptausführungsschleife"""
        rospy.loginfo(f"Ashigaru Telemetrie-System aktiv: {self.recording_dir}")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.cleanup()
        except Exception as e:
            rospy.logerr(f"Unerwarteter Fehler: {e}")
            self.cleanup()

    def cleanup(self):
        """Aufräumen und Ressourcen freigeben"""
        if self.video_writer is not None:
            self.video_writer.release()
            rospy.loginfo(f"Aufzeichnung gespeichert: {self.video_file}")
            rospy.loginfo(f"Gesamtanzahl Frames: {self.frame_count}")
        rospy.loginfo("Ashigaru Telemetrie-System beendet")


if __name__ == "__main__":
    try:
        logger = AshigaruTelemetryLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass
