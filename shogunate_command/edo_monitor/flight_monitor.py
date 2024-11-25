#!/usr/bin/env python3
# im Verzeichnis Bakufu_Drone_System/shogunate_command/edo_monitor

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading
from queue import Queue
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

class EdoFlightMonitor:
    """
    Edo Monitor System für Bakufu Drohnen
    Visualisiert Flugdaten und Systemstatus in Echtzeit.
    """
    def __init__(self):
        self._initialize_ros()
        self._setup_data_structures()
        self._initialize_gui()
        self._setup_subscribers()

    def _initialize_ros(self):
        """Initialisiert den ROS-Node"""
        try:
            rospy.init_node('edo_flight_monitor')
            rospy.loginfo("Edo Monitor System initialisiert")
        except Exception as e:
            rospy.logerr(f"ROS-Initialisierung fehlgeschlagen: {e}")
            raise

    def _setup_data_structures(self):
        """Initialisiert die Datenstrukturen für die Flugdaten"""
        self.timestamps = []
        self.positions = []
        self.modes = []
        self.armed_states = []
        self.start_time = rospy.Time.now()
        self.data_queue = Queue()
        self.current_state = None
        self.current_pose = None

    def _initialize_gui(self):
        """Initialisiert die GUI-Komponenten"""
        try:
            self._setup_tkinter_window()
            self._setup_matplotlib_figures()
            self._setup_plot_layout()
            self._create_canvas()
        except Exception as e:
            rospy.logerr(f"GUI-Initialisierung fehlgeschlagen: {e}")
            raise

    def _setup_tkinter_window(self):
        """Richtet das Tkinter-Hauptfenster ein"""
        self.root = tk.Tk()
        self.root.title("Edo Flight Monitor - Bakufu System")
        # Füge Icon oder weitere Fenster-Eigenschaften hier hinzu

    def _setup_matplotlib_figures(self):
        """Konfiguriert die Matplotlib-Figuren"""
        self.fig = plt.figure(figsize=(15, 5))
        self._setup_3d_trajectory()
        self._setup_altitude_profile()
        self._setup_status_display()

    def _setup_3d_trajectory(self):
        """Konfiguriert den 3D-Trajektorienplot"""
        self.ax1 = self.fig.add_subplot(131, projection='3d')
        self.ax1.set_title('Flugtrajektorie')
        self.ax1.set_xlabel('X [m]')
        self.ax1.set_ylabel('Y [m]')
        self.ax1.set_zlabel('Z [m]')

    def _setup_altitude_profile(self):
        """Konfiguriert den Höhenprofilplot"""
        self.ax2 = self.fig.add_subplot(132)
        self.ax2.set_title('Höhenprofil')
        self.ax2.set_xlabel('Zeit [s]')
        self.ax2.set_ylabel('Höhe [m]')
        self.ax2.grid(True)

    def _setup_status_display(self):
        """Konfiguriert die Statusanzeige"""
        self.ax3 = self.fig.add_subplot(133)
        self.ax3.set_title('System Status')
        self.ax3.axis('off')
        self.status_text = self.ax3.text(0.1, 0.5, '', fontsize=12)

    def _create_canvas(self):
        """Erstellt und konfiguriert den Matplotlib-Canvas"""
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    def _setup_subscribers(self):
        """Richtet die ROS-Subscriber ein"""
        try:
            self.state_sub = rospy.Subscriber(
                "mavros/state",
                State,
                self._state_callback
            )
            self.pose_sub = rospy.Subscriber(
                "mavros/local_position/pose",
                PoseStamped,
                self._pose_callback
            )
            rospy.loginfo("ROS-Subscriber erfolgreich eingerichtet")
        except Exception as e:
            rospy.logerr(f"Subscriber-Setup fehlgeschlagen: {e}")
            raise

    def _state_callback(self, msg):
        """
        Callback für Statusupdates
        Args:
            msg: State Message
        """
        self.current_state = msg
        self.modes.append(msg.mode)
        self.armed_states.append(msg.armed)
        self.data_queue.put(('state', msg))

    def _pose_callback(self, msg):
        """
        Callback für Positionsupdates
        Args:
            msg: PoseStamped Message
        """
        self.current_pose = msg
        t = (msg.header.stamp - self.start_time).to_sec()
        self.timestamps.append(t)
        pos = msg.pose.position
        self.positions.append([pos.x, pos.y, pos.z])
        self.data_queue.put(('pose', msg))

    def update_plots(self):
        """Aktualisiert alle Plots mit den neuesten Daten"""
        if not self.positions:
            self.root.after(100, self.update_plots)
            return

        self._update_trajectory_plot()
        self._update_altitude_plot()
        self._update_status_display()

        self.canvas.draw()
        self.root.after(100, self.update_plots)

    def _update_trajectory_plot(self):
        """Aktualisiert den Trajektorienplot"""
        positions = np.array(self.positions)
        self.ax1.cla()
        self.ax1.plot(positions[:,0], positions[:,1], positions[:,2], 'b-')
        self.ax1.scatter(positions[-1,0], positions[-1,1], positions[-1,2],
                        color='red', marker='o', s=100)
        self._set_equal_aspect_3d(positions)

    def _set_equal_aspect_3d(self, positions):
        """
        Setzt gleiche Aspektverhältnisse für 3D-Plot
        Args:
            positions: Array der Positionen
        """
        max_range = np.array([
            positions[:,0].max()-positions[:,0].min(),
            positions[:,1].max()-positions[:,1].min(),
            positions[:,2].max()-positions[:,2].min()
        ]).max()

        if max_range > 0:
            mid_x = (positions[:,0].max()+positions[:,0].min()) * 0.5
            mid_y = (positions[:,1].max()+positions[:,1].min()) * 0.5
            mid_z = (positions[:,2].max()+positions[:,2].min()) * 0.5
            self.ax1.set_xlim(mid_x - max_range*0.5, mid_x + max_range*0.5)
            self.ax1.set_ylim(mid_y - max_range*0.5, mid_y + max_range*0.5)
            self.ax1.set_zlim(mid_z - max_range*0.5, mid_z + max_range*0.5)

    def _update_altitude_plot(self):
        """Aktualisiert den Höhenprofilplot"""
        positions = np.array(self.positions)
        timestamps = np.array(self.timestamps)
        self.ax2.cla()
        self.ax2.plot(timestamps, positions[:,2], 'g-')
        self.ax2.set_xlabel('Zeit [s]')
        self.ax2.set_ylabel('Höhe [m]')
        self.ax2.grid(True)

    def _update_status_display(self):
        """Aktualisiert die Statusanzeige"""
        if self.current_state and self.current_pose:
            status_str = (
                f"Flugmodus: {self.current_state.mode}\n"
                f"Armed: {'Ja' if self.current_state.armed else 'Nein'}\n"
                f"Position:\n"
                f"X: {self.current_pose.pose.position.x:.2f}m\n"
                f"Y: {self.current_pose.pose.position.y:.2f}m\n"
                f"Z: {self.current_pose.pose.position.z:.2f}m"
            )
            self.status_text.set_text(status_str)

    def run(self):
        """Hauptausführungsschleife"""
        try:
            rospy.loginfo("Edo Monitor gestartet")
            self.root.after(100, self.update_plots)
            self.root.mainloop()
        except KeyboardInterrupt:
            self.cleanup()
        except Exception as e:
            rospy.logerr(f"Laufzeitfehler: {e}")
            self.cleanup()

    def cleanup(self):
        """Aufräumen und Ressourcen freigeben"""
        self.root.quit()
        plt.close('all')
        rospy.loginfo("Edo Monitor beendet")


if __name__ == "__main__":
    try:
        monitor = EdoFlightMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
