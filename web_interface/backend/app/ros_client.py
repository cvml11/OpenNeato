import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState

class RosClient(Node):
    def __init__(self):
        super().__init__('openneato_backend_client')
        
        # Publisher per Stop Emergenza
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber per Batteria
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        
        # Stato Interno
        self.battery_level = 0.0
        self.is_cleaning = False
        self.current_state = "IDLE"
        
        # Threading
        self._spin_thread = None
        self._stop_event = threading.Event()

    def battery_callback(self, msg: BatteryState):
        """Aggiorna lo stato della batteria."""
        # Normalizza se necessario (assumiamo 0.0-1.0 o 0-100)
        self.battery_level = msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage

    def start(self):
        """Avvia il thread di spinning ROS 2."""
        self._stop_event.clear()
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        self.get_logger().info("ROS Client Thread Started")

    def stop(self):
        """Ferma il thread."""
        self._stop_event.set()
        if self._spin_thread:
            self._spin_thread.join()

    def _spin(self):
        while rclpy.ok() and not self._stop_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)

    def send_stop_command(self):
        """Invia comando di stop (velocità zero)."""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        self.is_cleaning = False
        self.current_state = "STOPPED"
        self.get_logger().info("Emergency Stop Sent")

    def start_cleaning(self, zone_ids: list, suction_power: int):
        """Simula l'avvio della pulizia (Action Client placeholder)."""
        # Qui andrebbe l'Action Client per Nav2 o logica custom
        self.is_cleaning = True
        self.current_state = f"CLEANING (Zones: {len(zone_ids)}, Power: {suction_power}%)"
        self.get_logger().info(f"Cleaning started: {self.current_state}")
