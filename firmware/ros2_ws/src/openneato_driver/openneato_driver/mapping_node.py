#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.sub_imu = self.create_subscription(Imu, 'imu', self.imu_callback, 10)

        # Parametri PID
        self.kp = 2.0
        self.ki = 0.0
        self.kd = 0.5
        
        self.target_distance = 0.10 # 10 cm
        self.forward_speed = 0.15
        
        self.prev_error = 0.0
        self.integral = 0.0
        
        # Loop Closure
        self.start_pose = None
        self.current_pose = None
        self.start_time = self.get_clock().now()
        self.min_loop_time = 30.0 # Secondi minimi prima di controllare chiusura loop
        self.loop_closed_threshold = 0.5 # Metri dall'origine
        self.is_finished = False
        
        # Recovery Logic
        self.is_recovering = False
        self.last_stuck_check_pos = None
        self.last_stuck_check_time = self.get_clock().now()

        self.get_logger().info("Wall Follower (Right) Started")

    def imu_callback(self, msg):
        if self.is_recovering or self.is_finished:
            return

        # Calcolo Pitch da Quaternione (semplificato)
        # sin(p) = 2(w*y - z*x)
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        
        # Gestione limiti numerici
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        pitch_deg = math.degrees(pitch)
        
        # Anti-Beach Logic
        if pitch_deg > 5.0:
            self.get_logger().warn(f"Anti-Beach Triggered! Pitch: {pitch_deg:.2f}°")
            self.stop_robot()
            self.perform_recovery()

    def perform_recovery(self):
        """Manovra di sblocco: Indietro 15cm -> Ruota 30°"""
        self.is_recovering = True
        
        # 1. Indietro 15cm (v=-0.15 m/s per 1s)
        twist = Twist()
        twist.linear.x = -0.15
        self.pub_vel.publish(twist)
        time.sleep(1.0)
        
        # 2. Ruota 30° (v_ang=0.5 rad/s per ~1s)
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        self.pub_vel.publish(twist)
        time.sleep(1.0)
        
        self.stop_robot()
        self.is_recovering = False
        # Reset stuck check
        self.last_stuck_check_time = self.get_clock().now()
        self.last_stuck_check_pos = self.current_pose

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        if self.start_pose is None:
            self.start_pose = pos
            self.get_logger().info(f"Start Pose recorded: {pos.x:.2f}, {pos.y:.2f}")
        
        self.current_pose = pos
        
        # Check Loop Closure
        if self.start_pose and not self.is_finished:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > self.min_loop_time:
                dist = math.sqrt((pos.x - self.start_pose.x)**2 + (pos.y - self.start_pose.y)**2)
                if dist < self.loop_closed_threshold:
                    self.get_logger().info("Loop Closed! Stopping mapping.")
                    self.stop_robot()
                    self.is_finished = True
        
        # Stuck Detection
        if not self.is_recovering and not self.is_finished:
            now = self.get_clock().now()
            if (now - self.last_stuck_check_time).nanoseconds / 1e9 > 2.0:
                if self.last_stuck_check_pos:
                    d = math.sqrt((pos.x - self.last_stuck_check_pos.x)**2 + 
                                  (pos.y - self.last_stuck_check_pos.y)**2)
                    # Se ci siamo mossi meno di 2cm ma stiamo comandando velocità
                    if d < 0.02:
                        self.get_logger().warn("Stuck Detected! Odometry not changing.")
                        self.perform_recovery()
                
                self.last_stuck_check_pos = pos
                self.last_stuck_check_time = now

    def scan_callback(self, msg):
        if self.is_finished or self.is_recovering:
            return

        # Neato Lidar: 360 punti. 0=Front, 90=Left, 180=Back, 270=Right.
        # Indice per 270 gradi (Destra)
        # Assumiamo angle_min=0, increment=1 grado (approx)
        # Calcolo indice preciso:
        angle_range = msg.angle_max - msg.angle_min
        num_readings = len(msg.ranges)
        
        # Angolo target in radianti (270 deg = 3/4 * 2pi = 4.71 rad)
        # Attenzione: verificare frame Lidar. Spesso 0 è avanti X.
        # Se 0 è avanti, 270 gradi è -90 gradi (o 3/2 pi).
        target_angle = math.radians(270)
        
        # Trova indice corrispondente
        # index = (angle - angle_min) / angle_increment
        idx = int((target_angle - msg.angle_min) / msg.angle_increment)
        
        # Clamp index
        idx = max(0, min(idx, num_readings - 1))
        
        # Prendi una media di 5 raggi intorno a 270 per stabilità
        valid_ranges = []
        for i in range(idx - 2, idx + 3):
            if 0 <= i < num_readings:
                r = msg.ranges[i]
                if msg.range_min < r < msg.range_max:
                    valid_ranges.append(r)
        
        if not valid_ranges:
            return # Nessun dato valido
            
        current_dist = sum(valid_ranges) / len(valid_ranges)
        
        # Calcolo PID
        error = self.target_distance - current_dist
        self.integral += error
        derivative = error - self.prev_error
        
        angular_z = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # Se il muro è troppo lontano (> 30cm), ruota a destra per cercarlo (comportamento recovery)
        if current_dist > 0.3:
            angular_z = -0.5 

        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = angular_z # Correzione rotazione
        
        self.pub_vel.publish(twist)
        self.prev_error = error

    def stop_robot(self):
        self.pub_vel.publish(Twist())

def main():
    rclpy.init()
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
