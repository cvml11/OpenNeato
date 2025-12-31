#!/usr/bin/env python3
import rclpy
import json
import os
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient

# Import interfaccia action (vedi nota in docking_server.py)
try:
    from openneato_interfaces.action import DockToBase
except ImportError:
    class DockToBase:
        class Goal: pass

class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        
        self.navigator = BasicNavigator()
        self.docking_client = ActionClient(self, DockToBase, 'dock_to_base')
        
        self.sub_battery = self.create_subscription(BatteryState, 'battery_state', self.battery_callback, 10)
        
        self.state_file = "mission_dump.json"
        self.battery_level = 100.0
        self.is_mission_active = False
        self.current_waypoints = []
        self.current_waypoint_index = 0
        
        # Timer persistenza (10s)
        self.create_timer(10.0, self.save_state)
        
        # Timer logica principale
        self.create_timer(1.0, self.control_loop)

        self.get_logger().info("Mission Control Initialized")

        # Check resume all'avvio
        self.check_resume_condition()

    def battery_callback(self, msg):
        # Normalizza percentuale 0-100
        self.battery_level = msg.percentage if msg.percentage > 1.0 else msg.percentage * 100.0

    def save_state(self):
        if not self.is_mission_active:
            return

        data = {
            "timestamp": time.time(),
            "waypoints": [self.pose_to_dict(p) for p in self.current_waypoints],
            "current_index": self.current_waypoint_index,
            "zone": "living_room" # Placeholder per logica zone
        }
        
        try:
            with open(self.state_file, 'w') as f:
                json.dump(data, f)
            self.get_logger().debug("Stato missione salvato.")
        except Exception as e:
            self.get_logger().error(f"Errore salvataggio stato: {e}")

    def load_state(self):
        if not os.path.exists(self.state_file):
            return None
        try:
            with open(self.state_file, 'r') as f:
                return json.load(f)
        except Exception:
            return None

    def check_resume_condition(self):
        # Se batteria carica e c'è un dump valido
        if self.battery_level > 90.0:
            state = self.load_state()
            if state:
                self.get_logger().info("Condizioni Resume soddisfatte. Caricamento missione...")
                self.resume_mission(state)

    def resume_mission(self, state):
        raw_waypoints = state.get('waypoints', [])
        start_index = state.get('current_index', 0)
        
        if not raw_waypoints or start_index >= len(raw_waypoints):
            self.get_logger().info("Missione completata o vuota nel dump.")
            return

        # Ricostruisci oggetti PoseStamped
        self.current_waypoints = [self.dict_to_pose(p) for p in raw_waypoints]
        self.current_waypoint_index = start_index
        
        # Riprendi dal punto corrente
        remaining_poses = self.current_waypoints[start_index:]
        self.navigator.followWaypoints(remaining_poses)
        self.is_mission_active = True

    def control_loop(self):
        # Watchdog Batteria
        if self.is_mission_active and self.battery_level < 20.0:
            self.get_logger().warn("Batteria critica (<20%). Abort missione e rientro alla base.")
            self.abort_and_dock()
            return

        # Monitoraggio Navigazione
        if self.is_mission_active:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("Missione completata!")
                    # Pulisci file stato
                    if os.path.exists(self.state_file):
                        os.remove(self.state_file)
                self.is_mission_active = False

    def abort_and_dock(self):
        self.navigator.cancelTask()
        self.save_state() # Salva ultimo stato noto
        self.is_mission_active = False
        
        self.get_logger().info("Inviando richiesta al Docking Server...")
        goal_msg = DockToBase.Goal()
        self.docking_client.wait_for_server()
        self.docking_client.send_goal_async(goal_msg)

    # Helpers serializzazione
    def pose_to_dict(self, pose_stamped):
        return {
            'x': pose_stamped.pose.position.x,
            'y': pose_stamped.pose.position.y,
            'z': pose_stamped.pose.position.z,
            'qx': pose_stamped.pose.orientation.x,
            'qy': pose_stamped.pose.orientation.y,
            'qz': pose_stamped.pose.orientation.z,
            'qw': pose_stamped.pose.orientation.w,
            'frame_id': pose_stamped.header.frame_id
        }

    def dict_to_pose(self, d):
        p = PoseStamped()
        p.header.frame_id = d['frame_id']
        p.pose.position.x = d['x']
        p.pose.position.y = d['y']
        p.pose.position.z = d['z']
        p.pose.orientation.x = d['qx']
        p.pose.orientation.y = d['qy']
        p.pose.orientation.z = d['qz']
        p.pose.orientation.w = d['qw']
        return p

def main():
    rclpy.init()
    node = MissionControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()