#!/usr/bin/env python3
import math
from typing import List, Tuple
from geometry_msgs.msg import PoseStamped

class ZoneCoveragePlanner:
    """
    Gestisce la pianificazione del percorso di copertura (Coverage Path Planning)
    per le zone di pulizia utilizzando un algoritmo Boustrophedon (a serpentina).
    """
    def __init__(self, stride: float = 0.25):
        """
        Inizializza il pianificatore.
        
        Args:
            stride (float): Distanza tra le linee parallele (metri). 
                            Default 0.25m (metà larghezza robot tipico).
        """
        self.stride = stride

    def generate_boustrophedon_path(self, zone_points: List[Tuple[float, float]]) -> List[PoseStamped]:
        """
        Genera una lista di waypoint a zig-zag basata sul Bounding Box dei punti forniti.
        
        Args:
            zone_points: Lista di tuple (x, y) che definiscono i vertici della zona.
            
        Returns:
            List[PoseStamped]: Lista ordinata di waypoint per il navigatore.
        """
        if not zone_points:
            return []

        # 1. Calcolo Bounding Box
        # Estraiamo min/max X e Y dai punti forniti
        x_coords = [p[0] for p in zone_points]
        y_coords = [p[1] for p in zone_points]
        
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        
        waypoints = []
        y = y_min
        row_count = 0
        
        # 2. Generazione percorso a serpentina
        # Iteriamo lungo l'asse Y con passo 'stride' fino a coprire Y_max
        while y <= y_max:
            # Creiamo i PoseStamped per inizio e fine riga
            p_start = PoseStamped()
            p_start.header.frame_id = "map"
            p_start.pose.position.y = y
            p_start.pose.orientation.w = 1.0 # Orientamento neutro
            
            p_end = PoseStamped()
            p_end.header.frame_id = "map"
            p_end.pose.position.y = y
            p_end.pose.orientation.w = 1.0

            if row_count % 2 == 0:
                # Riga pari: Da Sinistra (X_min) a Destra (X_max)
                p_start.pose.position.x = x_min
                p_end.pose.position.x = x_max
            else:
                # Riga dispari: Da Destra (X_max) a Sinistra (X_min)
                p_start.pose.position.x = x_max
                p_end.pose.position.x = x_min

            # Aggiungiamo i punti alla lista
            waypoints.append(p_start)
            waypoints.append(p_end)
            
            y += self.stride
            row_count += 1
            
        return waypoints