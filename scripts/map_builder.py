#!/usr/bin/env python3
import rospy
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import math
import numpy as np

class MapBuilder:
    def __init__(self):
        rospy.init_node('map_builder', anonymous=True)
        
        # Parámetros del mapa
        self.map_width = 5  # en metros
        self.map_height = 5
        self.map_resolution = 0.01  # 5 cm por celda
        self.map_center_x = self.map_width / 2
        self.map_center_y = self.map_height / 2
        
        # Inicializar mapa (matriz 2D)
        self.grid_width = int(self.map_width / self.map_resolution)
        self.grid_height = int(self.map_height / self.map_resolution)
        self.grid = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)  # -1 = desconocido
        
        # Transformación de coordenadas
        self.tf_listener = tf.TransformListener()
        
        # Suscriptores
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/natnet_ros/base_link/pose', PoseStamped, self.pose_callback)
        
        # Publicador del mapa
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        
        # Para almacenar la última posición conocida
        self.current_pose = None
        self.last_map_update_time = rospy.Time.now()
        
        # Tasa de actualización del mapa (Hz)
        self.rate = rospy.Rate(1)
        
    def pose_callback(self, msg):
        """Callback para actualizar la posición del robot"""
        self.current_pose = msg.pose
        
    def scan_callback(self, msg):
        """Callback para procesar los datos del LIDAR"""
        if self.current_pose is None:
            return
            
        # Solo actualizamos el mapa periódicamente para no sobrecargar
        if (rospy.Time.now() - self.last_map_update_time).to_sec() < 0.5:
            return
            
        self.last_map_update_time = rospy.Time.now()
        
        try:
            # Obtener la orientación del robot (yaw) del cuaternión
            orientation = self.current_pose.orientation
            (roll, pitch, yaw) = euler_from_quaternion([orientation.x, 
                                                       orientation.y, 
                                                       orientation.z, 
                                                       orientation.w])
            
            # Posición del robot en el mapa global
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y
            
            # Convertir posición del robot a coordenadas del grid
            grid_x = int(robot_x / self.map_resolution + self.map_center_x / self.map_resolution)
            grid_y = int(robot_y / self.map_resolution + self.map_center_y / self.map_resolution)


            # Filtrado de ruido - parámetros ajustables
            max_range = msg.range_max * 0.95  # Ignorar lecturas cerca del máximo
            min_range = msg.range_min * 1.05  # Ignorar lecturas cerca del mínimo
            jump_threshold = 0.2  # Umbral para detectar saltos bruscos (en metros)
            window_size = 5  # Tamaño de la ventana para el filtro de mediana
            
            filtered_ranges = []
            
            # Procesar cada lectura del LIDAR
            for i, distance in enumerate(msg.ranges):

                # Filtro 1: Eliminar valores NaN e infinitos
                if math.isnan(msg.ranges[i]) or math.isinf(msg.ranges[i]):
                    filtered_ranges.append(float('nan'))
                    continue
                    
                # Filtro 2: Eliminar lecturas fuera de rango válido
                if msg.ranges[i] < min_range or msg.ranges[i] > max_range:
                    filtered_ranges.append(float('nan'))
                    continue

                # Filtro 3: Detección de saltos bruscos (comparar con vecinos)
                if i > 0 and i < len(msg.ranges)-1:
                    prev = msg.ranges[i-1] if not math.isnan(msg.ranges[i-1]) else msg.ranges[i]
                    next = msg.ranges[i+1] if not math.isnan(msg.ranges[i+1]) else msg.ranges[i]
                    if (abs(msg.ranges[i] - prev) > jump_threshold and 
                        abs(msg.ranges[i] - next) > jump_threshold):
                        filtered_ranges.append(float('nan'))
                        continue

                 # Filtro 4: Filtro de mediana para ventana deslizante
                window = []
                for j in range(max(0, i-window_size//2), min(len(msg.ranges), i+window_size//2 +1)):
                    if not math.isnan(msg.ranges[j]) and not math.isinf(msg.ranges[j]):
                        window.append(msg.ranges[j])
                
                if window:
                    median = sorted(window)[len(window)//2]
                    if abs(msg.ranges[i] - median) > jump_threshold:
                        filtered_ranges.append(float('nan'))
                    else:
                        filtered_ranges.append(msg.ranges[i])
                else:
                    filtered_ranges.append(float('nan'))

                
            for i, distance in enumerate(filtered_ranges):
                if math.isnan(distance):
                    continue
                    
                angle = msg.angle_min + i * msg.angle_increment
                global_angle = angle + yaw
                
                obstacle_x = robot_x + distance * math.cos(global_angle)
                obstacle_y = robot_y + distance * math.sin(global_angle)
                
                obstacle_grid_x = int(obstacle_x / self.map_resolution + self.map_center_x / self.map_resolution)
                obstacle_grid_y = int(obstacle_y / self.map_resolution + self.map_center_y / self.map_resolution)
                
                if (0 <= obstacle_grid_x < self.grid_width and 
                    0 <= obstacle_grid_y < self.grid_height):
                    # Suavizado: incrementar valor en lugar de establecer a 100 directamente
                    current_val = self.grid[obstacle_grid_y, obstacle_grid_x]
                    if current_val < 100:
                        self.grid[obstacle_grid_y, obstacle_grid_x] = min(100, current_val + 30)
                
                self.mark_free_cells(grid_x, grid_y, obstacle_grid_x, obstacle_grid_y)
            
            # Publicar el mapa actualizado
            self.publish_map()
            
        except Exception as e:
            rospy.logerr("Error processing scan: %s", str(e))
    
    def mark_free_cells(self, x1, y1, x2, y2):
        """Marcar las celdas entre (x1,y1) y (x2,y2) como libres (algoritmo de Bresenham)"""
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        sx = -1 if x1 > x2 else 1
        sy = -1 if y1 > y2 else 1
        
        if dx > dy:
            err = dx / 2.0
            while x != x2:
                if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                    if self.grid[y, x] != 100:  # No sobrescribir obstáculos
                        self.grid[y, x] = 0  # Libre
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y2:
                if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                    if self.grid[y, x] != 100:  # No sobrescribir obstáculos
                        self.grid[y, x] = 0  # Libre
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        
        # También marcar la posición del robot como libre
        if 0 <= x1 < self.grid_width and 0 <= y1 < self.grid_height:
            self.grid[y1, x1] = 0
    
    def publish_map(self):
        """Publicar el mapa como un OccupancyGrid"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "world"
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.grid_width
        map_msg.info.height = self.grid_height
        
        # El origen del mapa está en la esquina inferior izquierda
        map_msg.info.origin.position.x = -self.map_center_x
        map_msg.info.origin.position.y = -self.map_center_y
        map_msg.info.origin.position.z = 0
        map_msg.info.origin.orientation.w = 1.0
        
        # Aplanar la matriz del grid y convertir a lista
        map_msg.data = self.grid.flatten().tolist()
        
        self.map_pub.publish(map_msg)
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        map_builder = MapBuilder()
        map_builder.run()
    except rospy.ROSInterruptException:
        pass