#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import transformations as tf_transformations  # Usar el paquete transformations
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class Particle:
    def __init__(self, x, y, theta, weight, map_shape):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight
        self.log_odds_map = np.zeros(map_shape, dtype=np.float32)

    def pose(self):
        return np.array([self.x, self.y, self.theta])

class PythonSlamNode(Node):
    def __init__(self):
        super().__init__('python_slam_node')

        # Parameters
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # TODO: define map resolution, width, height, and number of particles
        self.declare_parameter('map_resolution', 0.05)  # metros/celda
        self.declare_parameter('map_width_meters', 10.0)
        self.declare_parameter('map_height_meters', 10.0)
        self.declare_parameter('num_particles', 100)
        
        # Parámetros de ruido configurables
        self.declare_parameter('motion_noise', 0.1)     # Ruido en movimiento (metros)
        self.declare_parameter('rotation_noise', 0.1)   # Ruido en rotación (radianes)
        
        # Parámetros del mapa log-odds
        self.declare_parameter('log_odds_free', -0.4)    # Log-odds para celdas libres
        self.declare_parameter('log_odds_occupied', 0.85) # Log-odds para celdas ocupadas
        self.declare_parameter('log_odds_max', 5.0)      # Máximo log-odds
        self.declare_parameter('log_odds_min', -5.0)     # Mínimo log-odds

        self.resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_width_m = self.get_parameter('map_width_meters').get_parameter_value().double_value
        self.map_height_m = self.get_parameter('map_height_meters').get_parameter_value().double_value
        self.map_width_cells = int(self.map_width_m / self.resolution)
        self.map_height_cells = int(self.map_height_m / self.resolution)
        self.map_origin_x = -self.map_width_m / 2.0
        self.map_origin_y = -5.0

        # Usar parámetros configurables para log-odds
        self.log_odds_free = self.get_parameter('log_odds_free').get_parameter_value().double_value
        self.log_odds_occupied = self.get_parameter('log_odds_occupied').get_parameter_value().double_value
        self.log_odds_max = self.get_parameter('log_odds_max').get_parameter_value().double_value
        self.log_odds_min = self.get_parameter('log_odds_min').get_parameter_value().double_value

        # Particle filter
        self.num_particles = self.get_parameter('num_particles').get_parameter_value().integer_value
        self.particles = [Particle(0.0, 0.0, 0.0, 1.0/self.num_particles, (self.map_height_cells, self.map_width_cells)) for _ in range(self.num_particles)]
        self.last_odom = None
        
        # Variables críticas usando parámetros configurables
        self.motion_noise = self.get_parameter('motion_noise').get_parameter_value().double_value
        self.rotation_noise = self.get_parameter('rotation_noise').get_parameter_value().double_value
        self.current_map_pose = (0.0, 0.0, 0.0)  # Pose actual en mapa (x, y, theta)
        self.current_odom_pose = (0.0, 0.0, 0.0) # Pose actual odometría (x, y, theta)
        self.previous_odom_pose = None  # Pose anterior para calcular diferencial

        # ROS2 publishers/subscribers
        map_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', map_qos_profile)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').get_parameter_value().string_value,
            self.odom_callback,
            10)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').get_parameter_value().string_value,
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data)

        self.get_logger().info("Python SLAM node with particle filter initialized.")
        self.map_publish_timer = self.create_timer(1.0, self.publish_map)

    def odom_callback(self, msg: Odometry):
        # Store odometry for motion update
        self.last_odom = msg

    def scan_callback(self, msg: LaserScan):
        if self.last_odom is None:
            return

        # 1. Motion update (sample motion model)
        odom = self.last_odom
        # Retrieve odom_pose from odom message - remember that orientation is a quaternion
        pose = odom.pose.pose
        orientation_q = pose.orientation
        orientation_euler = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        current_odom_pose = [pose.position.x, pose.position.y, orientation_euler[2]]

        # Calcular odometría diferencial (como indica la consigna)
        if self.previous_odom_pose is not None:
            delta_x = current_odom_pose[0] - self.previous_odom_pose[0]
            delta_y = current_odom_pose[1] - self.previous_odom_pose[1]
            delta_theta = self.angle_diff(current_odom_pose[2], self.previous_odom_pose[2])
            
            # Model the particles around the current pose usando movimiento diferencial
            for p in self.particles:
                # Aplicar movimiento diferencial con ruido
                p.x += delta_x + np.random.normal(0, self.motion_noise)
                p.y += delta_y + np.random.normal(0, self.motion_noise)
                p.theta += delta_theta + np.random.normal(0, self.rotation_noise)
                # Normalizar el ángulo
                p.theta = np.arctan2(np.sin(p.theta), np.cos(p.theta))
        else:
            # Primera iteración: inicializar partículas en la pose actual
            for p in self.particles:
                p.x = current_odom_pose[0] + np.random.normal(0, self.motion_noise)
                p.y = current_odom_pose[1] + np.random.normal(0, self.motion_noise)
                p.theta = current_odom_pose[2] + np.random.normal(0, self.rotation_noise)

        # Guardar la pose actual para la próxima iteración
        self.previous_odom_pose = current_odom_pose.copy()

        # 2. Measurement update (weight particles)
        weights = []
        for p in self.particles:
            weight = self.compute_weight(p, msg) # Compute weights for each particle
            weights.append(weight)

        # Normalize weights
        sum_weights = sum(weights)
        if sum_weights == 0:
            # Para evitar división por cero, asignar pesos iguales
            weights = [1.0 / len(weights) for _ in weights]
        else:
            weights = [w / sum_weights for w in weights]

        for i, p in enumerate(self.particles):
            p.weight = weights[i] # Resave weights

        # 3. Resample
        self.particles = self.resample_particles(self.particles)

        # 4. Use weighted mean of all particles for mapping and pose (update current_map_pose and current_odom_pose)
        # Calcular la media ponderada de las partículas
        x_mean = 0.0
        y_mean = 0.0
        cos_sum = 0.0
        sin_sum = 0.0
        total_weight = 0.0

        for p in self.particles:
            w = p.weight
            x_mean += p.x * w
            y_mean += p.y * w
            cos_sum += math.cos(p.theta) * w
            sin_sum += math.sin(p.theta) * w
            total_weight += w

        if total_weight > 0:
            x_mean /= total_weight
            y_mean /= total_weight
            mean_theta = math.atan2(sin_sum / total_weight, cos_sum / total_weight)
        else:
            # En caso de que todos tengan peso cero, usar la primera partícula
            x_mean = self.particles[0].x
            y_mean = self.particles[0].y
            mean_theta = self.particles[0].theta

        # Actualizar la pose estimada del mapa y odometría
        self.current_map_pose = (x_mean, y_mean, mean_theta)
        self.current_odom_pose = (x_mean, y_mean, mean_theta)

        # 5. Mapping (update map with best particle's pose)
        for p in self.particles:
            self.update_map(p, msg)

        # 6. Broadcast map->odom transform
        self.broadcast_map_to_odom()

    def compute_weight(self, particle, scan_msg):
        # Improved likelihood: use probability distribution for matching
        score = 0.0
        valid_readings = 0
        robot_x, robot_y, robot_theta = particle.x, particle.y, particle.theta
        
        for i, range_dist in enumerate(scan_msg.ranges):
            # Skip invalid readings
            if range_dist < scan_msg.range_min or range_dist > scan_msg.range_max or math.isnan(range_dist):
                continue

            # Compute the map coordinates of the endpoint: transform the scan into the map frame
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            # Coordenadas del endpoint en el frame del robot
            endpoint_x_robot = range_dist * math.cos(angle)
            endpoint_y_robot = range_dist * math.sin(angle)

            # Transformar a coordenadas del mapa
            endpoint_x_map = robot_x + math.cos(robot_theta) * endpoint_x_robot - math.sin(robot_theta) * endpoint_y_robot
            endpoint_y_map = robot_y + math.sin(robot_theta) * endpoint_x_robot + math.cos(robot_theta) * endpoint_y_robot

            # Convertir a celdas de mapa
            map_x = int((endpoint_x_map - self.map_origin_x) / self.resolution)
            map_y = int((endpoint_y_map - self.map_origin_y) / self.resolution)

            # Use particle.log_odds_map for scoring
            if 0 <= map_x < self.map_width_cells and 0 <= map_y < self.map_height_cells:
                # Convertir log-odds a probabilidad
                log_odds = particle.log_odds_map[map_y, map_x]
                prob = 1.0 / (1.0 + math.exp(-log_odds))
                
                # Usar probabilidad gaussiana para el score
                # Si el punto debería estar ocupado (range_dist válido), 
                # darle más peso si la celda tiene alta probabilidad de ocupación
                expected_occupied = 1.0 if range_dist < scan_msg.range_max else 0.0
                error = abs(prob - expected_occupied)
                score += math.exp(-error * error / (2 * 0.1 * 0.1))  # Gaussiana con sigma=0.1
                valid_readings += 1

        # Normalizar por número de lecturas válidas
        if valid_readings > 0:
            score /= valid_readings
        else:
            score = 1e-6  # Peso muy pequeño si no hay lecturas válidas
            
        return max(score, 1e-6)  # Evitar peso cero

    def resample_particles(self, particles):
        # Improved resampling with systematic resampling
        weights = np.array([p.weight for p in particles])
        
        # Check for degeneracy (effective sample size)
        weights_sum = np.sum(weights)
        if weights_sum == 0:
            # Si todos los pesos son cero, reinicializar uniformemente
            for p in particles:
                p.weight = 1.0 / len(particles)
            return particles
        
        # Normalizar los pesos
        weights = weights / weights_sum
        
        # Calcular el tamaño efectivo de muestra
        ess = 1.0 / np.sum(weights**2)
        threshold = len(particles) / 2.0  # Umbral para remuestreo
        
        if ess < threshold:
            # Hacer remuestreo sistemático
            new_particles = []
            cumsum = np.cumsum(weights)
            
            # Generar puntos de muestreo uniformes
            u = np.random.uniform(0, 1.0 / len(particles))
            
            for i in range(len(particles)):
                # Encontrar el índice correspondiente
                sample_point = u + i / len(particles)
                idx = np.searchsorted(cumsum, sample_point)
                idx = min(idx, len(particles) - 1)  # Asegurar que esté en rango
                
                # Crear copia de la partícula seleccionada
                original_particle = particles[idx]
                new_particle = Particle(
                    original_particle.x, 
                    original_particle.y, 
                    original_particle.theta,
                    1.0 / len(particles),  # Peso uniforme después del remuestreo
                    original_particle.log_odds_map.shape
                )
                # Copiar el mapa
                new_particle.log_odds_map = original_particle.log_odds_map.copy()
                new_particles.append(new_particle)
            
            return new_particles
        else:
            # No es necesario remuestrear, solo normalizar pesos
            for i, p in enumerate(particles):
                p.weight = weights[i]
            return particles

    def update_map(self, particle, scan_msg):
        robot_x, robot_y, robot_theta = particle.x, particle.y, particle.theta
        for i, range_dist in enumerate(scan_msg.ranges):
            is_hit = range_dist < scan_msg.range_max
            current_range = min(range_dist, scan_msg.range_max)
            if math.isnan(current_range) or current_range < scan_msg.range_min:
                continue

            # Update map: transform the scan into the map frame
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            # Coordenadas en el frame del robot
            endpoint_x_robot = current_range * math.cos(angle)
            endpoint_y_robot = current_range * math.sin(angle)

            # Transformar a coordenadas globales
            endpoint_x_map = robot_x + math.cos(robot_theta) * endpoint_x_robot - math.sin(robot_theta) * endpoint_y_robot
            endpoint_y_map = robot_y + math.sin(robot_theta) * endpoint_x_robot + math.cos(robot_theta) * endpoint_y_robot

            # Convertir a celdas de mapa
            x0 = int((robot_x - self.map_origin_x) / self.resolution)
            y0 = int((robot_y - self.map_origin_y) / self.resolution)
            x1 = int((endpoint_x_map - self.map_origin_x) / self.resolution)
            y1 = int((endpoint_y_map - self.map_origin_y) / self.resolution)

            # Use self.bresenham_line for free cells along the ray
            self.bresenham_line(particle, x0, y0, x1, y1)

            # Update particle.log_odds_map accordingly - mark endpoint as occupied if hit
            if is_hit and 0 <= x1 < self.map_width_cells and 0 <= y1 < self.map_height_cells:
                particle.log_odds_map[y1, x1] += self.log_odds_occupied
                particle.log_odds_map[y1, x1] = np.clip(particle.log_odds_map[y1, x1], self.log_odds_min, self.log_odds_max)

    def bresenham_line(self, particle, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        path_len = 0
        max_path_len = dx + dy
        while not (x0 == x1 and y0 == y1) and path_len < max_path_len:
            if 0 <= x0 < self.map_width_cells and 0 <= y0 < self.map_height_cells:
                particle.log_odds_map[y0, x0] += self.log_odds_free
                particle.log_odds_map[y0, x0] = np.clip(particle.log_odds_map[y0, x0], self.log_odds_min, self.log_odds_max)
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
            path_len += 1

    def publish_map(self):
        # Fill in map_msg fields and publish the occupancy grid map
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = self.get_parameter('map_frame').get_parameter_value().string_value

        # Configurar resolución, tamaño y origen
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.map_width_cells
        map_msg.info.height = self.map_height_cells
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.orientation.w = 1.0

        # Promediar los mapas de las partículas (weighted average)
        combined_map = np.zeros((self.map_height_cells, self.map_width_cells))
        total_weight = sum(p.weight for p in self.particles)
        
        if total_weight > 0:
            for p in self.particles:
                combined_map += p.log_odds_map * p.weight
            combined_map /= total_weight
        else:
            # Si no hay pesos válidos, usar promedio simple
            for p in self.particles:
                combined_map += p.log_odds_map
            combined_map /= len(self.particles)

        # Convertir log-odds a valores de ocupación ROS (0-100 scale)
        # 0 = libre, 100 = ocupado, -1 = desconocido
        map_data = []
        for y in range(self.map_height_cells):
            for x in range(self.map_width_cells):
                log_odds = np.clip(combined_map[y, x], self.log_odds_min, self.log_odds_max)
                
                if abs(log_odds) < 0.5:  # Umbral para "desconocido"
                    occupancy = -1  # Desconocido
                else:
                    # Convertir log-odds a probabilidad, luego a escala 0-100
                    prob = 1.0 / (1.0 + math.exp(-log_odds))
                    occupancy = int(prob * 100)
                
                map_data.append(occupancy)

        # Asignar datos y publicar
        map_msg.data = map_data
        self.map_publisher.publish(map_msg)
        self.get_logger().debug("Map published.")
    

    def broadcast_map_to_odom(self):
        # Broadcast map->odom transform using the estimated pose
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('map_frame').get_parameter_value().string_value
        t.child_frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value

        # Usar la pose estimada del mapa
        x, y, yaw = self.current_map_pose

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # Convertir yaw a quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Enviar la transformación
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def angle_diff(a, b):
        d = a - b
        while d > np.pi:
            d -= 2 * np.pi
        while d < -np.pi:
            d += 2 * np.pi
        return d

def main(args=None):
    rclpy.init(args=args)
    node = PythonSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()