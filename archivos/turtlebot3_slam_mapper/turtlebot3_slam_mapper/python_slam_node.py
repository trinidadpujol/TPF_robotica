#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
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
        self.declare_parameter('map_resolution', 0.05)  # Por ejemplo, 0.05 metros/celda
        self.declare_parameter('map_width_meters', 10.0)
        self.declare_parameter('map_height_meters', 10.0)
        self.declare_parameter('num_particles', 100)

        self.resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_width_m = self.get_parameter('map_width_meters').get_parameter_value().double_value
        self.map_height_m = self.get_parameter('map_height_meters').get_parameter_value().double_value
        self.map_width_cells = int(self.map_width_m / self.resolution)
        self.map_height_cells = int(self.map_height_m / self.resolution)
        self.map_origin_x = -self.map_width_m / 2.0
        self.map_origin_y = -5.0

        # TODO: define the log-odds criteria for free and occupied cells
        self.log_odds_free = -1.0  # Valor log-odds para celdas libres
        self.log_odds_occupied = 1.0  # Valor log-odds para celdas ocupadas

        self.log_odds_max = 5.0
        self.log_odds_min = -5.0

        # Particle filter
        self.num_particles = self.get_parameter('num_particles').get_parameter_value().integer_value
        self.particles = [Particle(0.0, 0.0, 0.0, 1.0/self.num_particles, (self.map_height_cells, self.map_width_cells)) for _ in range(self.num_particles)]
        self.last_odom = None

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
        # TODO: Retrieve odom_pose from odom message - remember that orientation is a quaternion

        pose = odom.pose.pose
        orientation_q = pose.orientation
        orientation_euler = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        odom_pose = [pose.position.x, pose.position.y, orientation_euler[2]]

        # TODO: Model the particles around the current pose
        for p in self.particles:
            update_x = odom_pose[0] + np.random.normal(0, self.motion_noise)
            update_y = odom_pose[1] + np.random.normal(0, self.motion_noise)
            update_yaw = odom_pose[2] + np.random.normal(0, self.rotation_noise)
            p.x = update_x
            p.y = update_y
            p.theta = update_yaw

        # TODO: 2. Measurement update (weight particles)
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

        # TODO: 4. Use weighted mean of all particles for mapping and pose (update current_map_pose and current_odom_pose, for each particle)

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

        # Actualizar la pose estimada del mapa
        self.current_map_pose = (x_mean, y_mean, mean_theta)

        # Además, guardar la pose de odometría para transformaciones
        # La pose de odometría en el frame base
        self.current_odom_pose = (x_mean, y_mean, mean_theta)

        # 5. Mapping (update map with best particle's pose)
        for p in self.particles:
            self.update_map(p, msg)

        # 6. Broadcast map->odom transform
        self.broadcast_map_to_odom()

    def compute_weight(self, particle, scan_msg):
        # Simple likelihood: count how many endpoints match occupied cells
        score = 0.0
        robot_x, robot_y, robot_theta = particle.x, particle.y, particle.theta
        for i, range_dist in enumerate(scan_msg.ranges):
            if range_dist < scan_msg.range_min or range_dist > scan_msg.range_max or math.isnan(range_dist):
                continue

            # TODO: Compute the map coordinates of the endpoint: transform the scan into the map frame
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

            # TODO: Use particle.log_odds_map for scoring
            if 0 <= map_x < self.map_width_cells and 0 <= map_y < self.map_height_cells:
            # Acumular el log-odds
                score += particle.log_odds_map[map_y, map_x]

        return score + 1e-6

    def resample_particles(self, particles):
        # TODO: Resample particles
        # Extraer los pesos de las partículas
        weights = np.array([p.weight for p in particles])
        # Normalizar los pesos (por si no están normalizados)
        weights /= np.sum(weights)
    
        # Selección de índices con probabilidad de acuerdo a los pesos
        indices = np.random.choice(len(particles), size=len(particles), p=weights)
        
        # Crear una nueva lista de partículas resampleadas
        new_particles = [particles[i] for i in indices]
        
        # Opcional: reinicializar los pesos a valor uniforme
        for p in new_particles:
            p.weight = 1.0 / len(new_particles)

        return new_particles

    def update_map(self, particle, scan_msg):
        robot_x, robot_y, robot_theta = particle.x, particle.y, particle.theta
        for i, range_dist in enumerate(scan_msg.ranges):
            is_hit = range_dist < scan_msg.range_max
            current_range = min(range_dist, scan_msg.range_max)
            if math.isnan(current_range) or current_range < scan_msg.range_min:
                continue

            # TODO: Update map: transform the scan into the map frame
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

            # TODO: Use self.bresenham_line for free cells
            self.bresenham_line(particle, x0, y0, x1, y1)

            # TODO: Update particle.log_odds_map accordingly
            # Marcar la celda de impacto como ocupada
            if 0 <= x1 < self.map_width_cells and 0 <= y1 < self.map_height_cells:
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
        # TODO: Fill in map_msg fields and publish one map
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

        # Promediar los mapas de las partículas
        prob_map = np.zeros((self.map_height_cells, self.map_width_cells))
        for p in self.particles:
            prob_map += np.clip(p.log_odds_map, self.log_odds_min, self.log_odds_max)
        prob_map /= len(self.particles)

        # Convertir a valores de ocupación (0 = ocupado, 100 = libre)
        map_data = []
        for y in range(self.map_height_cells):
            for x in range(self.map_width_cells):
                prob = prob_map[y, x]
                occupancy = int(np.clip((prob / self.log_odds_max) * 100, 0, 100))
                map_data.append(100 - occupancy)  # Ajusta si quieres al revés

        # Asignar datos y publicar
        map_msg.data = map_data
        self.map_publisher.publish(map_msg)
        self.get_logger().debug("Map published.")
    

    def broadcast_map_to_odom(self):
        # TODO: Broadcast map->odom transform
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