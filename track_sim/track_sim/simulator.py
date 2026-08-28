import math
import time

import pygame
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty

from track_sim.level_loader import load_level
from track_sim.track import Track


# Vehicle and sensor constants are intentionally kept together for the laboratory.
MAX_LINEAR_SPEED = 2.0
MAX_ANGULAR_SPEED = 2.0
VEHICLE_RADIUS = 0.20
LIDAR_FOV = math.pi
LIDAR_RAYS = 61
LIDAR_RANGE_MIN = 0.05
LIDAR_RANGE_MAX = 8.0
LIDAR_RATE = 10.0
ODOM_RATE = 20.0

SCREEN_WIDTH = 1100
SCREEN_HEIGHT = 700
TOP_PANEL_HEIGHT = 78
VIEW_MARGIN = 35


class TrackSimulator(Node):
    def __init__(self):
        super().__init__('track_sim')

        self.declare_parameter('level', 'level1')
        self.declare_parameter('keyboard_control', False)
        self.declare_parameter('show_lidar', True)

        level_name = self.get_parameter('level').value
        self.keyboard_control = self.get_parameter('keyboard_control').value
        self.show_lidar = self.get_parameter('show_lidar').value

        self.level = load_level(level_name)
        self.track = Track(self.level['points'])

        self.cmd_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.goal_publisher = self.create_publisher(Bool, '/goal_reached', 10)
        self.reset_service = self.create_service(Empty, '/reset', self.reset_callback)

        pygame.init()
        pygame.display.set_caption(f'ROS 2 Track Simulator - {level_name}')
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.font = pygame.font.Font(None, 29)
        self.small_font = pygame.font.Font(None, 23)
        self.large_font = pygame.font.Font(None, 58)

        self.running = True
        self.last_scan_publish = 0.0
        self.last_odom_publish = 0.0
        self.scan_ranges = [LIDAR_RANGE_MAX] * LIDAR_RAYS
        self.path = []
        self._configure_view()
        self.reset()

        mode = 'teclado' if self.keyboard_control else '/cmd_vel'
        self.get_logger().info(f'Nivel {level_name} cargado. Control: {mode}')

    def _configure_view(self):
        min_x, max_x, min_y, max_y = self.track.bounds
        world_width = max(max_x - min_x, 1.0)
        world_height = max(max_y - min_y, 1.0)
        available_width = SCREEN_WIDTH - 2 * VIEW_MARGIN
        available_height = SCREEN_HEIGHT - TOP_PANEL_HEIGHT - 2 * VIEW_MARGIN
        self.pixels_per_meter = min(available_width / world_width, available_height / world_height)

        drawn_width = world_width * self.pixels_per_meter
        drawn_height = world_height * self.pixels_per_meter
        self.view_left = (SCREEN_WIDTH - drawn_width) / 2.0
        self.view_top = TOP_PANEL_HEIGHT + (SCREEN_HEIGHT - TOP_PANEL_HEIGHT - drawn_height) / 2.0
        self.world_min_x = min_x
        self.world_max_y = max_y

    def world_to_screen(self, x, y):
        """Convert world coordinates in meters to pygame pixel coordinates."""
        pixel_x = self.view_left + (x - self.world_min_x) * self.pixels_per_meter
        pixel_y = self.view_top + (self.world_max_y - y) * self.pixels_per_meter
        return int(pixel_x), int(pixel_y)

    def cmd_callback(self, msg):
        if self.keyboard_control:
            return
        self.linear_speed = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, msg.linear.x))
        self.angular_speed = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, msg.angular.z))

    def reset_callback(self, request, response):
        del request
        self.reset()
        return response

    def reset(self):
        self.x = self.level['start_x']
        self.y = self.level['start_y']
        self.yaw = self.level['start_yaw']
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.status = 'RUNNING'
        self.path = [(self.x, self.y)]
        self.scan_ranges = self.calculate_scan()
        self.get_logger().info('Nivel reiniciado')

    def process_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_r:
                    self.reset()

    def update(self, delta_time):
        if self.keyboard_control:
            self._read_keyboard()
        if self.status != 'RUNNING':
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            return

        new_yaw = self.yaw + self.angular_speed * delta_time
        new_x = self.x + self.linear_speed * math.cos(new_yaw) * delta_time
        new_y = self.y + self.linear_speed * math.sin(new_yaw) * delta_time

        if not self.track.contains_vehicle(new_x, new_y, VEHICLE_RADIUS):
            self.status = 'COLLISION'
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            self.get_logger().warn('Vehiculo fuera de la pista')
            return

        self.x = new_x
        self.y = new_y
        self.yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))

        if math.hypot(self.x - self.level['goal_x'], self.y - self.level['goal_y']) <= self.level['goal_radius']:
            self.status = 'GOAL REACHED'
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            self.get_logger().info('Meta alcanzada')

        if math.hypot(self.x - self.path[-1][0], self.y - self.path[-1][1]) >= 0.05:
            self.path.append((self.x, self.y))

    def _read_keyboard(self):
        keys = pygame.key.get_pressed()
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        if keys[pygame.K_w] or keys[pygame.K_UP]:
            self.linear_speed = MAX_LINEAR_SPEED * 0.65
        if keys[pygame.K_s] or keys[pygame.K_DOWN]:
            self.linear_speed = -MAX_LINEAR_SPEED * 0.45
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            self.angular_speed = MAX_ANGULAR_SPEED * 0.75
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            self.angular_speed = -MAX_ANGULAR_SPEED * 0.75

    def calculate_scan(self):
        angle_increment = LIDAR_FOV / (LIDAR_RAYS - 1)
        return [
            self.track.cast_ray(
                self.x,
                self.y,
                self.yaw - LIDAR_FOV / 2.0 + index * angle_increment,
                LIDAR_RANGE_MAX,
            )
            for index in range(LIDAR_RAYS)
        ]

    def publish_messages(self):
        now = time.monotonic()
        if now - self.last_odom_publish >= 1.0 / ODOM_RATE:
            self._publish_odom()
            self.last_odom_publish = now
        if now - self.last_scan_publish >= 1.0 / LIDAR_RATE:
            self.scan_ranges = self.calculate_scan()
            self._publish_scan()
            self.goal_publisher.publish(Bool(data=self.status == 'GOAL REACHED'))
            self.last_scan_publish = now

    def _publish_odom(self):
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        message.twist.twist.linear.x = self.linear_speed
        message.twist.twist.angular.z = self.angular_speed
        self.odom_publisher.publish(message)

    def _publish_scan(self):
        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.angle_min = -LIDAR_FOV / 2.0
        message.angle_max = LIDAR_FOV / 2.0
        message.angle_increment = LIDAR_FOV / (LIDAR_RAYS - 1)
        message.scan_time = 1.0 / LIDAR_RATE
        message.time_increment = message.scan_time / LIDAR_RAYS
        message.range_min = LIDAR_RANGE_MIN
        message.range_max = LIDAR_RANGE_MAX
        message.ranges = self.scan_ranges
        self.scan_publisher.publish(message)

    def draw(self):
        self.screen.fill((24, 31, 38))
        self._draw_track()
        self._draw_path()
        self._draw_markers()
        if self.show_lidar:
            self._draw_lidar()
        self._draw_vehicle()
        self._draw_panel()
        self._draw_result()
        pygame.display.flip()

    def _draw_track(self):
        left = [self.world_to_screen(*point) for point in self.track.left_boundary]
        right = [self.world_to_screen(*point) for point in self.track.right_boundary]
        pygame.draw.polygon(self.screen, (82, 89, 94), left + list(reversed(right)))
        pygame.draw.lines(self.screen, (232, 232, 225), False, left, 3)
        pygame.draw.lines(self.screen, (232, 232, 225), False, right, 3)
        pygame.draw.line(self.screen, (232, 232, 225), left[0], right[0], 3)
        pygame.draw.line(self.screen, (232, 232, 225), left[-1], right[-1], 3)

    def _draw_path(self):
        if len(self.path) > 1:
            points = [self.world_to_screen(x, y) for x, y in self.path]
            pygame.draw.lines(self.screen, (245, 190, 70), False, points, 2)

    def _draw_markers(self):
        start = self.world_to_screen(self.level['start_x'], self.level['start_y'])
        pygame.draw.circle(self.screen, (75, 190, 115), start, 12, 3)
        self.screen.blit(self.small_font.render('START', True, (160, 245, 185)), (start[0] + 14, start[1] - 10))

        goal = self.world_to_screen(self.level['goal_x'], self.level['goal_y'])
        radius = max(8, int(self.level['goal_radius'] * self.pixels_per_meter))
        pygame.draw.circle(self.screen, (245, 195, 55), goal, radius, 3)
        self.screen.blit(self.small_font.render('GOAL', True, (255, 225, 115)), (goal[0] - 24, goal[1] - radius - 23))

    def _draw_lidar(self):
        origin = self.world_to_screen(self.x, self.y)
        angle_increment = LIDAR_FOV / (LIDAR_RAYS - 1)
        for index, distance in enumerate(self.scan_ranges):
            angle = self.yaw - LIDAR_FOV / 2.0 + index * angle_increment
            end = self.world_to_screen(
                self.x + distance * math.cos(angle),
                self.y + distance * math.sin(angle),
            )
            pygame.draw.line(self.screen, (72, 145, 125), origin, end, 1)

    def _draw_vehicle(self):
        half_length = 0.30
        half_width = 0.17
        local_points = [
            (half_length, 0.0),
            (half_length * 0.55, half_width),
            (-half_length, half_width),
            (-half_length, -half_width),
            (half_length * 0.55, -half_width),
        ]
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        points = []
        for local_x, local_y in local_points:
            world_x = self.x + local_x * cosine - local_y * sine
            world_y = self.y + local_x * sine + local_y * cosine
            points.append(self.world_to_screen(world_x, world_y))
        pygame.draw.polygon(self.screen, (65, 145, 225), points)
        pygame.draw.polygon(self.screen, (190, 225, 255), points, 2)

    def _draw_panel(self):
        pygame.draw.rect(self.screen, (14, 19, 24), (0, 0, SCREEN_WIDTH, TOP_PANEL_HEIGHT))
        level_text = self.font.render(f'Nivel: {self.level["name"]}', True, (235, 239, 242))
        mode = 'TECLADO' if self.keyboard_control else '/cmd_vel'
        state_color = (235, 92, 92) if self.status == 'COLLISION' else (90, 220, 135)
        state_text = self.font.render(f'Estado: {self.status}', True, state_color)
        speed_text = self.small_font.render(
            f'v = {self.linear_speed:+.2f} m/s    w = {self.angular_speed:+.2f} rad/s    control: {mode}',
            True,
            (175, 186, 195),
        )
        help_text = self.small_font.render('R: reiniciar    ESC: salir', True, (175, 186, 195))
        self.screen.blit(level_text, (22, 13))
        self.screen.blit(state_text, (245, 13))
        self.screen.blit(speed_text, (22, 47))
        self.screen.blit(help_text, (850, 47))

    def _draw_result(self):
        if self.status == 'RUNNING':
            return
        overlay = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 105))
        self.screen.blit(overlay, (0, 0))
        color = (100, 235, 145) if self.status == 'GOAL REACHED' else (255, 105, 95)
        text = self.large_font.render(self.status, True, color)
        prompt = self.font.render('Presiona R para reiniciar', True, (245, 245, 245))
        self.screen.blit(text, text.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 - 25)))
        self.screen.blit(prompt, prompt.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 + 28)))


def main(args=None):
    rclpy.init(args=args)
    simulator = None
    try:
        simulator = TrackSimulator()
        clock = pygame.time.Clock()
        while rclpy.ok() and simulator.running:
            delta_time = min(clock.tick(60) / 1000.0, 0.1)
            rclpy.spin_once(simulator, timeout_sec=0.0)
            simulator.process_events()
            simulator.update(delta_time)
            simulator.publish_messages()
            simulator.draw()
    except KeyboardInterrupt:
        pass
    finally:
        if simulator is not None:
            simulator.destroy_node()
        pygame.quit()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
