import time
from dronekit import connect, VehicleMode, mavutil
import threading
import rclpy
from dronekit import LocationGlobalRelative
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN
from math import radians, sin, atan2, sqrt, cos

obstacle_detected = False
TARGET_LAT = -35.36233764 
TARGET_LON = 149.16487783

class APFObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('apf_obstacle_avoidance')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, '/clustered_centres', 10)
        self.detection_distance = 1.0  
        self.influence_radius = 2.0 
        self.k_attractive = 1.0  
        self.k_repulsive = 5.0  
        self.obstacle_detected = False
        self.closest_obstacles = []
        self.get_logger().info("APFObstacleAvoidance node initialized.")

    def filter_points_by_radius(self, points):
        return [
            point for point in points 
            if 0.3 < point[1] < 2.0 and np.linalg.norm([-point[0], point[2]]) < self.influence_radius
        ]
    
    def pointcloud_callback(self, msg):
        global obstacle_detected
        points = np.array([
            (-point[0], point[1], point[2]) 
            for point in pc2.read_points(msg, field_names=("y", "z", "x"), skip_nans=True)
        ])

        if points.shape[0] > 1000:
            indices = np.random.choice(points.shape[0], size=1000, replace=False)
            points = points[indices]
        self.closest_obstacles = self.filter_points_by_radius(points)

        if len(self.closest_obstacles) >0:
            db = DBSCAN(eps=0.2, min_samples=10)
            labels = db.fit_predict(self.closest_obstacles)
            unique_labels = set(labels) - {-1}

            self.cluster_centers = []

            for label in unique_labels:
                cluster_points = self.closest_obstacles[labels == label, :]
                n_points = len(cluster_points)
                centroid = np.mean(cluster_points, axis = 0)
                self.cluster_centers.append(centroid)

        else:
            self.cluster_centers= []
        self.cluster_centers = self.cluster_centers    

        header = msg.header
        header.frame_id = "camera_link"
        pointcloud_msg = pc2.create_cloud_xyz32(header, self.cluster_centres)
        self.publisher.publish(pointcloud_msg)

        dis = np.sqrt(self.cluster_centers[0]**2 + self.cluster_centers[1]**2)
        if dis < self.influence_radius and dis >0.5:
            print("detected.!!")
            obstacle_detected = True
            
        # for point in self.closest_obstacles:
        #     n_pointy, n_pointz, n_pointx = point
        #     # print(f"x:{n_pointx},y:{n_pointy},z:{n_pointz}")
        #     distance = np.sqrt( n_pointx**2 + n_pointy**2 )
        #     # print(distance)
        #     if n_pointz > 0.2:
        #         if distance < self.influence_radius and distance > 0.5:
        #             self.closest_obstacles.append((n_pointx, n_pointy))
        #             # print("obs_values")
        #         if distance < self.detection_distance and distance > 0.5:
        #             print("detected.!!!")
        #             obstacle_detected = True
    
    # def pointcloud_callback(self, msg):
    #     global obstacle_detected
    #     points = np.array([
    #         (-point[0], point[1], point[2]) 
    #         for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    #     ])
    #     if points.shape[0] > 1000:
    #         indices = np.random.choice(points.shape[0], size=1000, replace=False)
    #         points = points[indices]

    #     self.closest_obstacles = self.filter_points_by_radius(points)
    #     obstacle_detected = any(
    #         0.4 < sqrt(point[0]**2 + point[2]**2) < self.detection_distance for point in self.closest_obstacles
    #     )

    def calculate_potential_forces(self, robot_position, target_position):
        dx = target_position[0] - robot_position[0]
        dy = target_position[1] - robot_position[1]
        d_goal = sqrt(dx**2 + dy**2)

        f_att_x = self.k_attractive * dx / d_goal if d_goal > 0 else 0
        f_att_y = self.k_attractive * dy / d_goal if d_goal > 0 else 0     

        f_rep_x = f_rep_y = 0.0  

        for obs in self.closest_obstacles:
            ox, oy = obs[0], obs[2]
            # print(f"ox:{ox},oy:{oy}")
            distance = sqrt(ox**2 + oy**2)
            print(f"dis_obs:{distance}")
            if 0.6 < distance < self.influence_radius:
                m = self.k_repulsive
                f_rep_x += (m * ox / distance)
                f_rep_y += (m * oy / distance)

        return f_att_x - f_rep_x, f_att_y - f_rep_y

def connect_to_vehicle():
    print("Connecting to vehicle...")
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
    print("Connected to vehicle.")
    return vehicle

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, 100
    )
    for _ in range(duration):
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(0.5)

def get_distance(lat1, lon1, lat2, lon2):
    R = 6371000  
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat / 2) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

def goto_gps_location(vehicle, lat, lon):
    print(f"Moving to Target: {lat}, {lon}")
    target_location = LocationGlobalRelative(lat, lon, 0)
    vehicle.simple_goto(target_location)

def run_ros2_node():
    rclpy.init()
    global apf_node
    apf_node = APFObstacleAvoidance()
    try:
        rclpy.spin(apf_node)
    except KeyboardInterrupt:
        apf_node.get_logger().info("Shutting down APFObstacleAvoidance node.")
    finally:
        apf_node.destroy_node()
        rclpy.shutdown()

def main():
    global obstacle_detected
    ros2_thread = threading.Thread(target=run_ros2_node, daemon=True)
    ros2_thread.start()

    vehicle = connect_to_vehicle()
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(2)

    target_position = (TARGET_LAT, TARGET_LON)

    try:
        while True:
            print("Checking for obstacles...")

            current_heading = vehicle.heading
            current_location = vehicle.location.global_relative_frame
            robot_position = (current_location.lat, current_location.lon)

            # print(f"Current Heading: {current_heading:.2f}°")
            # print(f"Current Position: {robot_position}")

            current_distance = get_distance(current_location.lat, current_location.lon, TARGET_LAT, TARGET_LON)
            # print(f"Distance to Target: {current_distance:.2f} meters")

            if current_distance <= 2.0:
                print("within 2m RTL...")
                vehicle.mode = VehicleMode('RTL')
                break 

            if obstacle_detected:
                print("Obstacle detected! Avoiding...")
                force_x, force_y = apf_node.calculate_potential_forces(robot_position, target_position)
                print(f"force_x:{force_x},force_y:{force_y}")
                total_force = sqrt(force_x**2 + force_y**2)
                if total_force > 0:
                    angle_degrees = atan2(force_y, force_x)
                    vel = 1.0
                    velocity_x = vel * cos(angle_degrees)
                    velocity_y = vel * sin(angle_degrees)

                    send_ned_velocity(vehicle, velocity_x, velocity_y, 0, 1)
                    print(f"Avoiding: velocities ({velocity_x}, {velocity_y})")
                    print("-----------------------------------------")
                    obstacle_detected = False
                else:
                    send_ned_velocity(vehicle, 0, 0, 0, 1) 
                    obstacle_detected = False

            else:
                print("No obstacle.target...")
                print("-----------------------------------------")
                goto_gps_location(vehicle,TARGET_LAT ,TARGET_LON)

            time.sleep(1)
    except KeyboardInterrupt:
        print("Script interrupted by user.")
    finally:
        print("Setting vehicle mode to RTL...")
        vehicle.mode = VehicleMode('RTL')
        time.sleep(5)
        print("Returning to Launch (RTL)...")
        vehicle.close()
        print("Vehicle connection closed.")

if __name__ == '__main__':
    main()


