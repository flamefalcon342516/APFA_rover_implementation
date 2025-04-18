#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import read_points
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from math import sqrt,cos,sin,radians,atan2,degrees
from dronekit import connect,mavutil,VehicleMode,Vehicle
import time
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
#import pygame
import struct

class APF_Node(Node):
    def __init__(self):
        super().__init__('mazerunner2')

        self.pointcloud_sub = self.create_subscription(PointCloud2,'/camera/camera/depth/color/points',self.pointcloud_callback,10)
        self.publisher1 = self.create_publisher(PointCloud2, '/filtered_pointcloud', 10)
        self.publisher2 = self.create_publisher(PointCloud2,'/clustered_centroids',10)
        self.publisher3 = self.create_publisher(PointCloud2,'/points',10)
        self.publisher4 = self.create_publisher(PointCloud2,'/closest_cluster',10)
        self.k_att = 10.0
        self.cluster_centres = []
        self.cluster_sizes = []
        self.closest_centre = []

    #@profile
    def pointcloud_callback(self, msg):
        fields = ('x','y','z')
        #self.points = np.array([(p[0], p[1], p[2]) for p in read_points(msg, field_names=fields, skip_nans=True)])
        # self.points = np.fromiter(read_points(msg, field_names=fields, skip_nans=True), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        # self.points = np.vstack((self.points['x'], self.points['y'], self.points['z'])).T

        #print(self.points.shape)
        self.points = pc2.read_points(msg,field_names=fields,skip_nans=True)
        self.points = np.vstack((self.points['x'], self.points['y'], self.points['z'])).T # Extract only the first three columns (x, y, z)
        #self.points = self.points.reshape(-1,3)
        print(self.points.shape)

        self.filtered_points = self.points[(0.3 > self.points[:, 1]) & (self.points[:, 1] > -1.0) & (0 < self.points[:, 2]) & (self.points[:, 2] < 1.5)]

        num_samples = self.filtered_points.shape[0] // 100  # Adjust fraction as needed
        random_indices = np.random.choice(self.filtered_points.shape[0], num_samples, replace=False)
        self.filtered_points = self.filtered_points[random_indices]

        print(self.filtered_points.shape)

        if len(self.filtered_points) > 0:
            db = DBSCAN(eps=0.3, min_samples=5, algorithm='ball_tree', n_jobs=-1)
            labels = db.fit_predict(self.filtered_points)
            unique_labels = set(labels) - {-1}

            self.cluster_centres = []
            self.cluster_sizes = []
            for label in unique_labels:
                cluster_points = self.filtered_points[labels == label]
                centroid = np.mean(cluster_points, axis=0)
                self.cluster_centres.append(centroid)
                self.cluster_sizes.append(len(cluster_points))
            if self.cluster_centres:
                closest_index = min(range(len(self.cluster_centres)), key=lambda i: self.cluster_centres[i][2])
                self.closest_centre = self.cluster_centres[closest_index]  # Store the closest cluster    
            else:
                self.closest_centre = None
        else:
            self.cluster_centres = []
            self.cluster_sizes = []
            self.closest_centre = None

        # Publish the filtered PointCloud2
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link"
        pointcloud_msg = pc2.create_cloud_xyz32(header, self.filtered_points)
        cluster_msg = pc2.create_cloud_xyz32(header,self.cluster_centres)
        points_msg = pc2.create_cloud_xyz32(header,self.points)
        if self.closest_centre is not None:
            closest_cluster_msg = pc2.create_cloud_xyz32(header,[self.closest_centre])
            self.publisher4.publish(closest_cluster_msg)
        self.publisher1.publish(pointcloud_msg)
        self.publisher2.publish(cluster_msg)
        self.publisher3.publish(points_msg)
        #self.get_logger().info("Published filtered PointCloud2")

#@profile
def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371e3
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1
    a = (sin(delta_lat/2)**2 +
         cos(lat1) * cos(lat2) * sin(delta_lon/2)**2)
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c
#@profile
def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    delta_lon = lon2 - lon1
    x = sin(delta_lon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon)
    bearing = atan2(x, y)
    return (degrees(bearing) + 360) % 360
#@profile
def send_ned_velocity(vehicle, vx, vy, vz,duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

    for _ in range(duration * 5):
        vehicle.send_mavlink(msg)
        time.sleep(0.02)
#@profile
# def Compute_Repulsive_force(point,point_size,k_repulsion):
#     k_rep = k_repulsion
#     robot_pos = np.array([0.0, 0.0, 0.0])
#     net_repulsive_force = np.array([0.0, 0.0, 0.0])

#     if len(point) == 0:
#         return 0.0, 0.0  # No obstacles, no repulsive force

#     point = np.array(point)  # Convert list of NumPy arrays into a single NumPy array
#     point_size = np.array(point_size)
#     distances = np.abs(point[:, 2])  # Now this slicing works correctly

#     # Avoid division by zero
#     distances[distances == 0] = 1e-6
#     #print(f"point_size: {point_size}, Type: {type(point_size)}")
#     #scaling_factor = float(np.log(point_size + 1))
#     scaling_factor = np.log(point_size[:, np.newaxis] + 1)

#     repulsive_potential = k_rep * scaling_factor / distances[:, np.newaxis] 
#     repulsive_direction = (robot_pos - point) / distances[:, np.newaxis] 
#     repulsive_force = repulsive_potential * repulsive_direction

#     net_repulsive_force = np.sum(repulsive_force, axis=0)

#     return net_repulsive_force[0], net_repulsive_force[1]

def Compute_Repulsive_force(point, point_size, k_repulsion):
    k_rep = k_repulsion
    robot_pos = np.array([0.0, 0.0, 0.0])

    if len(point) == 0:
        return 0.0, 0.0  # No obstacles, no repulsive force

    point = np.array(point)  
    point_size = np.array(point_size)

    # Consider only clusters in front (z > 0)
    front_indices = np.where(point[:, 2] > 0)[0]
    
    if len(front_indices) == 0:
        return 0.0, 0.0  # No obstacles in front, no repulsive force

    # Find the closest cluster in front (minimum z-depth)
    min_z_index = front_indices[np.argmin(point[front_indices, 2])]
    closest_cluster = point[min_z_index]
    closest_size = point_size[min_z_index]

    # Compute repulsive force from the closest cluster
    distance = np.abs(closest_cluster[2])
    distance = max(distance, 1e-6)  # Avoid division by zero
    scaling_factor = np.log(closest_size + 1)

    repulsive_potential = k_rep * scaling_factor / distance
    repulsive_direction = (robot_pos - closest_cluster) / distance
    repulsive_force = repulsive_potential * repulsive_direction

    return repulsive_force[0], repulsive_force[1]

def main():
    rclpy.init()
    node = APF_Node()

    # vehicle = connect('127.0.0.1:14550', wait_ready=True)
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
    vehicle.armed = True
    while not vehicle.armed:
        print("waiting for vehicle to arm..")
        time.sleep(1)
    print("Vehicle is armed")
    vehicle.mode = VehicleMode('GUIDED')

    #goal_lat, goal_lon = -35.36215058, 149.16507715
    goal_lat, goal_lon = 28.74925198,  77.11593573 
    #for DTU
    #goal_lat,goal_lon =  +35.34766723 , 149.16787988
    try:
        while rclpy.ok():
            start_time = time.time()
            current_location = vehicle.location.global_relative_frame
            dist_to_goal = haversine_distance(
                current_location.lat, current_location.lon, goal_lat, goal_lon)
            if dist_to_goal < 1:
                node.get_logger().info("Goal reached.")
                break
            else:
                node.get_logger().info(f"distance to goal : {dist_to_goal}")
            
            bearing = calculate_bearing(current_location.lat, current_location.lon, goal_lat, goal_lon)

            dx = dist_to_goal * cos(radians(bearing))
            dy = dist_to_goal * sin(radians(bearing))

            f_att_x = node.k_att * dx / dist_to_goal if dist_to_goal > 0 else 0
            f_att_y = node.k_att * dy / dist_to_goal if dist_to_goal > 0 else 0
            yaw = vehicle.attitude.yaw
            # f_att_x = f_att_x_univ * cos(yaw) + f_att_y_univ * sin(yaw)
            # f_att_y = -f_att_x_univ * sin(yaw) + f_att_y_univ * cos(yaw)
            # f_att_x = f_att_x_univ * sin(yaw) + f_att_y_univ * cos(yaw)
            # f_att_y = -f_att_x_univ * cos(yaw) + f_att_y_univ * sin(yaw)

            f_rep_x,f_rep_y = Compute_Repulsive_force(node.cluster_centres,node.cluster_sizes,k_repulsion=4.0)
            node.get_logger().info(f"f_att_x : {f_att_x},f_att_y : {f_att_y},f_rep_x : {f_rep_x},f_rep_y : {f_rep_y}")

            Vel_x = f_att_x + f_rep_x
            Vel_y = f_att_y - f_rep_y

            if Vel_y < 0:
                movement_intent = "Moving right to avoid obstacles."
            elif Vel_y > 0:
                movement_intent = "Moving left to avoid obstacles."
            else:
                movement_intent = "Moving straight ahead."

            # **Intent Message Based on Closest Cluster Position**
            if node.closest_centre is not None and len(node.closest_centre) == 3:
                cluster_x, cluster_y, cluster_z = node.closest_centre  # Extract closest cluster coordinates
                if cluster_x < 0:
                    cluster_intent = "Detected closest cluster on the left."
                elif cluster_x > 0:
                    cluster_intent = "Detected closest cluster on the right."
                else:
                    cluster_intent = "Detected cluster directly in front."
            else:
                cluster_intent = "No obstacles detected."
            
            node.get_logger().info(f"Velocity X: {Vel_x:.2f}, Velocity Y: {Vel_y:.2f}")
            node.get_logger().info(f"{cluster_intent} {movement_intent}")
            send_ned_velocity(vehicle, Vel_x, Vel_y, 0 , 1)
            rclpy.spin_once(node, timeout_sec=0.01)
            loop_time = time.time() - start_time
            node.get_logger().info(f"Loop time: {loop_time:.3f}s")

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")

    finally:
        node.destroy_node()
        rclpy.shutdown()
        vehicle.close()

if __name__ == '__main__':
    main()
