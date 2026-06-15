#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import pyzed.sl as sl
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import threading
import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, PointCloud2, PointField
from collections import deque
import numpy as np

MIN_DIST_SQ = .1 

RESOLUTIONS = {
    "HD1200":   sl.RESOLUTION.HD1200,
    "HD1080": sl.RESOLUTION.HD1080,
    "SVGA":   sl.RESOLUTION.SVGA,
}

STATUS_MAP = {
    sl.SPATIAL_MEMORY_STATUS.INITIALIZING: "INITIALIZING",
    sl.SPATIAL_MEMORY_STATUS.KNOWN_MAP:    "KNOWN_MAP",
    sl.SPATIAL_MEMORY_STATUS.LOOP_CLOSED:  "LOOP_CLOSED",
    sl.SPATIAL_MEMORY_STATUS.LOST:         "LOST",
    sl.SPATIAL_MEMORY_STATUS.MAP_UPDATE:   "MAP_UPDATE",
}

DEPTH_MODE = {
    "NEURAL_LIGHT": sl.DEPTH_MODE.NEURAL_LIGHT,
    "NEURAL": sl.DEPTH_MODE.NEURAL,
    "NEURAL_PLUS": sl.DEPTH_MODE.NEURAL_PLUS
}

class ZEDSLAMNode(Node):
    def __init__(self):
        super().__init__('zed_positional_tracking_node')

        # -------------- Publishers ---------------------
        self.pose_pub               = self.create_publisher(PoseStamped,              '/zed/zed_node/pose', 10)
        self.pose_with_covariance_pub = self.create_publisher(PoseWithCovarianceStamped, '/zed/zed_node/pose_with_covariance', 10)
        self.status_pub = self.create_publisher(DiagnosticArray, '/zed/spatial_memory_status', 10)
        self.path_pub   = self.create_publisher(Path,          '/zed/path', 10)
        self.odom_pub   = self.create_publisher(Odometry,      '/zed/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self._save_lock = threading.Lock()
        self.create_service(Trigger, '/zed/save_map', self._save_map_cb)

        # ---------------- Load Config ----------------
        self.declare_parameter('fps', 30)
        self.declare_parameter('resolution', 'SVGA')
        self.declare_parameter('depth_mode', 'NEURAL_LIGHT')
        self.declare_parameter('area_file', '')
        self.declare_parameter('initial_mapping', False)
        self.declare_parameter('update_map', True)
        self.declare_parameter('enable_localization_only', False)
        self.declare_parameter('publish_image', False)
        self.declare_parameter('publish_pointcloud', False)
        self.declare_parameter('publish_depth', False)
        self.declare_parameter('pointcloud_rate', 5.0)
        self.declare_parameter('pointcloud_width',  448)
        self.declare_parameter('pointcloud_height', 256)
        self.declare_parameter('save_pointcloud', False)
        self.declare_parameter('enable_2d_mode', False)

        self.fps = self.get_parameter('fps').value
        self.resolution = self.get_parameter('resolution').value
        self.area_file = self.get_parameter('area_file').value
        self.initial_mapping = self.get_parameter('initial_mapping').value
        self.update_map = self.get_parameter('update_map').value
        self.depth_mode = self.get_parameter('depth_mode').value
        self.enable_localization_only = self.get_parameter('enable_localization_only').value
        self.publish_image = self.get_parameter('publish_image').value
        self.publish_pointcloud = self.get_parameter('publish_pointcloud').value
        self.publish_depth = self.get_parameter('publish_depth').value
        self.pointcloud_rate   = self.get_parameter('pointcloud_rate').value
        pc_w = self.get_parameter('pointcloud_width').value
        pc_h = self.get_parameter('pointcloud_height').value
        self.pc_resolution = sl.Resolution(pc_w, pc_h)
        self.save_pointcloud = self.get_parameter('save_pointcloud').value
        self.enable_2d_mode = self.get_parameter('enable_2d_mode').value

        if self.publish_image:
            self.img_pub   = self.create_publisher(Image,       '/zed/zed_node/left/image_rect_color', 1)
            self.img_mat   = sl.Mat()
        if self.publish_depth:
            self.depth_pub = self.create_publisher(Image,       '/zed/zed_node/depth/depth_registered', 1)
            self.depth_mat = sl.Mat()
        if self.publish_pointcloud:
            self.pc_pub    = self.create_publisher(PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', 1)
            self.pc_mat    = sl.Mat()

        # ---------------- State ----------------
        self.running = False
        self.path_poses = deque(maxlen=500)
        self.last_mem_status = None

        # ---------------- Camera Init ----------------
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = RESOLUTIONS[self.resolution]
        init_params.camera_fps = self.fps
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_params.depth_mode = DEPTH_MODE[self.depth_mode]

        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("ZED Camera failed to open!")
            rclpy.shutdown()
            return

        # ---------------- Tracking Init ----------------
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
        tracking_params.enable_area_memory = True
        tracking_params.enable_imu_fusion = True
        tracking_params.set_gravity_as_origin = True
        tracking_params.enable_localization_only = self.enable_localization_only
        tracking_params.enable_2d_ground_mode = self.enable_2d_mode

        if self.enable_2d_mode:
            self.get_logger().info("2D ground mode enabled — tracking constrained to XY plane")

        if self.area_file:
            if not self.initial_mapping:
                tracking_params.area_file_path = self.area_file
                self.get_logger().info(f"Loading Map with Area File: {self.area_file}")
                if self.enable_localization_only:
                    self.get_logger().info(f"Enabled Localization Only mode")
            else:
                self.get_logger().info(f"Initial Mapping with Area File: {self.area_file}")

        self.zed.enable_positional_tracking(tracking_params)

        if self.save_pointcloud and self.update_map and self.area_file:
            self._enable_spatial_mapping()
        elif self.save_pointcloud:
            self.get_logger().warn(
                "save_pointcloud=True but update_map or area_file not set — spatial mapping NOT enabled."
            )
            self.save_pointcloud = False

        self.runtime_params = sl.RuntimeParameters()
        # Depth is only needed when the point cloud or depth image threads request it;
        # disable by default so grab() doesn't pay the NEURAL compute cost every frame.
        self.runtime_params.enable_depth = self.publish_depth or self.publish_pointcloud or self.save_pointcloud
        self.pose = sl.Pose()

        self.running = True

        self.grab_thread = threading.Thread(target=self.grab_loop)
        self.grab_thread.start()
        if self.publish_pointcloud:
            self.pc_thread = threading.Thread(target=self._pc_loop, daemon=True)
            self.pc_thread.start()
        self.get_logger().info("ZED Positional Tracking Node started")

    def _moved_enough(self, x, y, z):
        if not self.path_poses:
            return True
        last = self.path_poses[-1].pose.position
        dx, dy, dz = x - last.x, y - last.y, z - last.z
        return dx*dx + dy*dy + dz*dz > MIN_DIST_SQ

    def _save_map_cb(self, request, response):
        if not (self.update_map and self.area_file):
            response.success = False
            response.message = "update_map or area_file not configured — nothing to save"
            return response

        if not self._save_lock.acquire(blocking=False):
            response.success = False
            response.message = "Save already in progress"
            return response

        try:
            self.get_logger().info("Service call: saving area map...")
            self.zed.save_area_map(self.area_file)
            msg = f"Area map saved to {self.area_file}"

            if self.save_pointcloud:
                self.get_logger().info("Service call: extracting fused point cloud...")
                fused_pc = sl.FusedPointCloud()
                err = self.zed.extract_whole_spatial_map(fused_pc)
                if err == sl.ERROR_CODE.SUCCESS:
                    ply_path = os.path.splitext(self.area_file)[0] + '.ply'
                    if fused_pc.save(ply_path, sl.MESH_FILE_FORMAT.PLY):
                        msg += f"; PLY saved to {ply_path} ({fused_pc.get_number_of_points()} points)"
                    else:
                        msg += "; PLY save failed"
                else:
                    msg += f"; PLY extraction failed: {err}"

            self.get_logger().info(msg)
            response.success = True
            response.message = msg
        finally:
            self._save_lock.release()

        return response

    def _enable_spatial_mapping(self):
        mapping_params = sl.SpatialMappingParameters()
        mapping_params.map_type = sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD
        mapping_params.set_resolution(sl.MAPPING_RESOLUTION.MEDIUM)
        mapping_params.set_range(sl.MAPPING_RANGE.AUTO)
        mapping_params.save_texture = False
        err = self.zed.enable_spatial_mapping(mapping_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Failed to enable spatial mapping: {err}. PLY will NOT be saved.")
            self.save_pointcloud = False
        else:
            ply_path = os.path.splitext(self.area_file)[0] + '.ply'
            self.get_logger().info(f"Spatial mapping enabled. Fused pointcloud will be saved to: {ply_path}")

    def _pc_loop(self):
        interval = 1.0 / self.pointcloud_rate
        while self.running and rclpy.ok():
            t0 = time.monotonic()

            if self.pc_pub.get_subscription_count() > 0:
                # XYZBGRA gives [B,G,R,A] color bytes — matches ROS "rgb" field directly
                self.zed.retrieve_measure(self.pc_mat, sl.MEASURE.XYZBGRA, sl.MEM.CPU, self.pc_resolution)
                pc_np = self.pc_mat.get_data()
                h, w  = pc_np.shape[:2]
                pc_msg = PointCloud2()
                pc_msg.header.stamp    = self.get_clock().now().to_msg()
                pc_msg.header.frame_id = 'zed_left_camera_frame'
                pc_msg.height     = h
                pc_msg.width      = w
                pc_msg.fields     = [
                    PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
                ]
                pc_msg.is_bigendian = False
                pc_msg.point_step   = 16
                pc_msg.row_step     = w * 16
                pc_msg.is_dense     = False
                pc_msg.data         = np.ascontiguousarray(pc_np).tobytes()
                self.pc_pub.publish(pc_msg)

            elapsed = time.monotonic() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def grab_loop(self):
        while self.running and rclpy.ok():
            if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
                continue

            state = self.zed.get_position(self.pose)
            if state != sl.POSITIONAL_TRACKING_STATE.OK:
                self.get_logger().warn("Tracking lost", once=True)
                continue
            mem_status = self.zed.get_positional_tracking_status().spatial_memory_status

            if mem_status != self.last_mem_status:
                self.get_logger().info(f"Memory status changed: {STATUS_MAP.get(mem_status, 'UNKNOWN')}")
                self.last_mem_status = mem_status
            # Extract pose data
            t = self.pose.get_translation(sl.Translation())
            x, y, z = t.get()
            o = self.pose.get_orientation(sl.Orientation())
            x_or, y_or, z_or, w_or = o.get()

            stamp = self.get_clock().now().to_msg()

            # ---------------- Image Publish ----------------
            if self.publish_image and self.img_pub.get_subscription_count() > 0:
                self.zed.retrieve_image(self.img_mat, sl.VIEW.LEFT)
                img_np = self.img_mat.get_data()   # H×W×4 uint8 BGRA
                img_msg = Image()
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = 'zed_left_camera_optical_frame'
                img_msg.height = img_np.shape[0]
                img_msg.width  = img_np.shape[1]
                img_msg.encoding     = 'bgra8'
                img_msg.is_bigendian = False
                img_msg.step         = img_np.shape[1] * 4
                img_msg.data         = img_np.tobytes()
                self.img_pub.publish(img_msg)

            # ---------------- Depth Publish ----------------
            if self.publish_depth and self.depth_pub.get_subscription_count() > 0:
                self.zed.retrieve_measure(self.depth_mat, sl.MEASURE.DEPTH)
                depth_np = self.depth_mat.get_data()   # H×W float32, metres
                depth_msg = Image()
                depth_msg.header.stamp = stamp
                depth_msg.header.frame_id = 'zed_left_camera_optical_frame'
                depth_msg.height = depth_np.shape[0]
                depth_msg.width  = depth_np.shape[1]
                depth_msg.encoding     = '32FC1'
                depth_msg.is_bigendian = False
                depth_msg.step         = depth_np.shape[1] * 4
                depth_msg.data         = depth_np.tobytes()
                self.depth_pub.publish(depth_msg)

            # ---------------- Diagnostic Publish ----------------
            diag_status = DiagnosticStatus()
            diag_status.message = STATUS_MAP.get(mem_status, "UNKNOWN")

            diag_msg = DiagnosticArray()
            diag_msg.header.stamp = stamp
            diag_msg.status = [diag_status]

            self.status_pub.publish(diag_msg)

            # ---------------- Pose Publish ----------------
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            pose_msg.pose.orientation.x = x_or
            pose_msg.pose.orientation.y = y_or
            pose_msg.pose.orientation.z = z_or
            pose_msg.pose.orientation.w = w_or

            self.pose_pub.publish(pose_msg)

            # ---------------- Odom Publish ----------------
            odom_msg = Odometry()
            odom_msg.header.stamp = stamp
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = z
            odom_msg.pose.pose.orientation.x = x_or
            odom_msg.pose.pose.orientation.y = y_or
            odom_msg.pose.pose.orientation.z = z_or
            odom_msg.pose.pose.orientation.w = w_or
            odom_msg.pose.covariance = self.pose.pose_covariance.flatten().tolist()

            twist_pose = sl.Pose()
            self.zed.get_position(twist_pose, sl.REFERENCE_FRAME.CAMERA)

            lin_vel = twist_pose.twist[0:3]
            ang_vel = twist_pose.twist[3:6]

            odom_msg.twist.twist.linear.x = lin_vel[0]
            odom_msg.twist.twist.linear.y = lin_vel[1]
            odom_msg.twist.twist.linear.z = lin_vel[2]
            odom_msg.twist.twist.angular.x = ang_vel[0]
            odom_msg.twist.twist.angular.y = ang_vel[1]
            odom_msg.twist.twist.angular.z = ang_vel[2]
            odom_msg.twist.covariance = twist_pose.twist_covariance.flatten().tolist()

            self.odom_pub.publish(odom_msg)

            # ---------------- Transform Publish ----------------
            tform = TransformStamped()
            tform.header.stamp = stamp
            tform.header.frame_id = "map"
            tform.child_frame_id = "zed_camera_link"
            tform.transform.translation.x = x
            tform.transform.translation.y = y
            tform.transform.translation.z = z
            tform.transform.rotation.x = x_or
            tform.transform.rotation.y = y_or
            tform.transform.rotation.z = z_or
            tform.transform.rotation.w = w_or

            self.tf_broadcaster.sendTransform(tform)

            # ---------------- Path Publish ----------------
            self.path_poses = self.path_poses
            if mem_status == sl.SPATIAL_MEMORY_STATUS.INITIALIZING:
            	continue
            if self._moved_enough(x, y, z):
                self.path_poses.append(pose_msg)

                path_msg = Path()
                path_msg.header.stamp = self.get_clock().now().to_msg()
                path_msg.header.frame_id = "map"
                path_msg.poses = list(self.path_poses)

                self.path_pub.publish(path_msg)

    # ---------------- Shutdown ----------------
    def destroy_node(self):
        self.running = False
        self.grab_thread.join()
        if self.publish_pointcloud:
            self.pc_thread.join()

        if self.update_map and self.area_file:
            self.get_logger().info("Saving area map before shutdown...")
            self.zed.save_area_map(self.area_file)

        if self.save_pointcloud:
            self.get_logger().info("Extracting fused point cloud (this may take a few seconds)...")
            fused_pc = sl.FusedPointCloud()
            err = self.zed.extract_whole_spatial_map(fused_pc)
            if err == sl.ERROR_CODE.SUCCESS:
                ply_path = os.path.splitext(self.area_file)[0] + '.ply'
                if fused_pc.save(ply_path, sl.MESH_FILE_FORMAT.PLY):
                    self.get_logger().info(
                        f"Fused point cloud saved to {ply_path} "
                        f"({fused_pc.get_number_of_points()} points)"
                    )
                else:
                    self.get_logger().error(f"fused_pc.save() failed for path: {ply_path}")
            else:
                self.get_logger().error(f"extract_whole_spatial_map failed: {err}")
            self.zed.disable_spatial_mapping()

        self.zed.disable_positional_tracking()
        self.zed.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZEDSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
