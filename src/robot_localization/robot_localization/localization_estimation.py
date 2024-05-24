import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import gtsam
from gtsam.symbol_shorthand import X, V, B
from gtsam import PriorFactorPose3, BetweenFactorPose3, Pose3, Rot3, Point3, noiseModel
from gtsam import PreintegratedImuMeasurements, PreintegrationParams
import numpy as np
import open3d as o3d

def preintegration_parameters():
     gravity=9.81
     params=PreintegrationParams.MakeSharedU(gravity)
     I=np.eye(3)
     params.setAccelerometerCovariance(I * 0.1)  # Set accelerometer covariance
     params.setGyroscopeCovariance(I*0.1)  # Set gyroscope covariance
     params.setIntegrationCovariance(I*0.1)
     params.setUse2ndOrderCoriolis(False) # Disable 2nd order Coriolis effect
     params.setOmegaCoriolis(np.ones((3,1))*0.1) # Zero out 
     bias_covariance=noiseModel.Isotropic.Sigma(6,0.1)
     return params, bias_covariance

class localization_estimator(Node):
    def __init__(self):
        super().__init__('gtsam_locate')
        self.optimizer_initialized_ = False # Flag for optimizer initialization
        self.imu_sub_ = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)  # IMU data subscription
        self.odom_sub_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)  # Odometry data subscription
        self.lidar_sub_ = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)  #Lidar data subsciption
        self.state_pub_ = self.create_publisher(Odometry, '/estimated_state', 10) # Publish estimated state
        
        self.initial_pose_sub_ = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)  # Get initial pose from user's guess
        self.got_initial_pose = False

        # GTSAM
        prior_noise = noiseModel.Diagonal.Sigmas([0.1] * 6)  # Prior noise for initial position
        self.graph_ = gtsam.NonlinearFactorGraph()  # 
        self.graph_.add(PriorFactorPose3(X(0), Pose3(), prior_noise))  # Add prior factor to the graph
        self.initial_estimate_ = gtsam.Values()  # Initial state estimates
        self.initial_estimate_.insert(X(0), Pose3())

        self.prev_pose_ = Pose3()  # Previous pose
        self.prev_velocity_ = self.Vector3(0, 0, 0)  # Previous velocity 
        self.prev_bias_ = gtsam.imuBias.ConstantBias()
        self.prev_time_ = self.get_clock().now
        params, self.bias_covariance = preintegration_parameters()

        # Initialize point cloud
        self.reference_cloud = None
        self.keyframe_index_ = 1

    def initial_pose_callback(self, msg):
        self.got_initial_pose = True


    def Vector3(self, x, y, z):
        return np.array([x, y, z])

    def convert_laserscan_to_open3d(self, msg):
    
    	# Extract the data fram message
        ranges = np.array(msg.ranges)
        angle_min = np.array(msg.angle_min)
        angle_max = np.array(msg.angle_max)
        angle_increment = np.array(msg.angle_increment)
    	
    	# Create array with all angles and calculate x, y coordinates of each measurement
        angles = np.arange(angle_min, angle_max, angle_increment)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)
    	
    	# Create an array of points
        points = np.vstack((x, y, z)).T
    	
    	# Remove wrong measurements (missing or infinite values)
        valid_points = np.isfinite(ranges)
        points = points[valid_points]
    	
    	# Create a point cloud in open3d format
        point_cloud = o3d.geometry.PointCloud()  
        point_cloud.points = o3d.utility.Vector3dVector(points)
        return point_cloud

    def run_icp(self, source_cloud, target_cloud):
        threshold = 0.02   # Matching threshold
        trans_init = np.eye(4)  # Inicjalizacja macierzy transformacji
        icp_result = o3d.pipelines.registration.registration_icp(
            source_cloud, target_cloud, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())  #ICP
        return icp_result.transformation

    
    #Callback for IMU data
    def imu_callback(self, msg):
        if not msg:
            self.get_logger().error("Problem with IMU data")
            return
        accelerometer = self.Vector3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        gyroscope = self.Vector3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        current_time = self.get_clock().now()  # Aktualny czas
        dt = (current_time - self.prev_time_).nanoseconds / 1e9
        self.prev_time_ = current_time

    	#TODO 
        imu_noise = gtsam.noiseModel.Diagonal.Sigmas([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

     # Callback for odometry data
    def odom_callback(self, msg):
        if not msg:
            self.get_logger().error("Problem with odometry data")
            return
        odom_pose = Pose3(Rot3.Quaternion(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z),
            Point3(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z))

        # Implement factor graph for odometry data
        odom_noise = gtsam.noiseModel.Diagonal.Sigmas([msg.pose.covariance[0],
                                                        msg.pose.covariance[7],
                                                        msg.pose.covariance[14],
                                                        msg.pose.covariance[21],
                                                        msg.pose.covariance[28],
                                                        msg.pose.covariance[35]])
        # if self.keyframe_index_ == 0:
        #     self.graph_.add(PriorFactorPose3(X(self.keyframe_index_), odom_pose, odom_noise))
        #     self.initial_estimate_.insert(X(self.keyframe_index_), odom_pose)
        # else:
        relative_pose = self.prev_pose_.between(odom_pose)
        self.graph_.add(BetweenFactorPose3(X(self.keyframe_index_ - 1), X(self.keyframe_index_), relative_pose, odom_noise))
        self.initial_estimate_.insert(X(self.keyframe_index_), odom_pose)
        self.prev_pose_ = odom_pose
        self.optimize_graph()
        self.keyframe_index_ += 1

    def lidar_callback(self, msg):
        if not msg:
            self.get_logger().error("Problem with LIDAR data")
            return

        point_cloud = self.convert_laserscan_to_open3d(msg)
        if self.reference_cloud is None:
            self.reference_cloud = point_cloud
            return
      
        transformation = self.run_icp(point_cloud, self.reference_cloud)
        # Update point cloud
        self.reference_cloud = point_cloud
        # Extract rotation and translation from transformation matrix
        # rotation_matrix = transformation[:3, :3]
        # translation_vector = transformation[:3, 3]
        # rotation = Rot3(rotation_matrix)
        # translation = Point3(translation_vector[0], translation_vector[1], translation_vector[2])
        # lidar_pose = Pose3(rotation, translation)
        lidar_pose = Pose3(transformation)

        self.graph_.add(BetweenFactorPose3(X(self.keyframe_index_ - 1), X(self.keyframe_index_), lidar_pose, odom_noise))
        #TODO

    # Optimize the factor graph
    def optimize_graph(self):
        lm_params = gtsam.LevenbergMarquardtParams()
        lm_params.setMaxIterations(100)
        lm_params.setRelativeErrorTol(1e-5)
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph_, self.initial_estimate_, lm_params)
        self.optimized_estimate_ = optimizer.optimize()
        optimized_pose = self.optimized_estimate_.atPose3(X(self.keyframe_index_-1))
        optimized_position = optimized_pose.translation()
        optimized_orientation = optimized_pose.rotation().toQuaternion()
        
        # Create a message with estimate state and publish it
        estimated_state = Odometry()
        estimated_state.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(str(optimized_position))
        self.get_logger().info(str(optimized_orientation))
        estimated_state.pose.pose.position.x = optimized_position[0]
        estimated_state.pose.pose.position.y = optimized_position[1]
        estimated_state.pose.pose.position.z = optimized_position[2]
        estimated_state.pose.pose.orientation.w = optimized_orientation.w()
        estimated_state.pose.pose.orientation.x = optimized_orientation.x()
        estimated_state.pose.pose.orientation.y = optimized_orientation.y()
        estimated_state.pose.pose.orientation.z = optimized_orientation.z()
        self.state_pub_.publish(estimated_state)
        # self.graph_.resize(0)
        # self.initial_estimate_.clear()


def main(args=None):
    rclpy.init(args=args)
    estimator = localization_estimator()
    rclpy.spin(estimator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()