import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
import gtsam
from gtsam.symbol_shorthand import X, V, B
from gtsam import PriorFactorPose3, BetweenFactorPose3, Pose3, Rot3, Point3, noiseModel
from gtsam import PreintegratedImuMeasurements, PreintegrationParams
import numpy as np
import open3d as o3d
import math

# From https://learnopencv.com/rotation-matrix-to-euler-angles/
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
# Calculates rotation matrix to euler angles
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

class localization_estimator(Node):
    def __init__(self):
        super().__init__('gtsam_locate')
        self.optimizer_initialized_ = False # Flag for optimizer initialization
        self.imu_sub_ = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)  # IMU data subscription
        self.odom_sub_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)  # Odometry data subscription
        self.lidar_sub_ = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)  #Lidar data subsciption
        self.state_pub_ = self.create_publisher(Odometry, '/estimated_state', 10) # Publish estimated state
        self.ground_truth_sub_ = self.create_subscription(TFMessage, '/tf', self.transform_callback, 10) # Transform subscription (to get real position of the robot)
        
        self.initial_pose_sub_ = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)  # Get initial pose from user's guess
        self.got_initial_pose = False

        # GTSAM
        prior_noise = noiseModel.Diagonal.Sigmas([0.1] * 6)  # Prior noise for initial position
        self.graph_ = gtsam.NonlinearFactorGraph()  # Create graph
        self.graph_.add(PriorFactorPose3(X(0), Pose3(), prior_noise))  # Add prior factor to the graph
        self.initial_estimate_ = gtsam.Values()  # Initial state estimates
        self.initial_estimate_.insert(X(0), Pose3())

        self.prev_pose_ = Pose3()  # Previous pose
        self.prev_velocity_ = self.Vector3(0, 0, 0)  # Previous velocity 
        self.prev_bias_ = gtsam.imuBias.ConstantBias()
        self.prev_time_ = self.get_clock().now
        self.pim = None
        self.is_pim_set = False
        self.imu_timer = 0.0

        # Initialize point cloud
        self.reference_cloud = None
        self.keyframe_index_ = 1

        # Collect data to calculate Cov matrix for Lidar
        # with open("lidar_data.csv", 'w') as file:
        #     file.write('x,y,z,rot_x,rot_y,rot_z\n')
        #     file.close()



    def initial_pose_callback(self, msg):
        self.got_initial_pose = False
        self.graph_ = gtsam.NonlinearFactorGraph()  # Reset the graph
        prior_noise = noiseModel.Diagonal.Sigmas([0.1] * 6)  # Prior noise for initial position
        self.graph_.add(PriorFactorPose3(X(0), Pose3(), prior_noise))  # Add prior factor to the graph
        self.initial_estimate_ = gtsam.Values()  # Reset Initial state
        self.initial_estimate_.insert(X(0), Pose3())
        self.got_initial_pose = True
        self.keyframe_index_ = 1
        # Save the trajectory
        with open("trajectory.csv", 'w') as file:
            file.write('type,t,x,y,z,qx,qy,qz,qw\n')
            file.close()


    def preintegration_parameters(self, msg):
        gravity=9.81
        params=PreintegrationParams.MakeSharedU(gravity)
        I=np.eye(3)
        params.setAccelerometerCovariance(np.array(msg.linear_acceleration_covariance).reshape((3, 3)))  # Set accelerometer covariance
        params.setGyroscopeCovariance(np.array(msg.angular_velocity_covariance).reshape((3, 3)))  # Set gyroscope covariance
        params.setIntegrationCovariance(I*1e-7)
        params.setUse2ndOrderCoriolis(False) # Disable 2nd order Coriolis effect
        params.setOmegaCoriolis(np.zeros((3,1))) # Zero out 
        # As in https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_description/urdf/turtlebot3_waffle.gazebo.xacro
        accBias = np.array([0.1, 0.1, 0.1])
        gyroBias = np.array([0.0000075, 0.0000075, 0.0000075])
        bias_covariance=noiseModel.Isotropic.Sigma(6,0.1)
        actualBias = gtsam.imuBias.ConstantBias(accBias, gyroBias)
        # Calculate PIM
        self.pim = PreintegratedImuMeasurements(params, actualBias)
        self.is_pim_set = True
        

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
        if not is_pim_set:
            self.preintegration_parameters(msg)
        if self.got_initial_pose:
            accelerometer = self.Vector3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
            gyroscope = self.Vector3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
            current_time = self.get_clock().now()  # Aktualny czas
            dt = (current_time - self.prev_time_).nanoseconds / 1e9
            self.prev_time_ = current_time

            pim.integrateMeasurement(measuredAcc, measuredOmega, dt)

            if self.imu_timer > 0.1:
                imufactor=gtsam.ImuFactor(X(self.keyframe_index -1), V(self.keyframeindex -1), X(self.keyframeindex), V(self.keyframeindex), B(0), self.pim)
                self.graph_.add(imufactor)
                dt = 0.0
                pim.resetIntegration()

     # Callback for odometry data
    def odom_callback(self, msg):
        if not msg:
            self.get_logger().error("Problem with odometry data")
            return
        if self.got_initial_pose:
            pos_x = msg.pose.pose.position.x + np.random.normal(0, 0.1)
            pos_y = msg.pose.pose.position.y + np.random.uniform(0, 0.1)
            # pos_x = msg.pose.pose.position.x
            # pos_y = msg.pose.pose.position.y
            pos_z = msg.pose.pose.position.z
            t = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            with open("trajectory.csv", 'a') as file:
                new_line = 'odom,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n' % (t, pos_x, pos_y, pos_z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                                                         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
                file.write(new_line)
            odom_pose = Pose3(Rot3.Quaternion(
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z),
                Point3(
                    pos_x,
                    pos_y,
                    pos_z))

            # Implement factor graph for odometry data
            odom_noise = gtsam.noiseModel.Diagonal.Sigmas([msg.pose.covariance[0] + 0.01,
                                                            msg.pose.covariance[7] + 0.01,
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
        if self.got_initial_pose:
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
            # Save data to calculate cov matrix
            # euler_angles = rotationMatrixToEulerAngles(rotation_matrix)
            # new_line = '%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n' % (translation_vector[0], translation_vector[1], translation_vector[2],
            #                                                 euler_angles[0], euler_angles[1], euler_angles[2])
            # with open('lidar_data.csv', 'a') as file:
            #     file.write(new_line)

            lidar_pose = Pose3(transformation)

            lidar_noise = gtsam.noiseModel.Diagonal.Sigmas([5.21146953e-07, 3.11243524e-07, 0.0, 0.0, 0.0, 3.48658533e-09])

            self.graph_.add(BetweenFactorPose3(X(self.keyframe_index_ - 1), X(self.keyframe_index_), lidar_pose, lidar_noise))

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
        estimated_state.header.frame_id = "map"
        estimated_state.pose.pose.position.x = optimized_position[0]
        estimated_state.pose.pose.position.y = optimized_position[1]
        estimated_state.pose.pose.position.z = optimized_position[2]
        estimated_state.pose.pose.orientation.w = optimized_orientation.w()
        estimated_state.pose.pose.orientation.x = optimized_orientation.x()
        estimated_state.pose.pose.orientation.y = optimized_orientation.y()
        estimated_state.pose.pose.orientation.z = optimized_orientation.z()
        self.state_pub_.publish(estimated_state)
        with open("trajectory.csv", 'a') as file:
            t = estimated_state.header.stamp.sec + estimated_state.header.stamp.nanosec/1e9
            new_line = 'estimator,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n' % (t, estimated_state.pose.pose.position.x, estimated_state.pose.pose.position.y,
                                                                        estimated_state.pose.pose.position.z, 
                                                                        estimated_state.pose.pose.orientation.x, estimated_state.pose.pose.orientation.y,
                                                                        estimated_state.pose.pose.orientation.z, estimated_state.pose.pose.orientation.w)
            file.write(new_line)
        # self.graph_.resize(0)
        # self.initial_estimate_.clear()

    def transform_callback(self, msg):
        for transformation in msg.transforms:
            if transformation.header.frame_id == "odom" and transformation.child_frame_id == "base_footprint":
                with open("trajectory.csv", 'a') as file:
                    t = transformation.header.stamp.sec + transformation.header.stamp.nanosec/1e9
                    new_line = 'real,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n' % (t, transformation.transform.translation.x, transformation.transform.translation.y,
                                                                                transformation.transform.translation.z, transformation.transform.rotation.x,
                                                                                transformation.transform.rotation.y, transformation.transform.rotation.z,
                                                                                transformation.transform.rotation.w)
                    file.write(new_line)

def main(args=None):
    rclpy.init(args=args)
    estimator = localization_estimator()
    rclpy.spin(estimator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
