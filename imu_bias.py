# #!/usr/bin/env python

# import rospy
# from sensor_msgs.msg import Imu
# import numpy as np

# class IMUDataProcessor:
#     def __init__(self):
#         rospy.init_node('imu_data_processor', anonymous=True)

#         # Initialize variables for accumulating data
#         self.gyro_data = np.zeros(3)
#         self.accel_data = np.zeros(3)
#         self.samples = 0

#         self.gyro_bias = 0
#         self.accel_bias = 0
#         self.gyro_variance = 0
#         self.accel_variance  = 0

#         # Lists to store raw IMU data
#         self.raw_gyro_data = []
#         self.raw_accel_data = []

#         # Define the IMU data subscriber
#         rospy.Subscriber('/imu', Imu, self.imu_callback)



#         # Define the publisher for corrected IMU data
#         self.corrected_imu_pub = rospy.Publisher('/corrected_imu', Imu, queue_size=10)

#     def imu_callback(self, data):
#         # Extract gyro and accelerometer data from the IMU message
#         gyro_data = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
#         accel_data = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]

#         # Accumulate data for bias calculation
#         self.gyro_data += np.array(gyro_data)
#         self.accel_data += np.array(accel_data)
#         self.samples += 1

#         # Append raw gyro and accelerometer data to the lists
#         self.raw_gyro_data.append(np.array(gyro_data))
#         self.raw_accel_data.append(np.array(accel_data))

#         secs = 20
#         frequency = 200

#         if self.samples > frequency * secs  and self.samples < frequency * secs + 10:
#             # Calculate and publish biases, then process and publish corrected IMU data
#             self.calculate_biases()
#         elif self.samples == frequency * secs + 10:
            
#             rospy.loginfo("Gyro Bias (X, Y, Z): {}".format(self.gyro_bias))
#             rospy.loginfo("Accelerometer Bias (X, Y, Z): {}".format(self.accel_bias))
#             rospy.loginfo("Gyro Variance (X, Y, Z): {}".format(self.gyro_variance))
#             rospy.loginfo("Accelerometer Variance (X, Y, Z): {}".format(self.accel_variance))

            
        

#         elif self.samples >  frequency * secs + 10 :
#             self.process_imu_data(data)

#     def plot_data(self):
#         # Plot mean biases
#         self.ax1.plot(self.time_steps, self.mean_gyro_biases, label='Mean Gyro Biases')
#         self.ax1.plot(self.time_steps, self.mean_accel_biases, label='Mean Accelerometer Biases')

#         # Plot variances
#         self.ax2.plot(self.time_steps, self.gyro_variances, label='Gyro Variances')
#         self.ax2.plot(self.time_steps, self.accel_variances, label='Accelerometer Variances')

#         # Set labels and legends
#         self.ax1.set_ylabel('Mean Bias')
#         self.ax2.set_ylabel('Variance')
#         self.ax2.set_xlabel('Time Steps')
#         self.ax1.legend()
#         self.ax2.legend()

#         # Show the plot
#         plt.show()


#     def calculate_biases(self):
#         if self.samples > 0:
#             # Calculate biases as the average of accumulated data
#             self.gyro_bias = self.gyro_data / self.samples
#             self.accel_bias = self.accel_data / self.samples

#             # Convert the lists to numpy arrays
#             raw_gyro_array = np.array(self.raw_gyro_data)
#             raw_accel_array = np.array(self.raw_accel_data)

#             # Calculate variance along each axis for gyro and accelerometer
#             self.gyro_variance = np.var(raw_gyro_array, axis=0)
#             self.accel_variance = np.var(raw_accel_array, axis=0)


#         else:
#             rospy.logwarn("No IMU data received for bias calculation.")


#         #     # rospy.loginfo("Gyro Bias: {}".format(self.gyro_bias))
#         #     # rospy.loginfo("Accelerometer Bias: {}".format(self.accel_bias))
#         # else:
#         #     rospy.logwarn("No IMU data received for bias calculation.")


#     def publish_bias(self, gyro_bias, accel_bias):
#         # Create and publish a message containing the biases

#         bias_msg = Imu()
#         bias_msg.angular_velocity.x = gyro_bias[0]
#         bias_msg.angular_velocity.y = gyro_bias[1]
#         bias_msg.angular_velocity.z = gyro_bias[2]
#         bias_msg.linear_acceleration.x = accel_bias[0]
#         bias_msg.linear_acceleration.y = accel_bias[1]
#         bias_msg.linear_acceleration.z = accel_bias[2]

#         self.corrected_imu_pub.publish(bias_msg)

#     def process_imu_data(self, data):
#         # Define a callback function to process raw IMU data and publish corrected data
#         # Subtract biases from raw IMU data
#         corrected_gyro = [data.angular_velocity.x - self.gyro_bias[0],
#                             data.angular_velocity.y - self.gyro_bias[1],
#                             data.angular_velocity.z - self.gyro_bias[2]]
#         corrected_accel = [data.linear_acceleration.x - self.accel_bias[0],
#                             data.linear_acceleration.y - self.accel_bias[1],
#                             data.linear_acceleration.z - self.accel_bias[2]  ]

#         # Create a new IMU message with corrected data
#         corrected_imu_msg = Imu()
#         corrected_imu_msg.header = data.header
#         corrected_imu_msg.angular_velocity.x, corrected_imu_msg.angular_velocity.y, corrected_imu_msg.angular_velocity.z = corrected_gyro
#         corrected_imu_msg.linear_acceleration.x, corrected_imu_msg.linear_acceleration.y, corrected_imu_msg.linear_acceleration.z = corrected_accel

#         # Publish the corrected IMU data
#         self.corrected_imu_pub.publish(corrected_imu_msg)

# if __name__ == '__main__':
#     imu_processor = IMUDataProcessor()


#     try:
#         # Run until the node is stopped
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass



#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import numpy as np
import matplotlib.pyplot as plt

class IMUDataProcessor:
    def __init__(self):
        rospy.init_node('imu_data_processor', anonymous=True)

        # Initialize variables for accumulating data
        self.gyro_data = np.zeros(3)
        self.accel_data = np.zeros(3)
        self.samples = 0

        self.gyro_bias = 0
        self.accel_bias = 0
        self.gyro_variance = 0
        self.accel_variance = 0

        # Lists to store raw IMU data
        self.raw_gyro_data = []
        self.raw_accel_data = []


        self.last_position = [0, 0, 0]
        self.last_velocity = [0, 0, 0]

        self.corrected_current_velocity = [0, 0, 0]
        self.corrected_gyro_integration = [0, 0, 0]

        # Define the IMU data subscriber
        rospy.Subscriber('/imu', Imu, self.imu_callback)

        # Define the publisher for corrected IMU data
        self.corrected_imu_pub = rospy.Publisher('/corrected_imu', Imu, queue_size=10)

        # Define the publisher for odometry data
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # Create figure and axes for plotting
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True)
        self.fig.suptitle('IMU Data Analysis')

        # Initialize arrays for plotting
        self.time_steps = []
        self.mean_gyro_biases = []
        self.mean_accel_biases = []
        self.gyro_variances = []
        self.accel_variances = []

        # Variable to store the timestamp of the previous IMU reading
        self.prev_imu_timestamp = None

        # Register the shutdown callback
        rospy.on_shutdown(self.on_shutdown)


    def imu_callback(self, data):
        # Extract gyro and accelerometer data from the IMU message
        gyro_data = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
        accel_data = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]


        # Accumulate data for bias calculation
        self.gyro_data += np.array(gyro_data)
        self.accel_data += accel_data
        self.samples += 1

        # Append raw gyro and accelerometer data to the lists
        self.raw_gyro_data.append(np.array(gyro_data))
        self.raw_accel_data.append(accel_data)

        secs = 10
        frequency = 200

        if self.samples > frequency * secs and self.samples < frequency * secs + 10:
            # Calculate and publish biases, then process and publish corrected IMU data
            self.calculate_biases()
        elif self.samples == frequency * secs + 10:
            rospy.loginfo("Gyro Bias (X, Y, Z): {}".format(self.gyro_bias))
            rospy.loginfo("Accelerometer Bias (X, Y, Z): {}".format(self.accel_bias))
            rospy.loginfo("Gyro Variance (X, Y, Z): {}".format(self.gyro_variance))
            rospy.loginfo("Accelerometer Variance (X, Y, Z): {}".format(self.accel_variance))
            # # Shut down ROS after plotting data
            # rospy.signal_shutdown("Plotting completed.")
        elif self.samples >  frequency * secs + 10:
            self.process_imu_data(data)

    def calculate_biases(self):
        if self.samples > 0:
            # Calculate biases as the average of accumulated data
            self.gyro_bias = self.gyro_data / self.samples
            self.accel_bias = self.accel_data / self.samples

            # Convert the lists to numpy arrays
            raw_gyro_array = np.array(self.raw_gyro_data)
            raw_accel_array = np.array(self.raw_accel_data)

            # Calculate variance along each axis for gyro and accelerometer
            self.gyro_variance = np.var(raw_gyro_array, axis=0)
            self.accel_variance = np.var(raw_accel_array, axis=0)

            # Append values for plotting
            self.mean_gyro_biases.append(self.gyro_bias)
            self.mean_accel_biases.append(self.accel_bias)
            self.gyro_variances.append(self.gyro_variance)
            self.accel_variances.append(self.accel_variance)

            # Append current time step for plotting
            self.time_steps.append(self.samples)

            # Publish corrected IMU data
            self.publish_corrected_imu_data()

        else:
            rospy.logwarn("No IMU data received for bias calculation.")

    def process_imu_data(self, data):
        # Define a callback function to process raw IMU data and publish corrected data
        # Subtract biases from raw IMU data
        corrected_gyro = [
            data.angular_velocity.x - self.gyro_bias[0],
            data.angular_velocity.y - self.gyro_bias[1],
            data.angular_velocity.z - self.gyro_bias[2],
        ]
        corrected_accel = [
            data.linear_acceleration.x - self.accel_bias[0],
            data.linear_acceleration.y - self.accel_bias[1],
            data.linear_acceleration.z - self.accel_bias[2], 
        ]

        # Create a new IMU message with corrected data
        corrected_imu_msg = Imu()
        corrected_imu_msg.header = data.header
        corrected_imu_msg.angular_velocity.x, corrected_imu_msg.angular_velocity.y, corrected_imu_msg.angular_velocity.z = corrected_gyro
        corrected_imu_msg.linear_acceleration.x, corrected_imu_msg.linear_acceleration.y, corrected_imu_msg.linear_acceleration.z = corrected_accel

        # Publish the corrected IMU data
        self.corrected_imu_pub.publish(corrected_imu_msg)

        # Create and publish odometry data based on corrected IMU readings
        odom_msg = Odometry()
        odom_msg.header = data.header

        # Integrate accelerometer data to estimate position
        dt = 1.0 / 200.0  # Assuming the IMU operates at 200 Hz

        # Calculate time difference (dt) between current and previous IMU readings
        # if self.prev_imu_timestamp is not None:
        #     dt = (data.header.stamp - self.prev_imu_timestamp).to_sec()
        # else:
        #     dt = 0.0


        # Store the current timestamp as the previous timestamp for the next iteration
        self.prev_imu_timestamp = data.header.stamp
        corrected_accel_integration = [
            0.5 * corrected_accel[0] * dt ** 2,
            0.5 * corrected_accel[1] * dt ** 2,
            0.5 * corrected_accel[2] * dt ** 2,
        ]

        corrected_vel_integration = [
            self.last_velocity[0] * dt ,
             self.last_velocity[1] * dt,
             self.last_velocity[2] *dt ]

        self.corrected_current_velocity =  [self.last_velocity[0] + corrected_accel[0] * dt, 
                                            self.last_velocity[1] + corrected_accel[1] * dt,
                                            self.last_velocity[2] + corrected_accel[2] * dt]
        

        # self.corrected_current_velocity =  [corrected_accel[0] * dt, 
        #                                     corrected_accel[1] * dt,
        #                                     corrected_accel[2] * dt]
        
        
        # print(self.corrected_current_velocity)




        # Update position based on integration
        odom_msg.pose.pose.position.x = self.last_position[0] + corrected_accel_integration[0] + corrected_vel_integration[0]
        odom_msg.pose.pose.position.y = self.last_position[1] + corrected_accel_integration[1] + corrected_vel_integration[1]
        odom_msg.pose.pose.position.z = self.last_position[2] + corrected_accel_integration[2] + corrected_vel_integration[2]

        odom_msg.twist.twist.linear.x = self.corrected_current_velocity[0]
        odom_msg.twist.twist.linear.y = self.corrected_current_velocity[1]
        odom_msg.twist.twist.linear.z = self.corrected_current_velocity[2]

        odom_msg.twist.twist.angular.x = corrected_gyro[0]
        odom_msg.twist.twist.angular.y = corrected_gyro[1]
        odom_msg.twist.twist.angular.z = corrected_gyro[2]

        # Update orientation (using simple integration from gyro readings)
        self.corrected_gyro_integration += [
            corrected_gyro[0] * dt,
            corrected_gyro[1] * dt,
            corrected_gyro[2] * dt,
        ]

        # Quaternion integration
        q_delta = self.quaternion_from_euler(self.corrected_gyro_integration[0], self.corrected_gyro_integration[1], self.corrected_gyro_integration[2])
        q_current = Quaternion(
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w,
        )

        q_new = self.quaternion_multiply(q_current, q_delta)

        # # Normalize the quaternion to prevent drift
        # q_new_normalized = self.quaternion_normalize(q_new)

        # Update orientation in odometry message
        odom_msg.pose.pose.orientation = q_new

        # Update the last position for the next iteration
        self.last_position = [
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z,
        ]

        # self.last_velocity = [odom_msg.twist.twist.linear.x,
        #                       odom_msg.twist.twist.linear.y, 
        #                       odom_msg.twist.twist.linear.z, ] 

        alpha = 0.5  # Adjust the smoothing factor as needed

        for i in range(len(self.last_velocity)):
            self.last_velocity[i] = (alpha * self.last_velocity[i] + self.corrected_current_velocity[i]) / (1+alpha)


        # Publish the odometry data
        self.odom_pub.publish(odom_msg)

    def publish_corrected_imu_data(self):
        # Create and publish a message containing the biases
        bias_msg = Imu()
        bias_msg.angular_velocity.x = self.gyro_bias[0]
        bias_msg.angular_velocity.y = self.gyro_bias[1]
        bias_msg.angular_velocity.z = self.gyro_bias[2]
        bias_msg.linear_acceleration.x = self.accel_bias[0]
        bias_msg.linear_acceleration.y = self.accel_bias[1]
        bias_msg.linear_acceleration.z = self.accel_bias[2]

        self.corrected_imu_pub.publish(bias_msg)

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def quaternion_multiply(self, q1, q2):
        result = Quaternion()
        result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        return result

    def quaternion_normalize(self, q):
        norm = np.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2) 
        if norm != 0.0:
            q.w /= norm
            q.x /= norm
            q.y /= norm
            q.z /= norm
        else:
            # Handle the case where the norm is zero (to avoid division by zero)
            q.w = 1.0
            q.x = 0.0
            q.y = 0.0
            q.z = 0.0
        # q.w /= norm
        # q.x /= norm
        # q.y /= norm
        # q.z /= norm
        return q

    def plot_data(self):
        # Plot mean biases
        self.ax1.plot(self.time_steps, self.mean_gyro_biases, label='Mean Gyro Biases')
        self.ax1.plot(self.time_steps, self.mean_accel_biases, label='Mean Accelerometer Biases')

        # Plot variances
        self.ax2.plot(self.time_steps, self.gyro_variances, label='Gyro Variances')
        self.ax2.plot(self.time_steps, self.accel_variances, label='Accelerometer Variances')

        # Set labels and legends
        self.ax1.set_ylabel('Mean Bias')
        self.ax2.set_ylabel('Variance')
        self.ax2.set_xlabel('Time Steps')
        self.ax1.legend()
        self.ax2.legend()

        # Save the plot to a file (optional)
        # self.fig.savefig('imu_data_plot.png')
        plt.show()

    def on_shutdown(self):
        self.plot_data()
        rospy.loginfo("Shutting down IMUDataProcessor node")

if __name__ == '__main__':
    imu_processor = IMUDataProcessor()

    try:
        # Run until the node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
