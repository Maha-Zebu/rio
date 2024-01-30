#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from threading import Lock
import numpy as np
from scipy.integrate import cumtrapz



# Global variables for storing velocity and time data
velocities = []
times = []
average_velocity_data = []

position_estimate = []

lock = Lock()

def calculate_velocity_components(points):
    if len(points) == 0:
        return 0, 0, 0

    # Separate points into X, Y, and velocity arrays
    # points_array = np.array(points)
    # x, y, z = points_array[:, :3]  # Extract X, Y, Z
    x, y, z, intensity, velocity = points

    unit_vector =  calculate_unit_vector(x, y, z)
    # velocity = points_array[:, 4]  # Extract velocity

    vx, vy, vz = unit_vector[0] * velocity, unit_vector[1] * velocity, unit_vector[2] * velocity

    return vx, vy, vz

def calculate_unit_vector(x, y, z):
    magnitude = np.sqrt(x**2 + y**2 + z**2)
    if magnitude == 0:
        return 0, 0, 0  # Avoid division by zero
    unit_vector = x / magnitude, y / magnitude, z / magnitude
    return unit_vector

def pointcloud_callback(msg):
    global velocities, times
    global average_velocity_data


    # Extracting information from the PointCloud2 message
    fields = msg.fields
    data = msg.data

    # Specify the field names to include velocity
    field_names = ["x", "y", "z", "intensity", "velocity"]  # Add "velocity" to the list if it's the correct field name

    # Iterate through the point cloud data and collect velocity data
    gen = pc2.read_points(msg, field_names=field_names, skip_nans=True)
    
    total_velocity = [0.0, 0.0, 0.0]
    point_count = 0

    for point in gen:
        # x, y, z, intensity, velocity = point
        vx, vy, vz = calculate_velocity_components(point)

        total_velocity[0] += vx
        total_velocity[1] += vy
        total_velocity[2] += vz

        point_count += 1

    if point_count > 0:
        average_velocity = np.sqrt(total_velocity[0]**2 + total_velocity[1]**2 + total_velocity[2]**2)/ point_count
        rospy.loginfo(f"Average Velocity: {average_velocity}")


        # Record the current time and velocity
        with lock:
            times.append(rospy.get_time())
            velocities.append(average_velocity)
            # Calculate average velocities along each axis
            average_velocity_data.append((
                np.array(total_velocity)[0]/ point_count,  # Average vx
                np.array(total_velocity)[1]/ point_count,  # Average vy
                np.array(total_velocity)[2]/ point_count   # Average vz
            ))

            if len(times) >= 2: 
                dt  = np.array(times)[-1] - np.array(times)[-2]
            else: dt = 0

            if len(position_estimate) <= 0: 
                position_estimate.append((
                np.array(total_velocity)[0]/ point_count * dt,  # Average vx
                np.array(total_velocity)[1]/ point_count * dt,  # Average vy
                np.array(total_velocity)[2]/ point_count * dt  # Average vz
                ))
            else:
                position_estimate.append((
                    np.array(position_estimate)[-1, 0] + np.array(total_velocity)[0]/ point_count * dt,  # Average vx
                    np.array(position_estimate)[-1, 1] + np.array(total_velocity)[1]/ point_count * dt,  # Average vy
                    np.array(position_estimate)[-1, 2] + np.array(total_velocity)[2]/ point_count * dt  # Average vz
                ))


def animate(fig, ani):
    global velocities, times
    global average_velocity_data
    global position_estimate

    if len(average_velocity_data) ==  0: return

    # print("average velocirt data ", average_velocity_data)

    average_velocity_data_np = np.array(average_velocity_data)
    position_estimate_np = np.array(position_estimate)

    


    # if i >= 10:  # Stop animation after 10 seconds
    #     ani.event_source.stop()

    # Plot the graph on script interruption (Ctrl+C)
    with lock:

        plt.clf()

        # # # Plot individual velocity components
        # # fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True)
        # ax1 = plt.subplot(4, 1, 1)

        # ax1.plot(times, velocities, label='Velocity vs Time')
        # ax1.set_xlabel('Time (s)')
        # ax1.set_ylabel('Average Speed')
        # ax1.legend()

        



        # print(average_velocity_data_np.shape)
        ax2 = plt.subplot(3, 1, 1)

        # Plot average velocity along each axis
        ax2.plot(times, average_velocity_data_np[:, 0], label='Average vx', color="red")
        ax2.plot(times, position_estimate_np[:, 0], label='Estimated px', color="maroon")

        # ax2.plot(times, average_velocity_data[:, 1], label='Average vy')
        # ax2.plot(times, average_velocity_data[:, 2], label='Average vz')
        ax2.set_xlabel('Time (s)')
        # ax2.set_ylabel('Average Velocity Components')
        # ax2.set_title('Average Velocity Components vs Time')
        ax2.grid(True)
        ax2.legend()

        ax3 = plt.subplot(3, 1, 2)

        # Plot average velocity along each axis
        # ax2.plot(times, average_velocity_data[:, 0], label='Average vx')
        ax3.plot(times, average_velocity_data_np[:, 1], label='Average vy', color="green")
        ax3.plot(times, position_estimate_np[:, 1], label='Estimated py', color="magenta")

        # ax2.plot(times, average_velocity_data[:, 2], label='Average vz')
        ax3.set_xlabel('Time (s)')
        # ax3.set_ylabel('Average Velocity Components')
        # ax3.set_title('Average Velocity Components vs Time')
        ax3.grid(True)
        ax3.legend()


        ax4 = plt.subplot(3, 1, 3)

        # Plot average velocity along each axis
        # ax2.plot(times, average_velocity_data[:, 0], label='Average vx')
        # ax2.plot(times, average_velocity_data[:, 1], label='Average vy')
        ax4.plot(times, average_velocity_data_np[:, 2], label='Average vz', color="blue")
        ax4.plot(times, position_estimate_np[:, 2], label='Estimated pz', color="darkcyan")
        ax4.set_xlabel('Time (s)')
        # ax4.set_ylabel('Average Velocity Components')
        # ax4.set_title('Average Velocity Components vs Time')
        ax4.grid(True)
        ax4.legend()



        # plt.show()





# def main():
#     rospy.init_node('pointcloud_reader', anonymous=True)

#     # Specify the PointCloud topic you want to subscribe to
#     pointcloud_topic = "/ti_mmwave/radar_scan_pcl_0"

#     # Subscribe to the PointCloud topic
#     rospy.Subscriber(pointcloud_topic, PointCloud2, pointcloud_callback)

#     # # Set up shutdown callback
#     # rospy.on_shutdown(on_shutdown)

#     # Set up animation
#     fig = plt.figure()
#     ani = FuncAnimation(fig, animate, interval=1000)  # Update every 1 second (1000 milliseconds)


#     # Keep the script running
#     rospy.spin()

if __name__ == '__main__':
    try:
        # main()
        rospy.init_node('pointcloud_reader', anonymous=True)

        # Specify the PointCloud topic you want to subscribe to
        pointcloud_topic = "/ti_mmwave/radar_scan_pcl_0"

        # Subscribe to the PointCloud topic
        rospy.Subscriber(pointcloud_topic, PointCloud2, pointcloud_callback)

        # # Set up shutdown callback
        # rospy.on_shutdown(on_shutdown)

        # Set up animation
        fig = plt.figure()
        ani = FuncAnimation(fig, animate, fargs=(fig,), interval=1000)  # Update every 1 second (1000 milliseconds)
        print("Plot updating...")
        plt.show()


        # Keep the script running
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")
        # plt.show()


#!/usr/bin/env python3

# #!/usr/bin/env python3

# import rospy
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2
# import numpy as np
# from matplotlib import pyplot as plt
# from sklearn.linear_model import RANSACRegressor

# def ransac_filter(points, n_inliers=3):
#     if len(points) < n_inliers:
#         return points
    
#     # Separate points into X, Y, and velocity arrays
#     points_array = np.array(points)
#     XYZ = points_array[:, :3]  # Extract X, Y, Z
#     velocity = points_array[:, 4]  # Extract velocity

#     # Fit RANSAC model
#     model = RANSACRegressor(base_estimator=None, min_samples=n_inliers, residual_threshold=0.1)
#     model.fit(XYZ, velocity)

#     # Mask inliers
#     inlier_mask = model.inlier_mask_
    
#     # Return only inliers
#     return points_array[inlier_mask]

# def pointcloud_callback(msg):
#     global velocity_data
#     # Extracting information from the PointCloud2 message
#     fields = msg.fields
#     width = msg.width
#     height = msg.height
#     point_step = msg.point_step
#     row_step = msg.row_step
#     is_dense = msg.is_dense
#     data = msg.data

#     rospy.loginfo("Received PointCloud message:")
#     rospy.loginfo(f"  Width: {width}, Height: {height}")
#     rospy.loginfo(f"  Point Step: {point_step}, Row Step: {row_step}")
#     rospy.loginfo(f"  Is Dense: {is_dense}")

#     rospy.loginfo("Fields:")
#     for field in fields:
#         rospy.loginfo(f"  Name: {field.name}, Offset: {field.offset}, Datatype: {field.datatype}, Count: {field.count}")

#     # Specify the field names to include velocity
#     field_names = ["x", "y", "z", "intensity", "velocity"]  # Add "velocity" to the list if it's the correct field name

#     # Iterate through the point cloud data and filter outliers
#     gen = pc2.read_points(msg, field_names=field_names, skip_nans=True)
    
#     filtered_points = ransac_filter(list(gen), n_inliers=3)
    
#     time_stamp = rospy.Time.now().to_sec()

#     total_velocity = np.sum(filtered_points[:, 4])
#     point_count = len(filtered_points)

#     if point_count > 0:
#         average_velocity = total_velocity / point_count
#         rospy.loginfo(f"Average Velocity: {average_velocity}")

#         # Store velocity and time information
#         velocity_data.append((time_stamp, average_velocity))

# def plot_velocity_vs_time():
#     global velocity_data
#     if not velocity_data:
#         return

#     velocity_data = np.array(velocity_data)
#     plt.plot(velocity_data[:, 0], velocity_data[:, 1])
#     plt.xlabel('Time (s)')
#     plt.ylabel('Average Velocity')
#     plt.title('Velocity vs Time')
#     plt.show()

# def main():
#     rospy.init_node('pointcloud_reader', anonymous=True)

#     # Specify the PointCloud topic you want to subscribe to
#     pointcloud_topic = "/ti_mmwave/radar_scan_pcl_0"

#     # Subscribe to the PointCloud topic
#     rospy.Subscriber(pointcloud_topic, PointCloud2, pointcloud_callback)

#     try:
#         # Keep the script running
#         rospy.spin()
#     except KeyboardInterrupt:
#         rospy.loginfo("Shutting down")
#         # Plot the velocity vs time graph on Ctrl+C
#         plot_velocity_vs_time()

# if __name__ == '__main__':
#     velocity_data = []  # Global variable to store velocity and time information
#     main()
