#! /usr/bin/env python

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import rospy
import time
import pcl
import ctypes
import struct
import rospkg
import tf


rospack = rospkg.RosPack()
path = rospack.get_path('point_cloud_solution')
# def callback_pointcloud(data):
#     assert isinstance(data, PointCloud2)
#     gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
#     time.sleep(1)
#     print(type(gen))
#     for p in gen:
#       print(" x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2]))

def main(cloud_msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 2),
                     tf.transformations.quaternion_from_euler(90, 0, 0),
                     rospy.Time.now(),
                     "/main_link",
                     "/world")
    rospy.loginfo_once("Publishing The cloud and TF")
    pub.publish(cloud_msg)


def pcl_to_ros(pcl_array):
    """ Converts a pcl PointXYZRGB to a ROS PointCloud2 message

        Args:
            pcl_array (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud

        Returns:
            PointCloud2: A ROS point cloud
    """
    ros_msg = PointCloud2()

    ros_msg.header.stamp = rospy.Time(0)
    ros_msg.header.frame_id = "main_link"

    ros_msg.height = 1
    ros_msg.width = pcl_array.size

    ros_msg.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="rgb",
                            offset=16,
                            datatype=PointField.FLOAT32, count=1))

    ros_msg.is_bigendian = False
    ros_msg.point_step = 32
    ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
    ros_msg.is_dense = False
    buffer = []

    for data in pcl_array:
        s = struct.pack('>f', data[3])
        i = struct.unpack('>l', s)[0]
        pack = ctypes.c_uint32(i).value

        r = (pack & 0x00FF0000) >> 16
        g = (pack & 0x0000FF00) >> 8
        b = (pack & 0x000000FF)

        buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))

    ros_msg.data = "".join(buffer)

    return ros_msg






if __name__ == "__main__":
    # print(path)
    cloud = pcl.load_XYZRGB(path + "/scripts/single_pointcloud_msg.pcd", format="pcd")
    # cloud = pcl.load_XYZRGB(path + "/scripts/400Kpoints.pcd", format="pcd")
    rospy.init_node("pcl_msg_pub", anonymous=True)
    pub = rospy.Publisher('/msg/points', PointCloud2, queue_size=10)
    cloud_msg_ros = pcl_to_ros(cloud)
    print(cloud_msg_ros._type)
    while not rospy.is_shutdown():
        main(cloud_msg_ros)