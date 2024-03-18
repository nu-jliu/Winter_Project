#!/usr/bin/env python3
import rospy

# from ultralytics.engine.results import Results
# from ultralytics import YOLO
# from cv_bridge import CvBridge

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped

# import tf
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class ObjectDetection:

    def __init__(self):
        self.msg_pointcloud: PointCloud2 = None
        self.msg_image: Image = None
        self.depth_image: Image = None
        # self.model = YOLO("yolov8n.pt")
        # self.bridge = CvBridge()

        self.timer = rospy.Timer(rospy.Duration(secs=0, nsecs=1e7), self.timer_callback)

        self.image_topic = rospy.get_param("image_topic")
        self.points_topic = rospy.get_param("pointcloud_topic")
        self.depth_topic = rospy.get_param(
            "depth_topic", "realsense/depth/image_rect_raw"
        )

        self.sub_pointcloud = rospy.Subscriber(
            self.points_topic,
            PointCloud2,
            self.sub_pointclould_callback,
            queue_size=10,
        )
        self.sub_image = rospy.Subscriber(
            self.image_topic,
            Image,
            self.sub_image_callback,
            queue_size=10,
        )
        self.sub_depth_image = rospy.Subscriber(
            self.depth_topic,
            Image,
            self.sub_depth_image_callback,
            queue_size=10,
        )

        self.pub_image = rospy.Publisher("/robot/head_display", Image, queue_size=10)

        self.tf_broadcaster = TransformBroadcaster()
        self.tf_static_broadcaster = StaticTransformBroadcaster()

        tf_static = TransformStamped()

        tf_static.header.stamp = rospy.Time.now()
        tf_static.header.frame_id = "base"
        tf_static.child_frame_id = "d435_front_link"
        tf_static.transform.translation.x = 205e-3  # - 0.011
        tf_static.transform.translation.y = -50e-3  # - 0.018
        tf_static.transform.translation.z = 120e-3  # - 0.013

        tf_static.transform.rotation.x = 0.0
        tf_static.transform.rotation.y = 0.0
        tf_static.transform.rotation.z = 0.0
        tf_static.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(tf_static)

    def timer_callback(self, event):
        rospy.loginfo_once("Timer event")

        # if self.msg_image is not None:
        #     frame = self.bridge.imgmsg_to_cv2(self.msg_image, self.msg_image.encoding)
        #     results: list[Results] = self.model.predict(source=frame, stream=True)

        #     if results is not None:
        #         for result in results:
        #             boxes = result.boxes

        #             if self.depth_image is not None:
        #                 for box in boxes:
        #                     coordinates = box.xywh.numpy()[0]
        #                     x = coordinates[0]
        #                     y = coordinates[1]

        #                     x_depth = x * self.depth_image.width / self.msg_image.width
        #                     y_depth = y * self.depth_image.height / self.msg_image.width

        #                     col_first = round(x_depth * 2 - 0.5)
        #                     col_second = round(x_depth * 2 + 0.5)
        #                     row = round(y_depth)

        #                     depth_first = self.depth_image.data[
        #                         row * self.depth_image.step + col_first
        #                     ]
        #                     depth_second = self.depth_image.data[
        #                         row * self.depth_image.step + col_second
        #                     ]

        #                     depth = float(max(depth_first, depth_second)) * 0.001
        #                     rospy.loginfo(
        #                         f"Depth in meters: {result.names[int(box.cls)]} -> {depth}"
        #                     )

        #                     # self.tf_broadcaster.sendTransform(
        #                     #     [
        #                     #         (x_depth - self.depth_image.width / 2.0)
        #                     #         * 0.0002645833,
        #                     #         (y_depth - self.depth_image.height / 2.0)
        #                     #         * 0.0002645833,
        #                     #         depth,
        #                     #     ],
        #                     #     [0.0, 0.0, 0.0, 1.0],
        #                     #     rospy.Time.now(),
        #                     #     f"object_{result.names[int(box.cls)]}",
        #                     #     "camera_depth_optical_frame",
        #                     # )
        #                     tf_obj = TransformStamped()
        #                     tf_obj.header.stamp = rospy.Time.now()
        #                     tf_obj.header.frame_id = "camera_depth_optical_frame"
        #                     tf_obj.child_frame_id = (
        #                         f"object_{result.names[int(box.cls)]}"
        #                     )
        #                     tf_obj.transform.translation.x = (
        #                         x_depth - self.depth_image.width / 2.0
        #                     ) * 0.0002645833
        #                     tf_obj.transform.translation.y = (
        #                         y_depth - self.depth_image.height / 2.0
        #                     ) * 0.0002645833
        #                     tf_obj.transform.translation.z = depth

        #                     tf_obj.transform.rotation.x = 0.0
        #                     tf_obj.transform.rotation.y = 0.0
        #                     tf_obj.transform.rotation.z = 0.0
        #                     tf_obj.transform.rotation.w = 1.0

        #                     self.tf_broadcaster.sendTransform(tf_obj)

        #             plot_result = result.plot()
        #             self.pub_image.publish(self.bridge.cv2_to_imgmsg(plot_result))

    def sub_pointclould_callback(self, msg: PointCloud2):
        self.msg_pointcloud = msg
        # rospy.loginfo(f"Received message {self.msg_pointcloud.header.frame_id}")

    def sub_image_callback(self, msg: Image):
        self.msg_image = msg
        self.pub_image.publish(msg)
        # rospy.loginfo(f"Received message {self.msg_image.header.frame_id}")

    def sub_depth_image_callback(self, msg: Image):
        self.depth_image = msg


def main(args=None):
    rospy.init_node("object_detection")
    node_object_detection = ObjectDetection()
    rospy.spin()


if __name__ == "__main__":
    main()
