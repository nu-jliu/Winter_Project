#!/usr/bin/env python3
import rospy

from ultralytics.engine.results import Results
from ultralytics import YOLO
from cv_bridge import CvBridge

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image


class ObjectDetection:

    def __init__(self):
        self.msg_pointcloud: PointCloud2 = None
        self.msg_image: Image = None
        self.model = YOLO("yolov8n.pt")
        self.bridge = CvBridge()

        self.timer = rospy.Timer(rospy.Duration(secs=0, nsecs=1e7), self.timer_callback)

        self.image_topic = rospy.get_param("image_topic")
        self.points_topic = rospy.get_param("pointcloud_topic")

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

        self.pub_image = rospy.Publisher("detect/image_raw", Image, queue_size=10)

    def timer_callback(self, event):
        rospy.loginfo("Timer event")

        if self.msg_image is not None:
            frame = self.bridge.imgmsg_to_cv2(self.msg_image)
            results: list[Results] = self.model.predict(source=frame, stream=True)

            if results is not None:
                for result in results:
                    boxes = result.boxes
                    rospy.loginfo(f"Got boxes {boxes}")

                    plot_result = result.plot()
                    self.pub_image.publish(self.bridge.cv2_to_imgmsg(plot_result))

    def sub_pointclould_callback(self, msg: PointCloud2):
        self.msg_pointcloud = msg
        # rospy.loginfo(f"Received message {self.msg_pointcloud.header.frame_id}")

    def sub_image_callback(self, msg: Image):
        self.msg_image = msg
        # rospy.loginfo(f"Received message {self.msg_image.header.frame_id}")


def main(args=None):
    rospy.init_node("object_detection")
    node_object_detection = ObjectDetection()
    rospy.spin()


if __name__ == "__main__":
    main()
