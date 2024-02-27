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
        self.depth_image: Image = None
        self.model = YOLO("yolov8n.pt")
        self.bridge = CvBridge()

        self.timer = rospy.Timer(rospy.Duration(secs=0, nsecs=1e7), self.timer_callback)

        self.image_topic = rospy.get_param("image_topic")
        self.points_topic = rospy.get_param("pointcloud_topic")
        self.depth_topic = rospy.get_param("depth_topic", "camera/depth/image_rect_raw")

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

        self.pub_image = rospy.Publisher("detect/image_raw", Image, queue_size=10)

    def timer_callback(self, event):
        rospy.loginfo_once("Timer event")

        if self.msg_image is not None:
            frame = self.bridge.imgmsg_to_cv2(self.msg_image, self.msg_image.encoding)
            results: list[Results] = self.model.predict(source=frame, stream=True)

            if results is not None:
                for result in results:

                    # rospy.loginfo(f"Names: {result.names}")
                    boxes = result.boxes
                    # rospy.loginfo(f"Got boxes {boxes.xyxy.}")
                    for box in boxes:
                        # rospy.loginfo(f"cls: {box.cls}")
                        # rospy.loginfo(f"{box.xyxy.numpy()}")
                        coordinates = box.xywh.numpy()[0]
                        x = coordinates[0]
                        y = coordinates[1]
                        # w = coordinates[2]
                        # h = coordinates[3]

                        # rospy.loginfo(f"X: {x}")
                        # rospy.loginfo(f"Y: {y}")
                        # rospy.loginfo(f"W: {w}")
                        # rospy.loginfo(f"H: {h}")

                        # rospy.loginfo(f"width: {self.depth_image.width}")
                        # rospy.loginfo(f"height: {self.depth_image.height}")
                        # rospy.loginfo(f"Step: {self.depth_image.step}")
                        # rospy.loginfo(f"size: {len(self.depth_image.data)}")

                        x_depth = x * self.depth_image.width / self.msg_image.width
                        y_depth = y * self.depth_image.height / self.msg_image.width

                        # rospy.loginfo(f"X depth: {x_depth}")
                        # rospy.loginfo(f"Y depth: {y_depth}")

                        col_first = round(x_depth * 2 - 0.5)
                        col_second = round(x_depth * 2 + 0.5)
                        row = round(y_depth)

                        depth_first = self.depth_image.data[
                            row * self.depth_image.step + col_first
                        ]
                        depth_second = self.depth_image.data[
                            row * self.depth_image.step + col_second
                        ]

                        # rospy.loginfo(f"Depth 1: {depth_first}")
                        # rospy.loginfo(f"Depth 2: {depth_second}")

                        rospy.loginfo(
                            f"Depth in meters: {result.names[int(box.cls)]} -> {float(max(depth_first, depth_second)) / 1000.0}"
                        )

                        # self.depth_image.data
                    # rospy.loginfo(f"Got boxes {boxes.xyxy}")
                    # rospy.loginfo(f"Got boxes {boxes.xyxy}")
                    # rospy.loginfo(f"Got boxes {boxes.xyxy}")

                    # boxes.xyxy[0]

                    plot_result = result.plot()
                    self.pub_image.publish(self.bridge.cv2_to_imgmsg(plot_result))

    def sub_pointclould_callback(self, msg: PointCloud2):
        self.msg_pointcloud = msg
        # rospy.loginfo(f"Received message {self.msg_pointcloud.header.frame_id}")

    def sub_image_callback(self, msg: Image):
        self.msg_image = msg
        # rospy.loginfo(f"Received message {self.msg_image.header.frame_id}")

    def sub_depth_image_callback(self, msg: Image):
        self.depth_image = msg


def main(args=None):
    rospy.init_node("object_detection")
    node_object_detection = ObjectDetection()
    rospy.spin()


if __name__ == "__main__":
    main()
