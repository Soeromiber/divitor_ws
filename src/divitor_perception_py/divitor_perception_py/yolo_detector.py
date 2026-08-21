#!/usr/bin/env python3

import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        # Parameters
        self.declare_parameter("model_path", "yolo26n.pt")
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("publish_annotated_image", True)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.conf_thresh = (
            self.get_parameter("conf_threshold").get_parameter_value().double_value
        )
        self.pub_annotated = (
            self.get_parameter("publish_annotated_image")
            .get_parameter_value()
            .bool_value
        )

        # Initialize YOLO Model
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        self.model = YOLO(model_path, task="detect")
        self.get_logger().info("YOLO Model loaded successfully.")

        self.bridge = CvBridge()

        # Publishers
        self.det_pub = self.create_publisher(Detection2DArray, "detections", 10)
        if self.pub_annotated:
            self.img_pub = self.create_publisher(Image, "image_annotated", 10)

        # Subscriber using SensorData QoS to prevent queue buildup and input lag on Pi 5
        self.sub = self.create_subscription(
            Image, "image_raw", self.image_callback, qos_profile_sensor_data
        )

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge exception: {e}")
            return

        # Run YOLO inference
        results = self.model.predict(
            source=cv_image, conf=self.conf_thresh, verbose=False
        )[0]

        # Construct vision_msgs/Detection2DArray
        det_array_msg = Detection2DArray()
        det_array_msg.header = msg.header

        for box in results.boxes:
            det = Detection2D()
            det.header = msg.header

            # Extract Bounding Box: Convert (x1, y1, x2, y2) to center (cx, cy) and dimensions (w, h)
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            width = x2 - x1
            height = y2 - y1
            center_x = x1 + (width / 2.0)
            center_y = y1 + (height / 2.0)

            det.bbox.center.position.x = float(center_x)
            det.bbox.center.position.y = float(center_y)
            det.bbox.size_x = float(width)
            det.bbox.size_y = float(height)

            # Class Hypothesis
            hyp = ObjectHypothesisWithPose()
            cls_id = int(box.cls[0].item())
            class_name = (
                self.model.names[cls_id]
                if hasattr(self.model, "names")
                else str(cls_id)
            )

            hyp.hypothesis.class_id = class_name
            hyp.hypothesis.score = float(box.conf[0].item())

            det.results.append(hyp)
            det_array_msg.detections.append(det)

        # Publish bounding box metadata
        self.det_pub.publish(det_array_msg)

        # Publish visual debug image
        if self.pub_annotated:
            # 1. results.plot() outputs RGB; convert to BGR for ROS image standards
            annotated_frame = results.plot()
            annotated_frame = np.ascontiguousarray(annotated_frame, dtype=np.uint8)

            # 2. Use "passthrough" to prevent cv_bridge from throwing KeyError: 16
            annotated_msg = self.bridge.cv2_to_imgmsg(
                annotated_frame, encoding="passthrough"
            )

            annotated_msg.header = msg.header
            self.img_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
