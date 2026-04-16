# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from ultralytics import YOLO
# import cv2


# class YoloNode(Node):
#     def __init__(self):
#         super().__init__("yolo_node")

#         self.bridge = CvBridge()
#         self.model = YOLO("yolov8n.pt")

#         self.subscription = self.create_subscription(
#             Image,
#             "/camera/image",
#             self.image_callback,
#             10
#         )

#         self.get_logger().info("Generic YOLO object node started")

#     def image_callback(self, msg):
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
#         except Exception as e:
#             self.get_logger().error(f"CV bridge conversion failed: {e}")
#             return

#         results = self.model(frame, conf=0.5, verbose=False)

#         detections = []
#         display = frame.copy()

#         if results[0].boxes is not None:
#             for box in results[0].boxes:
#                 conf = float(box.conf[0])
#                 x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

#                 detections.append({
#                     "label": "object",
#                     "confidence": conf,
#                     "bbox": [x1, y1, x2, y2]
#                 })

#                 cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
#                 cv2.putText(
#                     display,
#                     f"object {conf:.2f}",
#                     (x1, max(20, y1 - 10)),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     0.6,
#                     (0, 255, 0),
#                     2
#                 )

#         self.get_logger().info(str(detections))

#         cv2.imshow("Generic Object Detection", display)
#         cv2.waitKey(1)


# def main(args=None):
#     rclpy.init(args=args)
#     node = YoloNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n-oiv7.pt")

        self.subscription = self.create_subscription(
            Image,
            "/camera/image",
            self.image_callback,
            10
        )

        self.get_logger().info("YOLO Open Images model started")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CV bridge conversion failed: {e}")
            return

        results = self.model(frame, conf=0.5, verbose=False)
        annotated = results[0].plot()

        detections = []
        if results[0].boxes is not None:
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.model.names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                detections.append({
                    "label": label,
                    "confidence": conf,
                    "bbox": [x1, y1, x2, y2]
                })

        self.get_logger().info(str(detections))

        cv2.imshow("YOLOv8 Open Images", annotated)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()