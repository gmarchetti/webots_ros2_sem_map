import rclpy
import random
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from transformers import DetrImageProcessor, DetrForObjectDetection
import torch
from PIL import Image
import requests

class ImgDetection(Node):    
    def __init__(self):
        super().__init__("img_recognition")
        # you can specify the revision tag if you don't want the timm dependency
        self.__processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50", revision="no_timm")
        self.__model = DetrForObjectDetection.from_pretrained("facebook/detr-resnet-50", revision="no_timm")

        self.create_subscription(String, 'parse_img', self.__parse_image_callback, 1)

    def __parse_image_callback(self, message):
        self.parse_img_from_array("")

    def parse_img_from_array(self, img_bytes):
        url = "http://images.cocodataset.org/val2017/000000039769.jpg"
        image = Image.open(requests.get(url, stream=True).raw)
        
        inputs = self.__processor(images=image, return_tensors="pt")
        outputs = self.__model(**inputs)

        # convert outputs (bounding boxes and class logits) to COCO API
        # let's only keep detections with score > 0.9
        target_sizes = torch.tensor([image.size[::-1]])
        results = self.__processor.post_process_object_detection(outputs, target_sizes=target_sizes, threshold=0.5)[0]

        for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
            box = [round(i, 2) for i in box.tolist()]
            self.get_logger().info(
                    f"Detected {self.__model.config.id2label[label.item()]} with confidence "
                    f"{round(score.item(), 3)} at location {box}"
                )
            
def main(args=None):
    rclpy.init(args=args)
    sensor = ImgDetection()
    rclpy.spin(sensor)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()