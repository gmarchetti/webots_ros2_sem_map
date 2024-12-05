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

class ImgDetection():    
    def __init__(self):
        # you can specify the revision tag if you don't want the timm dependency
        self.__processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50", revision="no_timm")
        self.__model = DetrForObjectDetection.from_pretrained("facebook/detr-resnet-50", revision="no_timm")

    def parse_img_from_array(self, img_bytes):
        # url = "http://images.cocodataset.org/val2017/000000039769.jpg"
        image = Image.fromarray(img_bytes)
        
        inputs = self.__processor(images=image, return_tensors="pt")
        outputs = self.__model(**inputs)

        # convert outputs (bounding boxes and class logits) to COCO API
        # let's only keep detections with score > 0.9
        target_sizes = torch.tensor([image.size[::-1]])
        results = self.__processor.post_process_object_detection(outputs, target_sizes=target_sizes, threshold=0.5)[0]

        for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
            box = [round(i, 2) for i in box.tolist()]
            print(
                    f"Detected {self.__model.config.id2label[label.item()]} with confidence "
                    f"{round(score.item(), 3)} at location {box}"
                )