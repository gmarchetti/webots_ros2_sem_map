import torch
import logging
import os
import numpy as np
from controller import Camera
from transformers import DetrImageProcessor, DetrForObjectDetection
from PIL import Image

class CameraImgHandler():

    def __get_camera_array_as_image(self) -> Image:
        cam_image = self.__camera.getImage()
        width = self.__camera.getWidth()
        height = self.__camera.getHeight()
        
        np_image_array = np.frombuffer(cam_image, dtype=np.uint8)
        bgr_array = np_image_array.reshape((height, width, 4))[:, :, :3]
        rgb_array = bgr_array[:,:,::-1]                        
        img = Image.fromarray(rgb_array)

        return img

    def __init__(self, camera : Camera):
        # you can specify the revision tag if you don't want the timm dependency
        self.__processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50", revision="no_timm")
        self.__model = DetrForObjectDetection.from_pretrained("facebook/detr-resnet-50", revision="no_timm")
        self.__logger = logging.getLogger(__name__)
        self.__camera = camera

    def save_camera_to_file(self):
        img = self.__get_camera_array_as_image()
        img.save("screenshot.jpg")            
        self.__logger.info(f"Image saved to {os.path.abspath(os.path.join(os.curdir, 'screenshot.jpg'))}")

    def __item_box_to_rad(self, item_box):
        cam_width = self.__camera.getWidth()
        cam_fov = self.__camera.getFov()

        x_min, x_max = int(item_box[0]), int(item_box[2])

        rel_x_min = float(x_min)/cam_width
        rel_x_max = float(x_max)/cam_width

        angle_min = (rel_x_min * cam_fov) - (cam_fov/2)
        angle_max = (rel_x_max * cam_fov) - (cam_fov/2)

        return angle_min, angle_max
        
    def parse_current_camera(self):
        img = self.__get_camera_array_as_image()
        
        self.__logger.info(f"Read img from camera")        
        self.__logger.info(f"Camera fov: {self.__camera.getFov()}")
        items_in_sight = self.parse_img(img)

        for item in items_in_sight:
            item_box = item["box"]
            angle_min, angle_max = self.__item_box_to_rad(item_box)
            item["angle_pos"] = [angle_min, angle_max]
            self.__logger.info(f"{item["label"]} position in rads: {angle_min} - {angle_max}")
        
        return items_in_sight
            

    def parse_img_from_file(self, file_name : str):
        self.__logger.info(f"Parsing image from {file_name}")
        image = Image.open(file_name)
        self.parse_img(image)

    def parse_img(self, image : Image):        
        inputs = self.__processor(images=image, return_tensors="pt")
        outputs = self.__model(**inputs)

        # convert outputs (bounding boxes and class logits) to COCO API
        # let's only keep detections with score > 0.9
        target_sizes = torch.tensor([image.size[::-1]])
        results = self.__processor.post_process_object_detection(outputs, target_sizes=target_sizes, threshold=0.2)[0]

        items = []

        # Box in COCO format top_left_x, top_left_y,bottom_right_x, bottom_right_y
        for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
            box = [round(i, 2) for i in box.tolist()]

            item = {
                "label" :  self.__model.config.id2label[label.item()],
                "confidence" : round(score.item(), 3),
                "box" : box
            }

            items.append(item)
        
        self.__logger.info(f"Possibly found {len(items)} items")

        for found_item in items:
            self.__logger.info(
                f"Detected {found_item["label"]} with confidence {found_item["confidence"]} at location {found_item["box"]}"
            )
        
        self.__logger.info("Finished parsing image")

        return items