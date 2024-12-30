import numpy
import logging
from nav_msgs.msg import OccupancyGrid, MapMetaData


class MapInfo():
    
    def __item_idx_to_color(self, idx):
        #RGB
        colors = [
            [255, 255, 255],
            [0, 0, 0],
            [255, 255, 0]
        ]
        return colors[idx]

    def __init__(self):
        self.__logger = logging.getLogger(__name__)

        self.__current_map = None
        self.__current_map_width = 0
        self.__current_map_height = 0
        self.__current_map_x_origin = 0
        self.__current_map_y_origin = 0
        self.__current_map_resolution = 0

        self.__known_items = {}

    def process_new_map_message(self, msg: OccupancyGrid):
        self.__current_map_width = msg.info.width
        self.__current_map_height = msg.info.height
        
        self.__current_map = numpy.reshape(msg.data, (self.__current_map_width, self.__current_map_height))
        
        self.__current_map_resolution = msg.info.resolution
        self.__current_map_x_origin = msg.info.origin.position.x
        self.__current_map_y_origin = msg.info.origin.position.y

        self.__x_pos_bins = [ self.__current_map_x_origin + i * self.__current_map_resolution for i in range(self.__current_map_width - 1 )]
        self.__y_pos_bins = [ self.__current_map_y_origin + i * self.__current_map_resolution for i in range(self.__current_map_height - 1)]

    def get_prob_is_xy_occupied(self, x, y):
        if self.__current_map is None:
            return -1

        x_idx = numpy.digitize(x, self.__x_pos_bins)
        y_idx = numpy.digitize(y, self.__y_pos_bins)

        self.__logger.debug(f"Checking if {x} {y} position is occupied -> {x_idx} {y_idx}")

        return self.__current_map[x_idx][y_idx]
    
    def add_item_position_info(self, item_label, x, y):

        if item_label in self.__known_items.keys():
            self.__known_items.append([x, y])
        else:
            self.__known_items = [ [x,y] ]

    def get_obj_map_for_display(self):
        obj_matrix = numpy.zeros((self.__current_map_width, self.__current_map_height, 3), dtype="uint8")

        for i in range(self.__current_map_width):
            for j in range (self.__current_map_height):
                obj_matrix[i][j] = self.__item_idx_to_color(0)

        rgb_color_array = numpy.reshape(obj_matrix, 3 * self.__current_map_width * self.__current_map_height)
        return rgb_color_array


