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

    def __digitize_xy(self, x, y):
        return numpy.digitize(x, self.__x_pos_bins), numpy.digitize(y, self.__y_pos_bins)

    def get_prob_is_xy_occupied(self, x, y):
        if self.__current_map is None:
            return -1

        x_idx, y_idx = self.__digitize_xy(x, y)

        self.__logger.debug(f"Checking if {x} {y} position is occupied -> {x_idx} {y_idx}")

        return self.__current_map[x_idx][y_idx]
    
    def add_item_position_info(self, item_label, x, y):
        
        self.__logger.info(f"Adding {item_label} in {x} {y} to list of known items")
        
        if item_label in self.__known_items.keys():
            self.__known_items[item_label].append([x, y])
        else:
            self.__known_items[item_label] = [ [x,y] ]

    def get_obj_map_for_display(self):
        obj_map = numpy.zeros((self.__current_map_width, self.__current_map_height), dtype="int")

        for item_label in self.__known_items.keys():
            item_positions = self.__known_items[item_label]
            
            item_pos_to_remove = []

            for position in item_positions:
                item_x, item_y = self.__digitize_xy(position[0], position[1])
                if obj_map[item_x][item_y] == 0:
                    self.__logger.info(f"Marking pos {item_x} {item_y} as occupied")
                    obj_map[item_x][item_y] = 1
                else:
                    self.__logger.info(f"Marking pos {item_x} {item_y} as duplicate")
                    item_pos_to_remove.append(position)

            # remove overlapping positions
            for overlapping_position in item_pos_to_remove:
                self.__logger.info(f"Removing {overlapping_position} as a duplicate")
                self.__known_items[item_label].remove(overlapping_position)


        obj_matrix = numpy.zeros((self.__current_map_width, self.__current_map_height, 3), dtype="uint8")

        for i in range(self.__current_map_width):
            for j in range (self.__current_map_height):
                obj_matrix[i][j] = self.__item_idx_to_color(obj_map[i][j])

        rgb_color_array = numpy.reshape(obj_matrix, 3 * self.__current_map_width * self.__current_map_height)
        return rgb_color_array


