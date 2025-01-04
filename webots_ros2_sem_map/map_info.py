import numpy
import logging
import operator
from controller import Display
from nav_msgs.msg import OccupancyGrid, MapMetaData

DIGITAL_EPS = 5
EXTRA_PIXELS = 1

class MapInfo():
    def __add_new_color(self):
        self.__item_colors.append([int(numpy.random.uniform(0, 256)), int(numpy.random.uniform(0, 256)), int(numpy.random.uniform(0, 256))])

    def __item_idx_to_color(self, idx):
        #RGB        
        return self.__item_colors[idx]

    def __add_new_item(self, item_label):
        self.__logger.info(f"{item_label} is a new item, adding to known items list")
        self.__item2idx[item_label] = len(self.__known_items)
        self.__known_items[item_label] = []
        
        if len(self.__known_items) > len(self.__item_colors):
            self.__add_new_color()

    def __init__(self):
        self.__logger = logging.getLogger(__name__)
        self.__logger.setLevel(logging.INFO)

        self.__current_map = None
        self.__current_map_width = 0
        self.__current_map_height = 0
        self.__current_map_x_origin = 0
        self.__current_map_y_origin = 0
        self.__current_map_resolution = 0
        self.__item2idx = {"empty" : 0}
        self.__known_items = { "empty" : [] }

        self.__obj_map = numpy.zeros((self.__current_map_width, self.__current_map_height), dtype="int")
        self.__prob_map = numpy.zeros((self.__current_map_width, self.__current_map_height), dtype="float")

        self.__item_colors = [
            [131, 138, 132],
            [145, 30, 180],
            [70, 240, 240],
            [210, 245, 60],
            [0, 128, 128],
            [170, 110, 40],
            [0, 0, 128],            
            ]
    
    def draw_current_pose(self, display: Display, current_pose : list):
        x, y = self.__digitize_xy(current_pose[0], current_pose[1])
        display.setColor(0xFF0000)
        display.fillOval(x * (1 + EXTRA_PIXELS), y * (1 + EXTRA_PIXELS), 3, 3)

    def draw_current_legend(self, display: Display):
        frame_size = 20
        frame_border = 1

        legend_height = (frame_size + 2*frame_border)*len(self.__known_items)
        legend_width = frame_size + 2*frame_border + 100
        legend_matrix = numpy.zeros((legend_height, legend_width, 3), dtype="uint8")

        legend_matrix[:,:] = [255, 255, 255]
        text_to_print = {}

        idx = 0
        for label in self.__known_items.keys():
            legend_item_top = idx * (frame_size + 2*frame_border)
            legend_item_bottom = legend_item_top + frame_size + 2*frame_border
            
            # item color
            legend_matrix[legend_item_top+frame_border:legend_item_bottom-frame_border, frame_border:frame_size] = self.__item_idx_to_color(self.__item2idx[label])
            
            # black frame
            legend_matrix[legend_item_top:legend_item_top+frame_border, 0:frame_size + frame_border] = [0,0,0] #top
            legend_matrix[legend_item_bottom-frame_border:legend_item_bottom, 0:frame_size + frame_border] = [0,0,0] #bottom

            legend_matrix[legend_item_top:legend_item_bottom, 0:frame_border] = [0,0,0] #left
            legend_matrix[legend_item_top:legend_item_bottom, frame_size:frame_size + frame_border] = [0,0,0] #right

            text_to_print[label] = [ frame_size + 20, (legend_item_top + frame_size/2) - 5]

            idx += 1

        rgb_color_array = numpy.reshape(legend_matrix, 3 * legend_width * legend_height)

        ir = display.imageNew(bytes(rgb_color_array), Display.RGB, legend_width, legend_height)
        
        display.imagePaste(ir, 0, display.getHeight() - legend_height, True)
        display.imageDelete(ir)        

        display.setFont(font="Arial", size=10, anti_aliasing=True)
        display.setColor(0x000000)
        for text in text_to_print.keys():
            display.drawText(text, text_to_print[text][0], display.getHeight() - legend_height + text_to_print[text][1])


    def process_new_map_message(self, msg: OccupancyGrid):
        
        if  msg.info.width != self.__current_map_width or \
            msg.info.height != self.__current_map_height or\
            msg.info.origin.position.x != self.__current_map_x_origin or \
            msg.info.origin.position.y != self.__current_map_y_origin:

            self.__current_map_width = msg.info.width
            self.__current_map_height = msg.info.height
            self.__current_map_resolution = msg.info.resolution
            self.__current_map_x_origin = msg.info.origin.position.x
            self.__current_map_y_origin = msg.info.origin.position.y
            
            self.__x_pos_bins = [ self.__current_map_x_origin + i * self.__current_map_resolution for i in range(self.__current_map_width - 1, -1, -1 )]
            self.__y_pos_bins = [ self.__current_map_y_origin + i * self.__current_map_resolution for i in range(self.__current_map_height - 1)]
            
            self.__build_obj_map()


        self.__current_map = numpy.reshape(msg.data, (self.__current_map_width, self.__current_map_height))

        # self.__print_occupied_slots()

    def __digitize_xy(self, x, y):
        return numpy.digitize(x, self.__x_pos_bins), numpy.digitize(y, self.__y_pos_bins)


    def __update_position_with_item(self, x, y, item_label, confidence):
        x_idx, y_idx = self.__digitize_xy(x, y)
        current_tile_prob = self.__prob_map[x_idx][y_idx]

        neighbors = self.__current_map[ max(x_idx - DIGITAL_EPS, 0) : x_idx + DIGITAL_EPS, max(y_idx - DIGITAL_EPS, 0) : y_idx + DIGITAL_EPS]

        neighbors = numpy.reshape(neighbors, len(neighbors) * len(neighbors[0]))

        highest_chance = max(neighbors)

        occupancy_prob = highest_chance/100

        self.__logger.debug(f"Current tile {x_idx} {y_idx} has a prob of {occupancy_prob} of being occupied and of {current_tile_prob} being a {self.__obj_map[x_idx][y_idx]}")
        # self.__logger.debug(f"Chances of neighbors being occupied:")
        # self.__logger.debug(f"\n{neighbors}")
        self.__logger.debug(f"Highest neighbor chance is: {highest_chance}")
        if confidence*occupancy_prob > current_tile_prob:
            self.__logger.debug(f"Tile has a higher probability of being occupied by {self.__item2idx[item_label]}, switching")
            self.__prob_map[x_idx][y_idx] = confidence*occupancy_prob
            self.__obj_map[x_idx][y_idx] = self.__item2idx[item_label]
            return True
        
        return False

    def add_item_position_info(self, item_label, x, y, confidence):
        if item_label not in self.__known_items.keys():
            self.__add_new_item(item_label)

        if self.__current_map_width > 0:
            if self.__update_position_with_item(x, y, item_label, confidence):
                self.__logger.debug(f"Adding {item_label} in {x} {y} to list of known items with {confidence} confidence")
                self.__known_items[item_label].append([x, y, confidence])
    
    def __build_obj_map(self):
        self.__obj_map = numpy.zeros((self.__current_map_width, self.__current_map_height), dtype="int")
        self.__prob_map = numpy.zeros((self.__current_map_width, self.__current_map_height), dtype="float")

        for item_label in self.__known_items.keys():
            self.__logger.debug(f"Trying to relocate {item_label}")
            item_positions = self.__known_items[item_label]
            
            item_pos_to_remove = []

            for position in item_positions:
                self.__logger.debug(f"Checking if position {position[0]} { position[1]} is {item_label} with confidence {position[2]}")
                if not self.__update_position_with_item(position[0], position[1], item_label, position[2]):
                    self.__logger.debug(f"Marking pos {position[0]} {position[1]} as duplicate")
                    item_pos_to_remove.append(position)

            # remove overlapping positions
            for overlapping_position in item_pos_to_remove:
                self.__logger.debug(f"Removing {overlapping_position} as a duplicate")
                self.__known_items[item_label].remove(overlapping_position)

    def draw_obj_map_for_display(self, display : Display):

        obj_matrix = numpy.zeros((self.__current_map_height * (1 + EXTRA_PIXELS), self.__current_map_width * (1 + EXTRA_PIXELS), 3), dtype="uint8")

        for i in range(self.__current_map_height):
            for j in range (self.__current_map_width):
                # 0 - 0 2
                obj_matrix[i * (1 + EXTRA_PIXELS): (i + 1) * (1 + EXTRA_PIXELS), j * (1 + EXTRA_PIXELS) : (j + 1) * (1 + EXTRA_PIXELS)] = self.__item_idx_to_color(self.__obj_map[j][i])


        rgb_color_array = numpy.reshape(obj_matrix, 3 * self.__current_map_width * self.__current_map_height * (1 + EXTRA_PIXELS) * (1 + EXTRA_PIXELS))

        ir = display.imageNew(bytes(rgb_color_array), Display.RGB, self.__current_map_width * (1 + EXTRA_PIXELS), self.__current_map_height * (1 + EXTRA_PIXELS))
        display.imagePaste(ir, 0, 0, False)
        display.imageDelete(ir)



