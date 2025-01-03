import numpy
import logging
from controller import Display
from nav_msgs.msg import OccupancyGrid, MapMetaData

DIGITAL_EPS = 2

class MapInfo():
    def __add_new_color(self):
        self.__item_colors.append([int(numpy.random.uniform(0, 256)), int(numpy.random.uniform(0, 256)), int(numpy.random.uniform(0, 256))])

    def __item_idx_to_color(self, idx):
        #RGB        
        return self.__item_colors[idx]

    def __add_new_item(self, item_label, x, y, confidence):
        self.__logger.info(f"{item_label} is a new item, adding to known items list")
        self.__item2idx[item_label] = len(self.__known_items)
        self.__known_items[item_label] = [ [x,y, confidence] ]
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

        self.__item_colors = [[255, 255, 255],]
    
    def draw_current_pose(self, display: Display, current_pose : list):
        x, y = self.__digitize_xy(current_pose[0], current_pose[1])
        display.setColor(0xFF0000)
        display.fillOval(x, y, 3, 3)

    def draw_current_legend(self, display: Display):
        frame_size = 20
        frame_border = 1

        legend_height = (frame_size + 2*frame_border)*len(self.__known_items)
        legend_width = frame_size + 2*frame_border + 100
        legend_matrix = numpy.zeros((legend_height, legend_width, 3), dtype="uint8")
        
        self.__logger.debug(f"Legend dimensions: {legend_height} {legend_width}")

        legend_matrix[:,:] = [255, 255, 255]
        text_to_print = {}

        idx = 0
        for label in self.__known_items.keys():
            self.__logger.debug(f"Color for {label} is {self.__item_idx_to_color(self.__item2idx[label])}")
            legend_item_top = idx * (frame_size + 2*frame_border)
            legend_item_bottom = legend_item_top + frame_size + 2*frame_border
            
            # item color
            self.__logger.debug(f"Legend item borders {(legend_item_top)}:{legend_item_bottom}")
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
        
        if msg.info.width != self.__current_map_width or msg.info.height != self.__current_map_height:
            self.__current_map_width = msg.info.width
            self.__current_map_height = msg.info.height
            self.__current_map_resolution = msg.info.resolution
            self.__current_map_x_origin = msg.info.origin.position.x
            self.__current_map_y_origin = msg.info.origin.position.y
            self.__x_pos_bins = [ self.__current_map_x_origin + i * self.__current_map_resolution for i in range(self.__current_map_width - 1, -1, -1 )]
            self.__y_pos_bins = [ self.__current_map_y_origin + i * self.__current_map_resolution for i in range(self.__current_map_height - 1)]

        self.__current_map = numpy.reshape(msg.data, (self.__current_map_width, self.__current_map_height))
        # self.__print_occupied_slots()

    def __digitize_xy(self, x, y):
        return numpy.digitize(x, self.__x_pos_bins), numpy.digitize(y, self.__y_pos_bins)

    def get_prob_is_xy_occupied(self, x, y):
        if self.__current_map is None:
            return -1

        x_idx, y_idx = self.__digitize_xy(x, y)

        self.__logger.debug(f"Checking if {x} {y} position is occupied -> {x_idx} {y_idx}")

        xs_to_search = range(max(0, x_idx - DIGITAL_EPS), min(x_idx + DIGITAL_EPS +1, self.__current_map_width))
        ys_to_search = range(max(0, y_idx - DIGITAL_EPS), min(y_idx + DIGITAL_EPS +1, self.__current_map_height))

        max_prob = -1

        for x_eps in xs_to_search:
            for y_eps in ys_to_search:
                if self.__current_map[x_eps][y_eps] > max_prob:
                    max_prob = self.__current_map[x_eps][y_eps]
        
        return max_prob

    def add_item_position_info(self, item_label, x, y, confidence):
        
        self.__logger.debug(f"Adding {item_label} in {x} {y} to list of known items with {confidence} confidence")
        
        if item_label in self.__known_items.keys():
            self.__known_items[item_label].append([x, y, confidence])
        else:
            self.__add_new_item(item_label, x, y, confidence)
            

    def get_obj_map_for_display(self):
        obj_map = numpy.zeros((self.__current_map_width, self.__current_map_height), dtype="int")
        highest_prob = numpy.zeros((self.__current_map_width, self.__current_map_height), dtype="int")

        for item_label in self.__known_items.keys():
            item_positions = self.__known_items[item_label]
            
            item_pos_to_remove = []

            for position in item_positions:
                item_x, item_y = self.__digitize_xy(position[0], position[1])
                if highest_prob[item_x][item_y] < position[2]:
                    self.__logger.debug(f"Marking pos {item_x} {item_y} as occupied")
                    obj_map[item_x][item_y] = self.__item2idx[item_label]
                    highest_prob[item_x][item_y] = position[2]
                else:
                    self.__logger.debug(f"Marking pos {item_x} {item_y} as duplicate")
                    item_pos_to_remove.append(position)

            # remove overlapping positions
            for overlapping_position in item_pos_to_remove:
                self.__logger.debug(f"Removing {overlapping_position} as a duplicate")
                self.__known_items[item_label].remove(overlapping_position)

        obj_matrix = numpy.zeros((self.__current_map_height, self.__current_map_width, 3), dtype="uint8")

        for i in range(self.__current_map_height):
            for j in range (self.__current_map_width):
                obj_matrix[i][j] = self.__item_idx_to_color(obj_map[j][i])


        rgb_color_array = numpy.reshape(obj_matrix, 3 * self.__current_map_width * self.__current_map_height)

        return rgb_color_array


