import xml.etree.ElementTree as ET
from math import sin, cos, pi
import numpy as np 

def generate_block(block_id, size, position, orientation):
    block = ET.Element("block")
    block.set("id", block_id)
    block.set("size", size)
    block.set("movable", "false")
    
    body = ET.SubElement(block, "body")
    body.set("position", position)
    body.set("orientation", orientation)
    
    leds = ET.SubElement(block, "leds")
    leds.set("medium","leds")
    led_0 = ET.SubElement(leds, "led")
    led_0.set("id", "led_0")
    led_0.set("offset", " 0.0075, 0.0625, 0.067")
    led_0.set("anchor", "origin")
    led_0.set("color", "red")
    
    led_1 = ET.SubElement(leds, "led")
    led_1.set("id", "led_1")
    led_1.set("offset", " 0.0075, -0.0625, 0.067")
    led_1.set("anchor", "origin")
    led_1.set("color", "green")
    
    return block

def generate_led(led_id, offset, color):

    led = ET.Element("led")
    led.set("id", led_id)
    led.set("offset", offset)
    led.set("anchor", "origin")
    led.set("color", color)
    
    return led

def generate_blocks(number_edges,number_boxes_per_edge, lenght_boxes):
    blocks = []

    initial_position_x = (number_boxes_per_edge/2)*lenght_boxes - 0.125
    initial_position_y = (number_boxes_per_edge/2)*lenght_boxes

    angle = -180*(number_edges-2)/number_edges
    # angle = -90
    wall_angle = angle
    c = 0
    for i in range(number_edges):
        for n in range(number_boxes_per_edge):
            if c < 10:
                block_id = "block_0" + str(c)
            else:
                block_id = "block_" + str(c)
            size = f"0.01,{lenght_boxes},0.1" # default size, you can change it as needed
            if c != 0:
                x_position = initial_position_x - (n*lenght_boxes)*sin(-wall_angle*pi/180)
                y_position = initial_position_y - (n*lenght_boxes)*cos(-wall_angle*pi/180)
            else:
                x_position = initial_position_x
                y_position = initial_position_y
            position = f"{x_position},{y_position},0" # default position, you can change it as needed
            orientation = f"{wall_angle},0,0"
            block = generate_block(block_id, size, position, orientation)
            blocks.append(block)

            c += 1
        print(wall_angle)
        wall_angle += -angle
        if i==0:
            initial_position_x = x_position - lenght_boxes/2
            initial_position_y = y_position - lenght_boxes/2
        elif i == 1:
            initial_position_x = x_position + (lenght_boxes/2)
            initial_position_y = y_position - (lenght_boxes/2)
        elif i == 2:
            initial_position_x = x_position + (lenght_boxes/2)
            initial_position_y = y_position + (lenght_boxes/2)

    return blocks


def generate_leds(x_size, y_size, led_size):
    leds = []

    id = 1000

    x_leds_array = np.arange(-x_size,x_size,led_size)
    y_leds_array = np.arange(-y_size,y_size,led_size)

    for x_led in x_leds_array:
        for y_led in y_leds_array:
            offset = f"{x_led},{y_led},0.0001"
            led_id = f"led_{id}"
            id += 1
            color="white"
            led = generate_led(led_id, offset, color)
            leds.append(led)

    print(len(leds))
    return leds


def indent(elem, level=0):
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

def write_elements_to_xml(elements, upper_element, filename):
    root = ET.Element(upper_element)
    for element in elements:
        root.append(element)
    
    tree = ET.ElementTree(root)
    indent(root)
    tree.write(filename)


def modify_arena_element(xml_file, element_name, new_element):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    arena = root.find('arena')
    if arena is not None:
        elements = arena.findall(element_name)
        for element in elements:
            if element is not None:
                arena.remove(element)
    for element in new_element:
        arena.append(element)
    indent(root)
    tree.write(xml_file)

def modify_phormica_element(xml_file, element_name, new_element):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    arena = root.find('arena')
    phormica = arena.find('phormica')
    leds = phormica.find('leds')
    if leds is not None:
        elements = leds.findall(element_name)
        print("lends")
        for element in elements:
            if element is not None:
                leds.remove(element)
    for element in new_element:
        leds.append(element)
    indent(root)
    tree.write(xml_file)


number_edges=4
number_boxes_per_edge=6
lenght_boxes=0.25

# blocks = generate_blocks(number_edges, number_boxes_per_edge, lenght_boxes)
# element_name="block"
# modify_arena_element("chain_robots.argos",element_name, blocks)

led_size = 0.03
x_size = lenght_boxes*number_boxes_per_edge/2 # Half of the total size to init in the center :)
y_size = x_size
leds = generate_leds(1, 1, led_size)
element_name="led"
modify_phormica_element("/home/robotmaster/argos3-installation/habanero/habanero-loopfunctions/scenarios/heterogeneity/chain_robots.argos",element_name, leds)