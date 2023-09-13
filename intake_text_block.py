"""Provides a scripting component.
    Inputs:
        x: The x script variable
        y: The y script variable
    Output:
        a: The a output variable"""

__author__ = "gregb"
__version__ = "2023.02.23"

import Rhino.Geometry as rg

class room_node(object):
    def __init__(self, name, area, max_dim, min_dim, height, color, adjacencies):
        self.name = name                    # string
        self.area = area                    # float or None for circulation/hallway
        self.max_dim = max_dim              # float or None
        self.min_dim = min_dim              # float or None
        self.height = height                # boolean
        self.adjacencies = adjacencies      # set of adjacent room_nodes
        self.rectangle = None               # rectangle representation of node
        self.edges = []                  # set of edges of graph
        self.bt_count = 0                   # number of back tracking count
        self.triangles = []
        self.force_vector = None
        self.color = color
        
    def update_adjacencies(self, room_node_list):
        updated_adj_list = []
        adj_dict = dict()
        for room in self.adjacencies:
            if type(room) == str:
                for node in room_node_list:
                    if node.name == room:
                        adj_dict[room] = node
                        updated_adj_list.append(node)
            elif type(room) == room_node:
                adj_dict[room.name] = room
                updated_adj_list.append(room)
        self.adjacencies = adj_dict
        
    def print_adj_nodes(self):
        for node in self.adjacencies:
            print("          " + self.name + " is in " + node.name + "'s list")
            
    def check_adjacencies(self):
        for other in self.adjacencies:
            if not (self.name in self.adjacencies.get(other).adjacencies):
                print(self.name + " not in " + other + "'s Dictionary")

# Room list enters as text block and returns as a list of room_nodes
def intake_program_list(program):
    # set up emty list of room_node objects
    room_node_list = []
    
    # loop through program list and set up values to initiate room_node
    for i in range(1, len(program)):
        
        values = program[i].split(", ")
        
        name = values[0]
        
        area =      None if values[1] == "None" else float(values[1])
        
        max_dim =   None if values[2] == "None" else True
        
        min_dim =   None if values[3] == "None" else float(values[3])
        
        height = float(values[4])
        
        color = str(values[5])
        
        adj_str = str(values[6:])[1:-1]
        adjacencies = []
        for room in adj_str.split(", "):
            adjacencies.append(room[1:-1])
        adjacencies[0] = adjacencies[0][1:]
        adjacencies[-1] = adjacencies[-1][:-1]
        room_node_list.append(room_node(name, area, max_dim, min_dim, height, color, adjacencies))
        
    return room_node_list

def add_area(room_node_list):
    total = 0
    for room_node in room_node_list:
        if room_node.area != None:
            total += room_node.area
    return total

def program_list(room_node_list):
    program_list = []
    for room_node in room_node_list:
        program_list.append(room_node.name)
    return program_list

def room_adjacencies(room_node_list):
    program_list = []
    for room_node in room_node_list:
        program_list.append(str(room_node.name) + ": " + str(room_node.adjacencies.keys()))
    return program_list

def update_adjacencies(room_node_list):
    for room_node in room_node_list:
        room_node.update_adjacencies(room_node_list)
    for room_node in room_node_list:
        room_node.check_adjacencies()
        
def convert_adjacencies_to_set(room_node_list):
    for room in room_node_list:
        adj_set = set()
        for key in room.adjacencies:
            adj_set.add(room.adjacencies[key])
        room.adjacencies = adj_set

room_node_list = intake_program_list(program)
total_area = add_area(room_node_list)
room_list = program_list(room_node_list)
update_adjacencies(room_node_list)
room_adjacencies = room_adjacencies(room_node_list)

convert_adjacencies_to_set(room_node_list)

print(room_node_list[0].adjacencies)