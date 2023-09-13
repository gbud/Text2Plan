"""Provides a scripting component.
    Inputs:
        x: The x script variable
        y: The y script variable
    Output:
        a: The a output variable"""

__author__ = "gregb"
__version__ = "2023.04.09"

import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import math

def factor_from_cosine(cosine):
    return (1 - cosine) / 2

def distance(p1, p2):
    return p1.DistanceTo(p2)

def calc_att_vector(room, rn_list, cnt_pt):
    
    vectors = []
    sorted_room_nodes = sorted(room.adjacencies, key=lambda rn: rn.rectangle.Center.DistanceTo(cnt_pt))
    
    for other in sorted_room_nodes:
        vec = other.rectangle.Center - room.rectangle.Center
        vectors.append(vec)
    
    #sort by most pointing at center
    ref_vector = cnt_pt - room.rectangle.Center
    for v in vectors:
        cosine = ref_vector * v / (ref_vector.Length * v.Length)
        factor = factor_from_cosine(cosine)
        v *= factor
    
    # sum force vectors and average
    force_vector = (sum(vectors, rg.Vector3d(0,0,0))) / len(vectors)
    room.force_vector = force_vector

def angle_key(line):
    vector = line.To - line.From
    return math.atan2(vector.Y, vector.X)

def calc_rep_vector(room, rn_list):
    
    vectors = []
    int_found = False
    
    for other in rn_list:
        if room !=  other:
            
            #check if rooms intersect
            room_pline  = room.rectangle.ToNurbsCurve()
            other_pline = other.rectangle.ToNurbsCurve()
            if rs.CurveCurveIntersection(room_pline, other_pline):
                int_found = True
                vec = room.rectangle.Center - other.rectangle.Center
                vectors.append(vec)
            
            #check if room is within another
            other_box = other.rectangle.BoundingBox
            in_box_flag = True
            for i in range(4):
                if not other_box.Contains(room.rectangle.Corner(i)):
                    in_box_flag = False
                    break
            if in_box_flag:
                vec = room.rectangle.Center - other.rectangle.Center
                vectors.append(vec * 1.5)
            
            #check edge angles arnt too wide
            if len(room.edges) == 2:
                ordered_edges = sorted(room.edges, key=angle_key)
                for i in range(len(ordered_edges) - 1):
                    dot_product = ordered_edges[i].Direction * ordered_edges[i+1].Direction
                    angle = math.acos(max(min(dot_product, 1), -1))
                    if math.degrees(angle) > 135:
                        v_1_rev = ordered_edges[i].Direction
                        v_2_rev = ordered_edges[i+1].Direction
                        v_1_rev.Reverse()
                        v_2_rev.Reverse()
                        avg_rev = (v_1_rev +v_2_rev) / 2
                        vectors.append(avg_rev * 0.5)
            
    if int_found == True:
        force_vector = (sum(vectors, rg.Vector3d(0,0,0))) / len(vectors)
        room.force_vector = force_vector
    else:
        room.force_vector = []

def displace_rectangle(room, vector):
    old_corner_0 = room.rectangle.Corner(0)
    old_corner_2 = room.rectangle.Corner(2)
    new_corner_0 = old_corner_0 + vector
    new_corner_2 = old_corner_2 + vector
    new_rectangle = rg.Rectangle3d(room.rectangle.Plane, new_corner_0, new_corner_2)
    room.rectangle = new_rectangle
    room.force_vector = []

def center_rectangles(rn_list, pt):
    x_lo = None
    x_hi = None
    y_lo = None
    y_hi = None
    for room in rn_list:
        curr_x_lo = room.rectangle.Corner(0).X
        curr_x_hi = room.rectangle.Corner(2).X
        curr_y_lo = room.rectangle.Corner(0).Y
        curr_y_hi = room.rectangle.Corner(2).Y
        if curr_x_lo < x_lo or x_lo == None:
            x_lo = curr_x_lo
        if curr_x_hi > x_hi or x_hi == None:
            x_hi = curr_x_hi
        if curr_y_lo < y_lo or y_lo == None:
            y_lo = curr_y_lo
        if curr_y_hi > y_hi or y_hi == None:
            y_hi = curr_y_hi
    mid_x = (x_lo + x_hi) / 2
    mid_y = (y_lo + y_hi) / 2
    mid_pt = rg.Point3d(mid_x, mid_y, 0)
    shift_vec = pt - mid_pt
    for room in rn_list:
        displace_rectangle(room, shift_vec)
    #snap_rectangles(room_nodes)
    for room in rn_list:
            update_edges(room)

def update_edges(room):
    edges = []
    for other in room.adjacencies:
        edges.append(rg.Line(room.rectangle.Center, other.rectangle.Center))
    room.edges = edges

def snap_rectangles(rn_list):
    for room in rn_list:
        floor_0_X = math.floor(room.rectangle.Corner(0).X)
        floor_0_Y = math.floor(room.rectangle.Corner(0).Y)
        floor_corner_0 = rg.Point3d(floor_0_X, floor_0_Y, 0)
        
        floor_2_X = math.floor(room.rectangle.Corner(2).X)
        floor_2_Y = math.floor(room.rectangle.Corner(2).Y)
        floor_corner_2 = rg.Point3d(floor_2_X, floor_2_Y, 0)
        
        floor_rec = rg.Rectangle3d(room.rectangle.Plane, floor_corner_0, floor_corner_2)
        room.rectangle = floor_rec
    for room in rn_list:
        update_edges(room)

def max_displacement(room_nodes_new):
    max_displacement = None
    for room in room_nodes_new:
        if room.force_vector.Length < max_displacement or max_displacement == None:
            max_displacement = room.force_vector.Length
    return max_displacement


def move_recs(rn_list, iter_check, center_pt, cooling_rate):
    if iter_check.count > 1:
        
        # calc attract forces
        iter_check.max_displacement = None
        curr_max = None
        for room in rn_list:
            calc_att_vector(room, rn_list, center_point)
            if iter_check.max_displacement == None or room.force_vector.Length > iter_check.max_displacement:
                iter_check.max_displacement = room.force_vector.Length
            displace_rectangle(room, room.force_vector * iter_check.k)
        
        #calc repulse forces
        rn_list.sort(key=lambda rn: distance(rn.rectangle.Center, center_pt))
        for room in rn_list:
            calc_rep_vector(room, rn_list)
            if room.force_vector != []:
                displace_rectangle(room, room.force_vector * 2 * iter_check.k)
        
        iter_check.k *= cooling_rate

def set_prev_points(iter_check, room_nodes):
    seen_points = []
    for room in room_nodes:
        seen_points.append(room.rectangle.Center)
    if seen_points == iter_check.prev_points and iter_check.count > 2:
        iter_check.prev_flag = True
    else:
        iter_check.prev_points = seen_points
        iter_check.prev_flag = False

is_running = True

def stop_timer():
    global is_running
    is_running = False
    ghenv.Component.ExpireSolution(True)

if reset:
    # initialize
    room_nodes_new = room_nodes
    max_iter = iterations
    max_displacement = None
    iter_check = iter_count
else:
    center_rectangles(room_nodes_new, center_point)
    
    if iter_check.count < max_iter and iter_check.max_displacement > 0.01 and iter_check.prev_flag == False:
        move_recs(room_nodes_new, iter_check, center_point, cooling_rate)
        iter_check.count += 1
    i = iter_check.count
    if iter_check.max_displacement != None:
        max_dis = iter_check.max_displacement * iter_check.k
    snap_rectangles(room_nodes_new)
    center_rectangles(room_nodes_new, center_point)
    print(iter_check.prev_flag)
    set_prev_points(iter_check, room_nodes_new)

################# OUTPUTS ##############################
    out_recs = []
    heights = []
    room_names = []
    color_list = []
    for room in room_nodes:
        out_recs.append(room.rectangle)
        heights.append(room.height)
        room_names.append(room.name)
        color_list.append(room.color)
    
    out_edges = set()
    for room in room_nodes:
        for edge in room.edges:
            if edge not in out_edges:
                out_edges.add(edge)
    
    for j in range(len(room_names)):
        room_names[j] = room_names[j].replace(" ", "\n")