"""Provides a scripting component.
    Inputs:
        x: The x script variable
        y: The y script variable
    Output:
        a: The a output variable"""

__author__ = "gregb"
__version__ = "2023.03.13"

import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import Rhino.Display as rd
import random
import math

class threshold_tracker(object):
    def __init__(self, count, max_displacement, cooling_rate):
        self.count = count
        self.max_displacement = max_displacement
        self.k = cooling_rate
        self.prev_flag = False
        self.prev_points = []

def place_first_room(room_node_list, seen_list, initial_point, init_plane):
    width = random.randint(int(room_node_list[0].min_dim), int(room_node_list[0].area//room_node_list[0].min_dim))
    height = int(math.ceil(room_node_list[0].area/width))
    
    half_width  = width/2
    half_height = height/2
    
    corner_0 = rg.Point3d(initial_point.X - half_width, initial_point.Y - half_height, 0)
    corner_2 = rg.Point3d(initial_point.X + half_width, initial_point.Y + half_height, 0)
    
    room_node_list[0].rectangle = rg.Rectangle3d(init_plane, corner_0, corner_2)
    seen_list.append(room_node_list[0])
    
    return room_node_list, seen_list

# returns node of the room with the most adjacencies to seen rooms
def most_adjacencies(room_list, seen_list):
    best_count = 0
    best_room = None
    
    for room in room_list:
        if room not in seen_list:
            curr_count = 0
            for adj_room in room.adjacencies:
                if adj_room in seen_list:
                    curr_count += 1
                if curr_count > best_count:
                    best_count = curr_count
                    best_room = room
                elif curr_count == best_count and best_count != 0:
                    if room.area > best_room.area:
                        best_room = room
    return best_room

# returns bool
def overlaps_seen(test_rect, seen_list):
    test_corners = [test_rect.Corner(i) for i in range(4)]
    test_bbox = test_rect.BoundingBox
    test_nurbs = test_rect.ToNurbsCurve()
    
    for room in seen_list:
        room_nurbs = room.rectangle.ToNurbsCurve()
        if rs.CurveCurveIntersection(test_nurbs, room_nurbs) != None:
            return True
        room_bbox = room.rectangle.BoundingBox
        room_corners = [room.rectangle.Corner(i) for i in range(4)]
        for corner in test_corners:                     #check if any corner of test is in seen room
            if room_bbox.Contains(corner):
                return True
        for corner in room_corners:                     #in reverse to check for concentric
            if test_bbox.Contains(corner):
                return True
    return False

#returns bool
def within_boundary(test_rect, boundary):
    bbox = boundary.BoundingBox
    test_corners = [test_rect.Corner(i) for i in range(4)]
    for corner in test_corners:
        if bbox.Contains(corner) == False:
            return False
    return True

def tri_rec_overlap(tri, rec):
    tri_curve = (tri.ToPolyline()).ToNurbsCurve()
    rec_curve = rec.ToNurbsCurve()
    if rs.CurveCurveIntersection(tri_curve, rec_curve):
        return True
    tri_srf = rg.Brep.CreateFromCornerPoints(tri.A, tri.B, tri.C, 0.001)
    if rs.IsPointOnSurface(tri_srf, rec.Center):
        return True
    return False

def on_triangles(rec, seen_tris):
    for tri in seen_tris:
        if tri_rec_overlap(tri, rec):
            return True
    return False

def on_edges(rec, edge_set):
    rec_curve = rec.ToNurbsCurve()
    for edge in edge_set:
        edge_curve = edge.ToNurbsCurve()
        if rs.CurveCurveIntersection(rec_curve, edge_curve):
            return True
    return False

#returns potential rectangle of next_room
def candidate_rectangle(next_room, boundary, init_plane, seen_list, edge_set, seen_tris, total_area):
    flag = False
    while flag == False:
        if next_room.max_dim == True:
            next_width = random.choice([int(next_room.min_dim), int(next_room.area//next_room.min_dim)])
            next_height = int(math.ceil(next_room.area/next_width))
        else:
            next_width = random.randint(int(next_room.min_dim), int(next_room.area//next_room.min_dim))
            next_height = int(math.ceil(next_room.area/next_width))
        
        # determine area of average adjacencies 
        x_sum = 0
        y_sum = 0
        n     = 0
        for room in seen_list:
            if room in next_room.adjacencies:
                x_sum += room.rectangle.Center.X
                y_sum += room.rectangle.Center.Y
                n += 1
        x = x_sum/n
        y = y_sum/n
        
        # determine repel factor
        x_repel = 0
        y_repel = 0
        r_n = 0
        for room in seen_list:
            if room not in next_room.adjacencies:
                x_repel += (x - room.rectangle.Center.X)
                y_repel += (y - room.rectangle.Center.Y)
                r_n += 1
        if x_repel != 0:
            x += 2 * (r_n/x_repel)
        if y_repel != 0:
            y += 2 * (r_n/y_repel)
        
        mult_factor = total_area/total_area
        print("mf: ", mult_factor)
        next_center_x = int(random.gauss(x, n * 20))
        next_center_y = int(random.gauss(y, n * 20))
        print("x:", next_center_x)
        print(next_center_x * mult_factor)
        print("y: ", next_center_y)
        print(next_center_y * mult_factor)
        next_center = rg.Point3d(next_center_x, next_center_y, 0)
        
        next_half_width  = next_width/2
        next_half_height = next_height/2
        
        next_corner_0 = rg.Point3d(next_center.X - next_half_width, next_center.Y - next_half_height, 0)
        next_corner_2 = rg.Point3d(next_center.X + next_half_width, next_center.Y + next_half_height, 0)
        
        test_rect = rg.Rectangle3d(init_plane, next_corner_0, next_corner_2)
        
        if overlaps_seen(test_rect, seen_list) == False and on_triangles(test_rect, seen_tris) == False and on_edges(test_rect, edge_set) == False:# and within_boundary(test_rect, boundary) == True:
            #print("within bounds: ", within_boundary(test_rect, boundary))
            #print("No intersection pass")
            flag = True
            break
        #print("had to try again")

    return test_rect

#returns potential edges of next room or False if any candidate_edge intersects seen_edges
def candidate_edges(room_node, seen_list, seen_edges):
    #print("seen list: ", seen_list)
    edges = []
    
    #loop through next_room's adjacencies
    for adj_room in room_node.adjacencies:
        #if the adjacency is plotted/in the seen list...
        if adj_room in seen_list:
            
            #create a line from the next_room's center to the adjacent plotted room's center
            test_edge = rg.Line(room_node.rectangle.Center, adj_room.rectangle.Center)
            
            #loop through the current list of plotted edges
            for seen_edge in seen_edges:
                test_curve = rg.NurbsCurve.CreateFromLine(test_edge)
                seen_curve = rg.NurbsCurve.CreateFromLine(seen_edge)
                #check if the edges intersect and that it is not the start or end of the seen_edge
                if rs.CurveCurveIntersection(test_curve, seen_curve) == None:
                    #get out of this
                    continue
                elif (test_edge.To == seen_edge.From or test_edge.To == seen_edge.To):
                    #also get out of this
                    continue
                else:
                    #print("intersect :", rs.CurveCurveIntersection(test_curve, seen_curve) != None)
                    return False
            
            edges.append(test_edge)
                
    #loop through edges and make sure they dont intersect with plotted rectangles
    for edge in edges:
        edge_curve = edge.ToNurbsCurve()
        for seen_room in seen_list:
            if seen_room not in room_node.adjacencies:
                seen_room_curve = seen_room.rectangle.ToNurbsCurve()
                if rs.CurveCurveIntersection(edge_curve, seen_room_curve) != None:
                    return False
                else:
                    continue
    return edges

# sort edges by counter clockwise order
def angle_key(line):
    vector = line.To - line.From
    return math.atan2(vector.Y, vector.X)

def candidate_triangles(room_node, seen_list):
    adj_count = 0
    for room in room_node.adjacencies:
        if room in seen_list:
            adj_count += 1
    if adj_count < 2:
        return []
    ordered_edges = sorted(room_node.edges, key=angle_key)
    triangles = []
    base_point = room_node.rectangle.Center
    for i in range(len(ordered_edges) - 1):
        second_point = ordered_edges[i].To
        third_point = ordered_edges[i+1].To
        tri = rg.Triangle3d(base_point, second_point, third_point)             #rg.Brep.CreateFromCornerPoints(base_point, second_point, third_point, 0.001)
        if math.degrees(tri.AngleA) < 15 or math.degrees(tri.AngleB) < 15 or math.degrees(tri.AngleC) < 15:
            return False
        for room in seen_list:
            if room not in room_node.adjacencies:
                if tri_rec_overlap(tri, room.rectangle):
                    return False
        triangles.append(tri)
    return triangles

def reset_room_node(room_node):
    room_node.rectangle = None
    room_node.edges = []
    room_node.triangles = []

def build_graph(rn_list, seen_list, edge_set, boundary, init_plane, seen_tris, total_area):
    if set(rn_list) == set(seen_list):
        print("GRAPH COMPLETE")
        return True, edge_set, seen_list
    elif seen_list == []:
        print("graph cannot be built, seen list to zero")
        return False, edge_set, seen_list
    else:
        while seen_list[-1].bt_count < 20:
            #set up next candidate room
            next_room = most_adjacencies(rn_list, seen_list)
            next_room.rectangle = candidate_rectangle(next_room, boundary, init_plane, seen_list, edge_set, seen_tris, total_area)
            next_room.edges = candidate_edges(next_room, seen_list, edge_set)
            
            # check if edges intersect
            if next_room.edges == False:
                reset_room_node(next_room)
                seen_list[-1].bt_count += 1
                print(str(seen_list[-1].name) + "'s bt_count = " + str(seen_list[-1].bt_count))
                continue
            
            #check that candidate triangles don't overlap seen list
            next_room.triangles = candidate_triangles(next_room, seen_list)
            if next_room.triangles == False:
                reset_room_node(next_room)
                seen_list[-1].bt_count += 1
                print(str(seen_list[-1].name) + "'s bt_count = " + str(seen_list[-1].bt_count))
                continue
            
            print(str(next_room.name) + "'s are edges clear")
            seen_list.append(next_room)
            edge_set.update(next_room.edges)
            seen_tris.update(next_room.triangles)
            
            return build_graph(rn_list, seen_list, edge_set, boundary, init_plane, seen_tris, total_area)
        
        seen_list[-1].bt_count = 0
        edge_set.difference_update(seen_list[-1].edges)
        seen_tris.difference_update(seen_list[-1].triangles)
        
        seen_list[-2].bt_count += 1
        seen_list.pop()
        
        return build_graph(rn_list, seen_list, edge_set, boundary, init_plane, seen_tris, total_area)

def iterate_room_node_list(room_node_list, inital_point, boundary, total_area):
    init_plane = rg.Plane(initial_point, rg.Vector3d(0,0,1))
    seen_list = []
    edge_set = set()
    seen_tris = set()
    
    # place first room
    room_node_list, seen_list = place_first_room(room_node_list, seen_list, initial_point, init_plane)
    
    graph_flag, edge_set, seen_list = build_graph(room_node_list, seen_list, edge_set, boundary, init_plane, seen_tris, total_area)
    print(seen_tris)
    
    if graph_flag == True:
        rectangle_list = []
        room_name_list = []
        triangles = []
        for room in room_node_list:
            if room.rectangle != None:
                rectangle_list.append(room.rectangle)
                room_name_list.append(room.name)
        for tri in seen_tris:
            print(tri)
            triangles.append(rg.Brep.CreateFromCornerPoints(tri.A, tri.B, tri.C, 0.001))
    else:
        rectangle_list = []
        
    return rectangle_list, edge_set, room_name_list, seen_list


rectangle_list, edge_list, room_name_list, room_nodes = iterate_room_node_list(room_node_list, initial_point, boundary, total_area)

for i in range(len(room_name_list)):
    room_name_list[i] = room_name_list[i].replace(" ", "\n")

iter_check = threshold_tracker(0, max_displacement, cooling_rate)