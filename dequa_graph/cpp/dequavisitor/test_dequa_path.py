import time
import datetime as dt

from pathlib import Path

import numpy as np

# GraphTool
import graph_tool.all as gt
from graph_tool.all import shortest_path


from dequa_graph.utils import load_graphs, get_all_coordinates
from dequa_graph.weights import get_weight_time, get_timetables
from dequa_graph.visitors import dequaVisitor
from dequa_graph.topology import calculate_path, td_shortest_path

import func_gt as fgt
from dequa_path import dequa_shortest_path

# breakpoint()

graph_path = Path('graph_waterbus.gt')
graph = load_graphs(graph_path)

# Lat e lon dei vertici è una proprietà dei vertici
gt_pos_land_prop = graph.vp['latlon']
gt_pos_land = np.array([gt_pos_land_prop[v].a for v in graph.iter_vertices()])

print("--- Test 1: No battelli ---")
print("Partenza: San Polo 1424")
print("Arrivo: Castello 123")
input_start = (12.331366701765935, 45.436707400204234) # san polo 1424
input_end = (12.357022348829444, 45.434129454831535) # castello 123
gt_start, gt_end = fgt.find_closest_vertices(
    [input_start, input_end],
    gt_pos_land,
    verbose=False)
print("Weight: time")
weight_t = get_weight_time(graph)

print("> Metodo vecchio: shortest_path")
time_old = time.time()
old_v, old_e = shortest_path(graph, gt_start, gt_end, weight_t)
time_old_elapsed = time.time() - time_old
print(f">> GraphTool shortest_path: {time_old_elapsed:.03f} seconds")

print("> Metodo nuovo: dequa_shortest_path (usa la stessa funzione di gt....])")
time_new = time.time()
new_v, new_e, new_t = dequa_shortest_path(graph, gt_start, gt_end, weight_t)
time_new_elapsed = time.time() - time_new
print(f">> DeQua shortest_path: {time_new_elapsed:.03f} seconds")

print(f"I vertici sono uguali? {old_v == new_v}")
print(f"Gli edge sono uguali? {old_e == new_e}")

print("--- Test 2: Battelli ---")
print("Partenza: San Polo 1424")
print("Arrivo: Giudecca 123")
input_start = (12.331366701765935, 45.436707400204234) # san polo 1424
input_start = (12.352622145543, 45.45463988978711) # Murano 123
input_end = (12.334255256862335, 45.42508669583883) # giudecca 123
# input_start = (12.326875851106916, 45.42934654038481) # zattere
# input_end = (12.325463808142006, 45.42656387430435) # palanca
gt_start, gt_end = fgt.find_closest_vertices(
    [input_start, input_end],
    gt_pos_land,
    verbose=False)
print("Weight: time")
weight_t = get_weight_time(graph)
walk_speed = 5/3.6
print(f"Speed: {walk_speed:.2f} m/s")
transport_change_penalty = 0
print(f"Transport change penalty: {transport_change_penalty}")
start_time = dt.datetime(2022, 12, 1, 12, 0, 0)
print(f"Start time: {start_time}")

time_edge_property = get_weight_time(graph=graph, speed=walk_speed)
transport_property = graph.vp.transport_stop
timetable_property = get_timetables(graph=graph, date=start_time)
direction_property = graph.ep.direction

print("> Metodo vecchio: td_shortest_path (DeQua visitor)")
time_old = time.time()
old_v, old_e, old_t = td_shortest_path(
    graph, gt_start, gt_end, weight_t,
    start_time, time_edge_property, transport_property, 
    timetable_property, direction_property, transport_change_penalty)
time_old_elapsed = time.time() - time_old
print(f">> Python td_shortest_path: {time_old_elapsed:.03f} seconds")

print("> Metodo nuovo: dequa_shortest_path")
# Bisogna ridefinire i weight!!! perché il dijkstra time dependent li modifica!!
weight_t = get_weight_time(graph)
time_edge_property = get_weight_time(graph=graph, speed=walk_speed)
time_new = time.time()
new_v, new_e, new_t = dequa_shortest_path(
    graph, gt_start, gt_end, weight_t,
    start_time=start_time, time_edges=time_edge_property, 
    transport_property=transport_property, timetable_property=timetable_property,
    direction_property=direction_property,
    transport_change_penalty=transport_change_penalty)
time_new_elapsed = time.time() - time_new
print(f">> C++ shortest_path: {time_new_elapsed:.03f} seconds")

print(f"I vertici sono uguali? {old_v == new_v}")
print(f"Gli edge sono uguali? {old_e == new_e}")
print(f"Tempo di percorrenza (nuovo - vecchio): {sum(new_t) - sum(old_t)} s")

# breakpoint()
