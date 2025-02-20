import time
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# NetworkX
import networkx as nx
import pickle

# GraphTool
import graph_tool.all as gt

import func_gt as fgt

from dequa_graph.utils import load_graphs, get_all_coordinates
from dequa_graph.weights import get_weight_time
from dequa_graph.visitors import dequaVisitor

from dequa_path import dequa_shortest_path

gt_land_path = Path('dequa_ve_terra_v13_1711.gt')
gt_land = load_graphs(gt_land_path)

# Lat e lon dei vertici è una proprietà dei vertici
gt_pos_land_prop = gt_land.vp['latlon']
gt_pos_land = np.array([gt_pos_land_prop[v].a for v in gt_land.iter_vertices()])


input_start = (12.331366701765935, 45.436707400204234) # san polo 1424
input_end = (12.357022348829444, 45.434129454831535) # castello 123

gt_start, gt_end = fgt.find_closest_vertices(
    [input_start, input_end],
    gt_pos_land,
    verbose=True)

time1 = time.time()
gt_path = gt.shortest_path(gt_land, gt_start, gt_end)
time_elapsed = time.time() - time1
print(f"GraphTool - shortest_path: {time_elapsed:.03f} seconds")
time1 = time.time()
dequa_gt_path = dequa_shortest_path(gt_land, gt_start, gt_end)
time_elapsed = time.time() - time1
print(f"GraphTool - dequa_shortest_path: {time_elapsed:.03f} seconds")