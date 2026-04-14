import  graph_tool.all as gt
from dequa_path import dequa_shortest_path
import datetime as dt

g = gt.Graph(directed=False)
weight = g.new_ep("double")
time_edges = g.new_ep("double")
transport_property = g.new_vp("bool")
timetable = g.new_ep("vector<int>")
direction = g.new_ep("int")
transport_change_penalty = 1.0
v1=g.add_vertex()
v2=g.add_vertex()
v3=g.add_vertex()
e12 = g.add_edge(v1,v2)
e23 = g.add_edge(v2,v3)
e13 = g.add_edge(v1,v3)
weight[e12] = 1
weight[e23] = 1
weight[e13] = 3
direction[e12] = v1
direction[e23] = v3
direction[e13] = v3
time_edges[e12]=1
time_edges[e23]=1
time_edges[e13]=3
path = dequa_shortest_path(g, v1, v3,
    start_time = 1.,
    weights=weight,
    time_edges=time_edges,
    transport_property=transport_property,
    timetable_property=timetable,
    direction_property=direction,
    transport_change_penalty=transport_change_penalty,
    start_time_seconds=dt.datetime.now().timestamp())

print(path)