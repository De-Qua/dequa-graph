from datetime import datetime, timedelta
import numpy as np
import time

from .weights import get_weight_time, get_timetables
from .topology import td_shortest_path
from .dequa_path import dequa_shortest_path
from .visitors import find_closest_vertices
from .formatting import retrieve_info_from_path_streets

def test_battelli(graph, coord_source, coord_target, start_time=None, speed=5/3.6, use_python_version=False):
    if start_time is None:
        start_time = datetime.today()
    pos = graph.vp['latlon']
    all_pos = np.array([pos[v].a for v in graph.iter_vertices()])

    id_closest_vertex_source, id_closest_vertex_target = find_closest_vertices([coord_source, coord_target], all_pos)

    source = graph.vertex(id_closest_vertex_source)
    target = graph.vertex(id_closest_vertex_target)

    time_edge_property = get_weight_time(graph=graph, speed=speed)
    transport_property = graph.vp.transport_stop
    timetable_property = get_timetables(graph=graph, date=start_time)
    direction_property = graph.ep.direction

    weight = get_weight_time(graph=graph, speed=speed)
    
    transport_change_penalty=0

    time_start = time.time()
    if use_python_version:
        vlist, elist, tlist = td_shortest_path(graph, source, target,
                                weight, start_time, time_edge_property,
                                transport_property, timetable_property, direction_property, transport_change_penalty)
    else:
        vlist, elist, tlist = dequa_shortest_path(
                        graph, source, target, weights=weight, 
                        start_time=start_time, 
                        time_edges=time_edge_property, 
                        transport_property=transport_property, 
                        timetable_property=timetable_property, 
                        direction_property=direction_property, 
                        transport_change_penalty=transport_change_penalty)
    computation_time = time.time() - time_start
    
    info = retrieve_info_from_path_streets(
        graph=graph, paths_vertices=[[vlist]], paths_edges=[[elist]],
        times_edges=[[tlist]], start_time=start_time)

    print(info)

    print(f"Partenza da {source}: {start_time}")
    for idx, v in enumerate(vlist):
        if graph.vp.transport_stop[v]:
            name = graph.vp.stop_info[v]["name"]
            elapsed_time = timedelta(seconds=sum(tlist[:idx+1]))
            print(f"Dopo {elapsed_time} arriviamo a {name}")
            if not graph.vp.transport_stop[vlist[idx-1]]:
                waiting_time = sum(tlist[:idx+1])-sum(tlist[:idx])
                waiting_time = timedelta(seconds=waiting_time)
                print(f"\tAbbiamo aspettato {waiting_time}")
                print("\tSaliamo in battello")
            else:
                print("\tRestiamo in battello")
            linea = graph.ep.route[graph.edge(vlist[idx-1], v)]["route_short_name"]
            print(f"\tBattello {linea} delle {start_time+elapsed_time}")
        elif graph.vp.transport_stop[vlist[idx-1]]:
            print(f"Scendiamo dal battello")
    tot_elapsed_time = timedelta(seconds=sum(tlist))
    print(f"Arrivo a {target}: {start_time+tot_elapsed_time}")
    print(f"Il percorso è durato {tot_elapsed_time}")
    print(f"La ricerca ha impiegato {computation_time}s")
