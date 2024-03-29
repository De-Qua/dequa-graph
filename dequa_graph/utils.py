"""Useful functions."""

import numpy as np
import pandas as pd
from datetime import timedelta
from itertools import groupby
from shapely.geometry import LineString
from shapely.ops import transform

import graph_tool.all as gt
import ipdb
# IMPORT OUR LIBRARIES
from . import set_up_logging
from . import lib_gtfs as gtfs

logger = set_up_logging()


def load_graphs(*paths_gt_graphs):
    """Load graph-tool graphs, one for each input path."""
    all_graphs = []
    for path_gt in paths_gt_graphs:
        path_gt = str(path_gt)
        all_graphs.append(gt.load_graph(path_gt))

    if len(all_graphs) == 1:
        all_graphs = all_graphs[0]
    return all_graphs


def add_waterbus_to_street(graph, path_gtfs):
    """Add gtfs vertices and edges to graph"""
    if type(path_gtfs) is str:
        path_gtfs = [path_gtfs]
    # ipdb.set_trace()
    # add transport stop boolean vertex property
    transport_stop = graph.new_vp("bool")
    graph.vp.transport_stop = transport_stop
    # add transport boolean edge property
    transport = graph.new_ep("bool")
    graph.ep.transport = transport
    # add info StopI
    stop_info = graph.new_vp("python::object")
    graph.vp.stop_info = stop_info
    # add timetable edge property
    timetable = graph.new_ep("vector<int>")
    graph.ep.timetable = timetable
    # add route edge property
    route = graph.new_ep("python::object")
    graph.ep.route = route
    # add direction edge property (target)
    direction = graph.new_ep("int")
    graph.ep.direction = direction
    # add duration edge property
    duration = graph.new_ep("double")
    graph.ep.duration = duration
    # add normal dates as graph property
    normal_dates = graph.new_gp("python::object")
    graph.gp.normal_dates = normal_dates
    # add last normal date as graph property
    last_normal_date = graph.new_gp("string")
    graph.gp.last_normal_date = last_normal_date
    # add special dates as graph property
    special_dates = graph.new_gp("python::object")
    graph.gp.special_dates = special_dates
    # get components
    comp, hist = gt.label_components(graph)
    comp.a += 1
    graph.vp.component_street = comp
    # get a copy of the original graph without the transports
    g_orig = gt.GraphView(graph, vfilt=lambda v: not transport_stop[v])
    pos = get_all_coordinates(g_orig)
    # load feed gtfs
    # feed = gtfs.load_feed(path_gtfs[0])
    feeds = gtfs.load_multiple_feeds(*path_gtfs)
    # create edge properties for special dates
    special_dates_dict = {}
    normal_dates_dict = {}
    last_normal_date = ""
    for feed in feeds:
        unique_special_dates = []
        if feed.calendar_dates is not None:
            unique_special_dates = np.unique(feed.calendar_dates.date.values)
        for unique_special_date in unique_special_dates:
            special_date_key = pd.to_datetime(unique_special_date).date()
            special_dates_dict[special_date_key] = special_date_key.strftime("%Y-%m-%d")

        start_date = pd.to_datetime(gtfs.get_start_date(feed)).date()
        end_date = pd.to_datetime(gtfs.get_end_date(feed)).date()
        key_for_date = f"{start_date.strftime('%Y-%m-%d')}_{end_date.strftime('%Y-%m-%d')}"
        for normal_date in daterange(start_date, end_date):
            if normal_date not in unique_special_dates:
                normal_date_key = pd.to_datetime(normal_date).date()
                normal_dates_dict[normal_date_key] = key_for_date
        # save the last normal date (is updated in every loop)
        last_normal_date = key_for_date
    # add edge properties for special dates
    for special_date_val in special_dates_dict.values():
        graph.edge_properties[special_date_val] = graph.new_ep("vector<int>")
    # add edge properties for normal dates (just one for each single date)
    for normal_date_val in set(normal_dates_dict.values()):
        graph.edge_properties[normal_date_val] = graph.new_ep("vector<int>")
    # add all special dates in the graph property
    graph.gp.special_dates = special_dates_dict
    # add all normal dates in the graph property
    graph.gp.normal_dates = normal_dates_dict
    # add the last normal date in the graph property
    graph.gp.last_normal_date = last_normal_date

    for feed in feeds:
        missing_stops = gtfs.check_stops_coordinates(feed, pos)
        if len(missing_stops) > 0:
            logger.error(f"Some stops are not present in the graph: {missing_stops}")
        else:
            logger.info("All the stops are present in the graph!")
        count = 1
        all_routes = gtfs.get_all_routes_id(feed)
        for route_id in all_routes:
            logger.debug(f"{count}/{len(all_routes)} - Route: {route_id}")
            count += 1
            route_df = gtfs.get_route_sequence(feed, route_id)
            last_v = None
            for idx, row in route_df.iterrows():
                # logger.debug(f"\t{idx+1}/{len(route_df)}")
                if row["end_stop_id"] is np.nan:
                    continue
                if (row["start_stop_id"] in missing_stops) or (row["end_stop_id"] in missing_stops):
                    logger.warning(f"Missing stop {row['start_stop_id']} in route {route_id}")
                    last_v = None
                    continue
                if last_v is None:
                    last_v = add_route_vertex_and_edge(graph, g_orig, pos, feed, row["start_stop_id"], row)
                new_v = add_route_vertex_and_edge(graph, g_orig, pos, feed, row["end_stop_id"], row)
                edge = graph.add_edge(last_v, new_v)
                # ipdb.set_trace()
                graph.ep.transport[edge] = True
                graph.ep.duration[edge] = int(row["duration"].total_seconds())
                graph.ep.route[edge] = row[["route_id", "route_short_name", "route_color", "route_text_color"]].to_dict()
                graph.ep.direction[edge] = new_v
                graph.ep.geometry[edge] = transform(lambda x, y: (y, x), row["geometry"])
                last_v = new_v
    comp, hist = gt.label_components(graph)
    comp.a += 1
    graph.vp.component_waterbus = comp

    return g_orig, graph


def get_all_coordinates(graph):
    """Return array with vertices coordinates from a graph."""
    pos = graph.vp['latlon']
    return np.array([pos[v].a for v in graph.get_vertices()])


def get_id_from_coordinates(pos, coordinates):
    try:
        return np.where((pos[:, 0] == coordinates[0]) & (pos[:, 1] == coordinates[1]))[0][0]
    except IndexError:
        ipdb.set_trace()


def add_route_vertex_and_edge(graph, graph_orig, pos, feed, stop_id, row):
    # ipdb.set_trace()
    # find the platform on the original graph
    start_stop_coordinate = gtfs.get_stop_coordinates(feed, stop_id)
    platform = graph_orig.vertex(get_id_from_coordinates(pos, start_stop_coordinate))
    # add a vertex for the route-specific platform
    v = graph.add_vertex()
    graph.vp.transport_stop[v] = True
    graph.vp.latlon[v] = start_stop_coordinate
    graph.vp.stop_info[v] = {"id": stop_id, "name": feed.stops[feed.stops.stop_id == stop_id]["stop_name"].iloc[0]}
    # add edge between platform and route
    e = graph.add_edge(platform, v)
    # add timetable to e
    time_info = gtfs.get_stop_times_from_stop_route(feed, stop_id, row["route_id"])
    # graph.ep.timetable[e] = time_info[["service_id", "departure_time"]]
    normal_dates, exception_dates = gtfs.convert_departure_to_array(time_info, feed)
    # get a normal date looking in the calendar and looping until we a date that is not special
    a_normal_date = pd.to_datetime(gtfs.get_start_date(feed)).date()
    while a_normal_date not in graph.gp.normal_dates.keys():
        a_normal_date += timedelta(days=1)
    timetable_edge_property = graph.gp.normal_dates[a_normal_date]
    graph.ep[timetable_edge_property][e] = normal_dates
    # old code: {'standard': normal_dates} # | exception_dates  # merge two dictionaries
    for exception_date_key, values in exception_dates.items():
        edge_property_name = graph.gp.special_dates[exception_date_key]
        # the edge property must have a string
        graph.ep[edge_property_name][e] = values  # array with timetable
    graph.ep.route[e] = row[["route_id", "route_short_name", "route_color", "route_text_color"]].to_dict()
    graph.ep.geometry[e] = LineString()
    return v


def _len_iter(items):
    return sum(1 for _ in items)


def consecutive_one(data):
    """Helper to count the maximum number of consecutive items in a list.
    Thanks to Veedrac: https://codereview.stackexchange.com/questions/138550/count-consecutive-ones-in-a-binary-list
    """
    try:
        return max(_len_iter(run) for val, run in groupby(data) if val)
    except ValueError:
        return 0


def adjacent_one(data, item=1):
    """
    Helper to count the number of adjacent items in a list
    """
    adjacent_data = list(val for val, run in groupby(data))
    return adjacent_data.count(item)


def daterange(start_date, end_date):
    for n in range(int((end_date - start_date).days)+1):
        yield start_date + timedelta(n)
