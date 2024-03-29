"""
Response format
[
    PATH1
    [
        total_distance
        total_duration
        paths: {
            start_time
            end_time
            distances
            time
            num_bridges
            num_edges
            num_transports
            routes
            edges
        }
    ]
    PATH2
    [
        start_time
        end_time
        distances
        time
        num_bridges
        num_edges
        num_transports

        edges
        path_steps
        [
            {
                type (walk/boat/ferry/passerella)
                start_time
                end_time
                distances
                time
                num_bridges
                route_color
                route_text_color
                route_name
                route_short_name
            }
            for route_piece
        ]
    ]
]
"""

import numpy as np
from shapely.geometry import mapping, MultiLineString
import ipdb
from datetime import datetime, timedelta

from .utils import adjacent_one


def format_path_steps(**kwargs):
    """
    Format a single step of a path.
    A path consists of (possibly) many steps.
    A step finishes when you change mean of transportation (walk, ferry, boat).
    """
    step = {
        # General info
        "type":         kwargs.get("type", "unknown"),
        "order":        kwargs.get("order", -1),
        "start_time":   kwargs.get("start_time", ""),
        "end_time":     kwargs.get("end_time", ""),
        "distance":     kwargs.get("distance", 0),
        "duration":     kwargs.get("duration", 0),
        # Bridges
        "walk": {
            "num_bridges": kwargs.get("num_bridges", 0)
        } if kwargs["type"] == "walk" else None,
        # Ferry
        "ferry": {
            "route_color":      kwargs.get("route_color", None),
            "route_text_color": kwargs.get("route_text_color", None),
            "route_name":       kwargs.get("route_name", None),
            "route_short_name": kwargs.get("route_short_name", None),
            "route_stops":      kwargs.get("route_stops", None),
            "route_waiting_time": kwargs.get("route_waiting_time", None)
        } if kwargs["type"] == "ferry" else None,
        # Boat
        "boat": {
            "type": kwargs.get("boat_type", "generic"),
        } if kwargs["type"] == "boat" else None,
    }
    return step


def format_edge_info(**kwargs):
    edge_type = kwargs.get("edge_type", None)
    if edge_type == "walk":
        return {
            "edge_type": "walk"
        }
    elif edge_type == "ferry":
        return {
            "edge_type": "ferry",
            "route_color": kwargs.get("route_color", None)
        }
    elif edge_type == "bridge":
        return {
            "edge_type": "bridge",
            "accessibility": kwargs.get("accessibility", 0)
        }
    else:
        return {
            "edge_type": edge_type
        }


def retrieve_info_from_path_streets(graph, paths_vertices, paths_edges, start_time,
                                    speed=5/3.6, times_edges=None, **kwargs):
    """Retrieve useful informations from the output of a path of streets (list
    of list of vertices and edges). The length of the two lists corresponds to
    the number of paths, i.e. if there are no stops betweem the start and the
    end point there will be only one path, otherwise there will be multiple
    paths.
    The output is a dictionary with the following keys:
        'n_paths' (int): indicates the number of paths
        'info' (list(dict)): informations of each path
    Each path has the following informations:
        'distance' (float): distance in meters
        'num_bridges' (int): the number of bridges
        'num_edges' (int): number of contained edges
        'edges' (dict): info for each single edge.
        'path_steps' (list of dict): info about transportations.
        Each key is a list of the dimension num_edges.
            'distances' (float): distance in meters
            'bridges' (boolean): True if the edge is a bridge
            'geometries' (geometry): Geometry of the edge
            'max_tides' (float): Maximum tide for the edge in cm
            'accessibility' (int): Accessibility value
            'walkways_zps' (float): If present, tide level for activation of walkways
            'walkways_cm' (float): Height of walkways in cm
            'streets_id' (int): Database id of the street
    """
    all_info = []
    for alternative_path, alternative_times in zip(paths_edges, times_edges):
        info = []
        intermediate_start_time = start_time
        tot_distance = 0
        tot_duration = 0
        for edges, edge_times in zip(alternative_path, alternative_times):
            distances = []
            durations = []
            is_bridge = []
            is_transport = []
            geojsons = []
            ferry_routes = {}  # []
            ferry_route = {}
            # list of the path steps (remember to use format method to append)
            path_steps = []
            current_step = {
                "type":         "walk",
                "order":        0,
                "distance":     0,
                "duration":     0,
                "bridges":       [],
                # "num_bridges":  0,
                'start_time':   intermediate_start_time.strftime("%Y-%m-%dT%H:%M:%S"),
            }
            # stops = []
            time_at_edge = intermediate_start_time

            last_edge = None
            edge_linestrings = []

            for e, e_time in zip(edges, edge_times):
                # general info
                distance = graph.ep["length"][e]
                # case transport
                if graph.ep["transport"][e] == 1:
                    edge_type = "ferry"
                    # ipdb.set_trace()
                    # store duration
                    duration = graph.ep['duration'][e]
                    time_at_edge += timedelta(seconds=duration)
                    # route = graph.ep["route"][e]["route_short_name"]
                    stop = {
                        "name":         graph.vp.stop_info[e.source()]["name"],
                        "clock_time":   time_at_edge.strftime("%Y-%m-%dT%H:%M:%S")
                    }
                    # routes[route] exists because we passed through ferry stop
                    # current_step["route_short_name"] = graph.ep["route"][e]["route_short_name"]
                    # ferry_routes[route]["stops"].append(stop)
                    # update current step
                    if current_step['type'] == 'ferry':
                        # just update the current
                        # update current ferry route
                        if "route_stops" not in current_step.keys():
                            # this should never happen, but just in case…
                            current_step.update(
                                route_short_name=graph.ep["route"][e]["route_short_name"],
                                route_text_color=graph.ep["route"][e]["route_text_color"],
                                route_color=graph.ep["route"][e]["route_color"],
                                route_waiting_time=0,
                                route_stops=[]
                            )
                        current_step["route_stops"].append(stop)
                        current_step['distance'] += distance
                        current_step['duration'] += duration
                    elif current_step['type'] == "walk":
                        # close the current step and create a new one
                        # this should never happen
                        current_step['end_time'] = time_at_edge.strftime("%Y-%m-%dT%H:%M:%S")
                        current_step["num_bridges"] = adjacent_one(current_step["bridges"])
                        path_steps.append(format_path_steps(**current_step))
                        # new step
                        last_order = current_step['order']
                        current_step = {
                            "type":                 "ferry",
                            "order":                last_order + 1,
                            "distance":             0,
                            "duration":             0,
                            # "num_bridges": 0,
                            "start_time":           time_at_edge.strftime("%Y-%m-%dT%H:%M:%S"),

                        }
                # else:
                # case ferry stop
                elif graph.ep["route"][e] is not None and times_edges is not None:
                    edge_type = "ferry_stop"
                    # ipdb.set_trace()
                    # store duration (that is the waiting time of the stop)
                    duration = e_time
                    arrival_time = time_at_edge
                    time_at_edge += timedelta(seconds=duration)
                    # update current ferry route

                    # update current step
                    if current_step['type'] == 'ferry':
                        # from ferry to walk
                        # close ferry step
                        current_step['distance'] += distance
                        current_step['duration'] += duration
                        current_step['end_time'] = time_at_edge.strftime("%Y-%m-%dT%H:%M:%S")
                        path_steps.append(format_path_steps(**current_step))
                        # new walk step
                        last_order = current_step['order']
                        current_step = {
                            "type": "walk",
                            "order": last_order + 1,
                            "distance": 0,
                            "duration": 0,
                            "bridges": [],
                            'start_time': time_at_edge.strftime("%Y-%m-%dT%H:%M:%S"),
                        }

                    elif current_step['type'] == "walk":
                        # from walk to ferry
                        # close walk step
                        current_step['end_time'] = arrival_time.strftime("%Y-%m-%dT%H:%M:%S")
                        current_step["num_bridges"] = adjacent_one(current_step["bridges"])
                        path_steps.append(format_path_steps(**current_step))

                        # update current ferry route
                        stop = {
                            "name":         graph.vp.stop_info[e.source()]["name"],
                            "clock_time":   time_at_edge.strftime("%Y-%m-%dT%H:%M:%S")
                        }

                        # new ferry step
                        last_order = current_step['order']
                        current_step = {
                            "type": "ferry",
                            "order": last_order + 1,
                            "distance": 0,
                            "duration": 0,
                            # "num_bridges": 0,
                            'start_time': arrival_time.strftime("%Y-%m-%dT%H:%M:%S"),
                            # Ferry
                            "route_text_color":     graph.ep["route"][e]["route_text_color"],
                            "route_color":          graph.ep["route"][e]["route_color"],
                            # "route_name": graph.ep["route"][e]["route_name"],
                            "route_short_name":     graph.ep["route"][e]["route_short_name"],
                            "route_waiting_time":   e_time,
                            "route_stops":          [stop]
                        }

                else:
                    # store duration
                    duration = graph.ep['length'][e]/speed
                    time_at_edge += timedelta(seconds=duration)
                    edge_type = "bridge" if graph.ep['ponte'][e] else "walk"

                    if current_step["type"] == "ferry":
                        # ipdb.set_trace()
                        # close transportation step and start walking step
                        current_step['end_time'] = time_at_edge.strftime("%Y-%m-%dT%H:%M:%S")
                        last_order = current_step['order']
                        path_steps.append(format_path_steps(**current_step))
                        # new one
                        current_step = {
                            "type": "walk",
                            "order": last_order + 1,
                            "distance": 0,
                            "duration": 0,
                            "bridges": [],
                            'start_time': time_at_edge.strftime("%Y-%m-%dT%H:%M:%S"),
                        }
                    elif current_step["type"] == "walk":
                        current_step['distance'] += distance
                        current_step['duration'] += duration
                        current_step['bridges'].append(graph.ep['ponte'][e])

                #time_at_edge += timedelta(seconds=time)

                edge_info = {
                    "edge_type": edge_type,
                    # # Calculate distance
                    # 'distance': graph.ep['length'][e],
                    # # Get hour at the edge
                    # 'clock_time': time_at_edge.strftime("%H:%M:%S"),
                    # # Check if it is a transport
                    # 'transport': graph.ep['transport'][e],
                    # # Duration (fixed for trasnport, calculated for walk/boat)
                    # 'duration': duration,  # graph.ep['duration'][e],
                    # Route
                    # 'route': route,
                    'route_color': graph.ep["route"][e]["route_color"] if graph.ep["route"][e] else None,
                    # Stop
                    # 'stop': stop,
                    # # Calculate number of bridges
                    # 'bridge': graph.ep['ponte'][e],
                    # append maximum tide level
                    'max_tide': graph.ep['max_tide'][e],
                    # append accessibility
                    'accessibility': graph.ep['accessible'][e],
                    # append walkways zps activation
                    'walkway_zps': graph.ep['pas_cm_zps'][e],
                    # append walkways height
                    'walkway_cm': graph.ep['pas_height'][e],
                    # # append street id
                    'street_id': graph.ep['street_id'][e]
                }
                edge_info_formatted = format_edge_info(**edge_info)
                # ipdb.set_trace()
                if last_edge is None:
                    last_edge = edge_info_formatted
                # correct for NaN values
                # for k, v in edge_info.items():
                #     if k in ["max_tide", "accessibility", "walkway_zps", "walkway_cm"]:
                #         if np.isnan(v):
                #             edge_info[k] = None
                if last_edge == edge_info_formatted:
                    if not graph.ep["geometry"][e].is_empty:
                        edge_linestrings.append(graph.ep['geometry'][e])
                else:
                    # close old edge
                    geojson = {
                        "type": "Feature",
                        "properties": last_edge,
                        "geometry": mapping(MultiLineString(edge_linestrings))
                    }
                    # ipdb.set_trace()
                    geojsons.append(geojson)
                    last_edge = edge_info_formatted
                    if graph.ep["geometry"][e].is_empty:
                        edge_linestrings = []
                    else:
                        edge_linestrings = [graph.ep['geometry'][e]]
                # # append geometries
                # geojson = {
                #     "type": "Feature",
                #     "properties": edge_info,
                #     "geometry": mapping(graph.ep['geometry'][e])
                # }
                # geojsons.append(geojson)

                # update distance, time and bridges
                distances.append(distance)
                durations.append(duration)
                is_bridge.append(graph.ep['ponte'][e])
                is_transport.append(graph.ep['transport'][e])

            # Add last step
            if current_step["type"] == "walk":
                current_step["num_bridges"] = adjacent_one(current_step["bridges"])
            current_step['end_time'] = time_at_edge.strftime("%Y-%m-%dT%H:%M:%S")
            # close last edge
            geojson = {
                "type": "Feature",
                "properties": last_edge,
                "geometry": mapping(MultiLineString(edge_linestrings))
            }
            geojsons.append(geojson)

            # CHECK: add only if duration and distance greater than zero.
            # We do not want the 0,0 steps (change of boat)
            if current_step['duration'] > 0 and current_step['distance'] > 0:
                path_steps.append(format_path_steps(**current_step))

            tot_distance = sum(distances)
            tot_duration = sum(durations)
            # num_bridges = adjacent_one(is_bridge)
            # num_transports = adjacent_one(is_transport)

            info.append({
                'start_time': intermediate_start_time.strftime("%Y-%m-%dT%H:%M:%S"),
                'end_time': time_at_edge.strftime("%Y-%m-%dT%H:%M:%S"),
                'distance': tot_distance,
                'duration': tot_duration,
                # 'num_bridges': num_bridges,
                # 'num_edges': len(edges),
                # 'num_transports': num_transports,
                # 'ferry_routes': ferry_routes,
                'steps': path_steps,
                #'stops': stops,
                'edges': geojsons
            })
            intermediate_start_time = time_at_edge
            tot_distance += tot_distance
            tot_duration += tot_duration
        info_path = {
            'total_distance': tot_distance,
            'total_duration': tot_duration,
            'paths': info
        }
        all_info.append(info_path)
    return all_info


def retrieve_info_from_path_water(graph, paths_vertices, paths_edges, start_time,
                                  boat_speed=5, times_edges=None, motor=False, **kwargs):
    """Retrieve useful informations from the output of a path of canals (list
    of list of vertices and edges). The length of the two lists corresponds to
    the number of paths, i.e. if there are no stops betweem the start and the
    end point there will be only one path, otherwise there will be multiple
    paths.
    The output is a dictionary with the following keys:
        'n_paths' (int): indicates the number of paths
        'info' (list(dict)): informations of each path
    Each path has the following informations:
        'distance' (float): distance in meters
        'num_bridges' (int): the number of bridges
        'num_edges' (int): number of contained edges
        'edges' (dict): info for each single edge.
                        Each key is a list of the dimension num_edges.
            'distances' (float): distance in meters
            'bridges' (boolean): True if the edge is a bridge
            'geometries' (geometry): Geometry of the edge
            'max_tides' (float): Maximum tide for the edge in cm
            'accessibility' (int): Accessibility value
            'walkways_zps' (float): If present, tide level for activation of walkways
            'walkways_cm' (float): Height of walkways in cm
            'streets_id' (int): Database id of the street
    """
    all_info = []
    for alternative_path, alternative_times in zip(paths_edges, times_edges):
        info = []
        intermediate_start_time = start_time
        tot_distance = 0
        tot_duration = 0
        for edges, edge_times in zip(alternative_path, alternative_times):
            distances = []
            durations = []
            geojsons = []

            path_steps = []
            current_step = {
                "type":         "boat",
                "order":        0,
                "distance":     0,
                "duration":     0,
                'start_time':   intermediate_start_time.strftime("%Y-%m-%dT%H:%M:%S"),
                "boat_type":    "motor" if motor else "row"
            }

            time_at_edge = intermediate_start_time

            for e, e_time in zip(edges, edge_times):
                if graph.ep['vel_max'][e] == 0:
                    edge_speed = boat_speed
                else:
                    edge_speed = min(boat_speed, graph.ep['vel_max'][e]/3.6)
                distance = graph.ep['length'][e]
                duration = distance / edge_speed
                time_at_edge += timedelta(seconds=duration)
                current_step["distance"] += distance
                current_step["duration"] += duration
                if graph.ep['solo_remi'][e]:
                    edge_type = "rioblu"
                else:
                    edge_type = "boat_motor" if motor else "boat_row"

                edge_info = {
                    'edge_type': edge_type,
                    'max_speed': graph.ep['vel_max'][e],
                    # append name
                    'name': graph.ep['nome'][e],
                    # append starting hour
                    'start_h': graph.ep['h_su_start'][e],
                    # append ending hour
                    'end_h': graph.ep['h_su_end'][e],
                    # append starting dt
                    'start_dt': graph.ep['dt_start'][e],
                    # append ending dt
                    'end_dt': graph.ep['dt_end'][e]
                }
                # correct for NaN values
                for k, v in edge_info.items():
                    if type(v) is not str and np.isnan(v):
                        edge_info[k] = None

                # append geometries
                geojson = {
                    "type": "Feature",
                    "properties": edge_info,
                    "geometry": mapping(graph.ep['geometry'][e])
                }
                geojsons.append(geojson)

                # update distance, time and bridges
                distances.append(distance)
                durations.append(duration)
            # Add last step
            current_step['end_time'] = time_at_edge.strftime("%Y-%m-%dT%H:%M:%S")
            path_steps.append(format_path_steps(**current_step))

            tot_distance = sum(distances)
            tot_duration = sum(durations)

            info.append({
                'start_time': intermediate_start_time.strftime("%Y-%m-%dT%H:%M:%S"),
                'end_time': time_at_edge.strftime("%Y-%m-%dT%H:%M:%S"),
                'distance': tot_distance,
                'duration': tot_duration,
                'steps': path_steps,
                'edges': geojsons
            })
            intermediate_start_time = time_at_edge
            tot_distance += tot_distance
            tot_duration += tot_duration
        info_path = {
            'total_distance': tot_distance,
            'total_duration': tot_duration,
            'paths': info
        }
        all_info.append(info_path)
    return all_info


def length_of_edges(graph, edge_list):
    """Calculate the length of a list of edges"""
    return sum([graph.ep['length'][e] for e in edge_list])
