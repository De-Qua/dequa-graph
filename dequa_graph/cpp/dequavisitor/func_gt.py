import numpy as np
import time

def find_closest_vertices(coord_list,vertices_latlon_list, MIN_DIST_FOR_THE_CLOSEST_NODE=100, verbose=False):
    """
    Returns list of nodes in vertices_latlon_list closest to coordinate_list (euclidean distance).
    """
    nodes_list=[]
    for coordinate in coord_list:
        # coordinate = np.asarray(d.get("coordinate"))
        #time1 = time.time()
        #tmp = np.subtract(np.ones(G_array.shape) * coordinate, G_array)
        #dists = np.sum(np.sqrt(tmp * tmp), axis=1)
        time2 = time.time()
        dists = distance_from_a_list_of_geo_coordinates(coordinate, vertices_latlon_list)
        time3 = time.time()
        # app.logger.debug("it took {} to calculate distances".format(time3-time2))
        if verbose == True:
            print(f"it took {time3-time2} to calculate distances")
        #dists=d.get("shape").distance(G_array)
        closest_id = np.argmin(dists)
        closest_dist = dists[closest_id]
        # app.logger.debug("il tuo nodo è distante {}".format(closest_dist))
        if verbose == True:
            print(f"il tuo nodo è distante {closest_dist}")
        # se la distanza e troppo grande, salutiamo i campagnoli
        if closest_dist>MIN_DIST_FOR_THE_CLOSEST_NODE:
            # app.logger.error("Sei troppo distante da Venezia, cosa ci fai là?? (il punto del grafo piu vicino dista {} metri)".format(closest_dist))
            if verbose == True:
                print("Sei troppo distante da Venezia, cosa ci fai là?? (il punto del grafo piu vicino dista {closest_dist} metri)")
            # raise custom_errors.UserError("Non abbiamo trovato nulla qua - magari cercavi di andare fuori venezia o forse vorresti andare in barca?")
            nodes_list.append(-1)
        nodes_list.append(closest_id)

    return nodes_list#, dists

def distance_from_a_list_of_geo_coordinates(thePoint, coordinates_list):
    """
    A python implementation from the answer here https://stackoverflow.com/questions/639695/how-to-convert-latitude-or-longitude-to-meters.
    Calculate the distance in meters between 1 geographical point (longitude, latitude) and a list of geographical points (list of tuples) or between 2 geographical points passing through distance_from_point_to_point
    """
    # maybe we need to invert
    lat_index = 1
    lon_index = 0
    # parameters
    earth_radius = 6378.137; # Radius of earth in KM
    deg2rad = np.pi / 180
    # single point
    lat1 = thePoint[lat_index] * deg2rad
    lon1 = thePoint[lon_index] * deg2rad
    # test the whole list again the single point
    lat2 = coordinates_list[:,lat_index] * deg2rad
    lon2 = coordinates_list[:,lon_index] * deg2rad
    dLat = lat2 - lat1
    dLon = lon2 - lon1
    a = np.sin(dLat/2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dLon/2) ** 2
    c = 2 * np.arcsin(np.sqrt(a))
    d = earth_radius * c
    distances_in_meters = d * 1000

    return distances_in_meters