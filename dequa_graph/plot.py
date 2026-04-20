import matplotlib.pyplot as plt
import contextily as ctx
import folium

def draw_gt_path_notebook(path_v, pos):
    
    path_v_geom = [pos[v] for v in path_v]
    # Create a Folium map centered at the first node
    start_lonlat = pos[path_v[0]]
    end_lonlat = pos[path_v[-1]]
    central_lon = start_lonlat[0]+(end_lonlat[0]-start_lonlat[0])/2
    central_lat = start_lonlat[1]+(end_lonlat[1]-start_lonlat[1])/2

    m = folium.Map(location=(central_lat,central_lon), zoom_start=15, tiles="CartoDB Positron")
    # # Add nodes as markers
    # for lon, lat in path_v_geom:
    #     folium.CircleMarker(location=(lat, lon), popup=f"({lat}, {lon})", radius=1, color="red", fill=True, fill_color="red", fill_opacity=0.8).add_to(m)

    # Add edges (lines between nodes)
    for (lon1, lat1), (lon2, lat2) in zip(path_v_geom[:-1], path_v_geom[1:]):
        folium.PolyLine([(lat1, lon1), (lat2, lon2)], weight=3, opacity=0.7).add_to(m)

    # Display the map
    return m

def draw_gt_path(path_v, pos):
    path_v_geom = [pos[v] for v in path_v]
    
    lons = [p[0] for p in path_v_geom]
    lats = [p[1] for p in path_v_geom]

    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Disegna il percorso
    ax.plot(lons, lats, color='blue', linewidth=2, alpha=0.7)
    
    # Aggiungi punto di inizio e fine
    ax.scatter([lons[0]], [lats[0]], color='green', s=100, zorder=5, label='Start')
    ax.scatter([lons[-1]], [lats[-1]], color='red', s=100, zorder=5, label='End')
    
    ax.legend()

    # Aggiungi mappa di sfondo (richiede che le coordinate siano in Web Mercator EPSG:3857)
    # Se le tue coordinate sono in WGS84 (lat/lon), devi prima convertirle
    ctx.add_basemap(ax, crs="EPSG:4326", source=ctx.providers.CartoDB.Positron)
    
    plt.tight_layout()
    plt.show()

def draw_gt_paths(paths, pos, labels=None, colors=None):
    """
    paths  : lista di percorsi, ognuno è una lista di nodi  -> [[v1,v2,...], [v1,v2,...], ...]
    pos    : dizionario nodo -> (lon, lat)
    labels : lista di etichette per la legenda (opzionale)
    colors : lista di colori per ogni percorso (opzionale)
    """
    default_colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
    
    if not isinstance(paths[0], list):
        paths = [paths]
    if colors is None:
        colors = [default_colors[i % len(default_colors)] for i in range(len(paths))]
    if labels is None:
        labels = [f"Path {i+1}" for i in range(len(paths))]

    fig, ax = plt.subplots(figsize=(10, 8))

    for path_v, label, color in zip(paths, labels, colors):
        path_v_geom = [pos[v] for v in path_v]
        lons = [p[0] for p in path_v_geom]
        lats = [p[1] for p in path_v_geom]

        ax.plot(lons, lats, color=color, linewidth=2, alpha=0.7, label=label)
        ax.scatter([lons[0]], [lats[0]], color=color, s=100, zorder=5, marker='o')   # Start
        ax.scatter([lons[-1]], [lats[-1]], color=color, s=100, zorder=5, marker='s') # End

    ax.legend()
    ctx.add_basemap(ax, crs="EPSG:4326", source=ctx.providers.CartoDB.Positron)

    plt.tight_layout()
    plt.show()