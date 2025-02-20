from graph_tool import _prop, Vector_int32_t, Vector_size_t, _check_prop_writable, \
     _check_prop_scalar, _check_prop_vector, Graph, VertexPropertyMap, \
     PropertyMap, GraphView, libcore, _get_rng, perfect_prop_hash, _limit_args

import numpy, collections

import lib_dequadistance

def dequa_shortest_distance(g, source=None, target=None, weights=None,
                      negative_weights=False, max_dist=None, directed=None,
                      dense=False, dist_map=None, pred_map=False,
                      return_reached=False, dag=False):


    tgtlist = False
    if isinstance(target, collections.Iterable):
        tgtlist = True
        target = numpy.asarray(target, dtype="int64")
    elif target is None:
        target = numpy.array([], dtype="int64")
    else:
        target = numpy.asarray([int(target)], dtype="int64")

    if weights is None:
        dist_type = 'int32_t'
    else:
        dist_type = weights.value_type()

    if dist_map is None:
        if source is not None:
            dist_map = g.new_vertex_property(dist_type)
        else:
            dist_map = g.new_vertex_property("vector<%s>" % dist_type)

    _check_prop_writable(dist_map, name="dist_map")
    if source is not None:
        _check_prop_scalar(dist_map, name="dist_map")
    else:
        _check_prop_vector(dist_map, name="dist_map")

    if max_dist is None:
        max_dist = 0

    if directed is not None:
        u = GraphView(g, directed=directed)
    else:
        u = g

    if source is not None:
        if numpy.issubdtype(dist_map.a.dtype, numpy.integer):
            dist_map.fa = numpy.iinfo(dist_map.a.dtype).max
        else:
            dist_map.fa = numpy.inf
        if isinstance(pred_map, PropertyMap):
            pmap = pred_map
            if pmap.value_type() != "int64_t":
                raise ValueError("supplied pred_map must be of value type 'int64_t'")
        else:
            pmap = u.copy_property(u.vertex_index, value_type="int64_t")
        reached = libcore.Vector_size_t()
        lib_dequadistance.dequa_get_dists(u._Graph__graph,
                                         int(source),
                                         target,
                                         _prop("v", u, dist_map),
                                         _prop("e", u, weights),
                                         _prop("v", u, pmap),
                                         float(max_dist),
                                         negative_weights, reached, dag)
    else:
        raise ValueError("Sourcceeeeeeeee missing!!!")

    if source is not None and len(target) > 0:
        if len(target) == 1 and not tgtlist:
            dist_map = dist_map.a[target[0]]
        else:
            dist_map = numpy.array(dist_map.a[target])

    if source is not None:
        if pred_map:
            ret = (dist_map, pmap)
        else:
            ret = (dist_map,)
        if return_reached:
            return ret + (numpy.asarray(reached.a.copy()),)
        else:
            if len(ret) == 1:
                return ret[0]
            return ret
    else:
        return dist_map

def dequa_shortest_path(g, source, target, weights=None, negative_weights=False,
                  pred_map=None, dag=False):

    if pred_map is None:
        pred_map = dequa_shortest_distance(g, source, target, weights=weights,
                                     negative_weights=negative_weights,
                                     pred_map=True, dag=dag)[1]

    if pred_map[target] == int(target):  # no path to target
        return [], []

    vlist = [target]
    elist = []

    if weights is not None:
        max_w = weights.a.max() + 1
    else:
        max_w = None

    source = g.vertex(source)
    target = g.vertex(target)
    v = target
    while v != source:
        p = g.vertex(pred_map[v])
        min_w = max_w
        pe = None
        s = None
        for e in v.in_edges() if g.is_directed() else v.out_edges():
            s = e.source() if g.is_directed() else e.target()
            if s == p:
                if weights is not None:
                    if weights[e] < min_w:
                        min_w = weights[e]
                        pe = e
                else:
                    pe = e
                    break
        elist.insert(0, pe)
        vlist.insert(0, p)
        v = p
    return vlist, elist
