// graph-tool -- a general graph modification and manipulation thingy
//
// Copyright (C) 2006-2022 Tiago de Paula Peixoto <tiago@skewed.de>
//
// This program is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 3 of the License, or (at your option) any
// later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
// details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#include "graph.hh"
#include "graph_filtering.hh"
#include "graph_properties.hh"
#include "graph_selectors.hh"
#include "graph_python_interface.hh"
#include "numpy_bind.hh"
#include "hash_map_wrap.hh"
#include "coroutine.hh"
#include "../generation/sampler.hh"

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
#include <boost/graph/bellman_ford_shortest_paths.hpp>
#include <boost/graph/dag_shortest_paths.hpp>
#include <boost/python.hpp>

#if (BOOST_VERSION >= 106000)
# include <boost/math/special_functions/relative_difference.hpp>
#endif

using namespace std;
using namespace boost;
using namespace graph_tool;

struct stop_search {};

template <class DistMap, class PredMap>
class bfs_max_visitor:
    public boost::bfs_visitor<null_visitor>
{
public:
    bfs_max_visitor(DistMap dist_map, PredMap pred, size_t max_dist,
                    size_t source, size_t target, std::vector<size_t>& reached)
        : _dist_map(dist_map), _pred(pred), _max_dist(max_dist),
          _source(source), _target(target), _dist(0), _reached(reached) {}

    ~bfs_max_visitor()
    {
        typedef typename property_traits<DistMap>::value_type dist_t;
        constexpr dist_t inf = std::is_floating_point<dist_t>::value ?
            numeric_limits<dist_t>::infinity() :
            numeric_limits<dist_t>::max();
        for (auto v : _unreached)
            _dist_map[v] = inf;
    }

    template <class Graph>
    void tree_edge(typename graph_traits<Graph>::edge_descriptor e,
                   Graph& g)
    {
        _pred[target(e,g)] = source(e,g);
    }

    template <class Graph>
    void examine_vertex(typename graph_traits<Graph>::vertex_descriptor v,
                        Graph&)
    {
        if ( _dist_map[v] > _max_dist)
            throw stop_search();
    }

    template <class Graph>
    void discover_vertex(typename graph_traits<Graph>::vertex_descriptor v,
                         Graph&)
    {
        auto p = _pred[v];
        if (size_t(p) == v)
            return;
        _dist_map[v] = _dist_map[p] + 1;
        if (_dist_map[v] > _max_dist)
            _unreached.push_back(v);
        else
            _reached.push_back(v);
        if (v == _target)
            throw stop_search();
    }

private:
    DistMap _dist_map;
    PredMap _pred;
    typename property_traits<DistMap>::value_type _max_dist;
    size_t _source;
    size_t _target;
    size_t _dist;
    std::vector<size_t> _unreached;
    std::vector<size_t>& _reached;
};

template <class DistMap, class PredMap>
class bfs_max_multiple_targets_visitor:
    public boost::bfs_visitor<null_visitor>
{
public:
    bfs_max_multiple_targets_visitor(DistMap dist_map, PredMap pred,
                                     size_t max_dist, size_t source,
                                     gt_hash_set<std::size_t> target,
                                     std::vector<size_t>& reached)
        : _dist_map(dist_map), _pred(pred), _max_dist(max_dist),
          _source(source), _target(target), _dist(0), _reached(reached) {}

    ~bfs_max_multiple_targets_visitor()
    {
        typedef typename property_traits<DistMap>::value_type dist_t;
        constexpr dist_t inf = std::is_floating_point<dist_t>::value ?
            numeric_limits<dist_t>::infinity() :
            numeric_limits<dist_t>::max();
        for (auto v : _unreached)
            _dist_map[v] = inf;
    }

    template <class Graph>
    void tree_edge(typename graph_traits<Graph>::edge_descriptor e,
                   Graph& g)
    {
        _pred[target(e,g)] = source(e,g);
    }

    template <class Graph>
    void examine_vertex(typename graph_traits<Graph>::vertex_descriptor v,
                        Graph&)
    {
        if ( _dist_map[v] > _max_dist)
            throw stop_search();
    }

    template <class Graph>
    void discover_vertex(typename graph_traits<Graph>::vertex_descriptor v,
                         Graph&)
    {
        auto p = _pred[v];
        if (size_t(p) == v)
            return;
        _dist_map[v] = _dist_map[p] + 1;

        if (_dist_map[v] > _max_dist)
            _unreached.push_back(v);

        auto iter = _target.find(v);
        if (iter != _target.end())
        {
            _target.erase(iter);
            if (_target.empty())
                throw stop_search();
        };
    }

private:
    DistMap _dist_map;
    PredMap _pred;
    typename property_traits<DistMap>::value_type _max_dist;
    size_t _source;
    gt_hash_set<std::size_t> _target;
    size_t _dist;
    std::vector<size_t> _unreached;
    std::vector<size_t>& _reached;
};


template <class DistMap>
class djk_max_visitor:
    public boost::dijkstra_visitor<null_visitor>
{
public:
    djk_max_visitor(DistMap dist_map,
                    typename property_traits<DistMap>::value_type max_dist,
                    typename property_traits<DistMap>::value_type inf,
                    size_t target, std::vector<size_t>& reached)
        : _dist_map(dist_map), _max_dist(max_dist), _inf(inf),
          _target(target), _reached(reached) {}

    ~djk_max_visitor()
    {
        for (auto v : _unreached)
        {
            if (_dist_map[v] > _max_dist)
                _dist_map[v] = _inf;
        }
    }

    template <class Graph>
    void examine_vertex(typename graph_traits<Graph>::vertex_descriptor u,
                        Graph&)
    {
        if (_dist_map[u] > _max_dist || u == _target)
            throw stop_search();
    }

    template <class Graph>
    void discover_vertex(typename graph_traits<Graph>::vertex_descriptor u,
                         Graph&)
    {
        if (_dist_map[u] > _max_dist)
            _unreached.push_back(u);
    }

    template <class Graph>
    void finish_vertex(typename graph_traits<Graph>::vertex_descriptor u,
                       Graph&)
    {
        if (_dist_map[u] <= _max_dist)
            _reached.push_back(u);
    }

private:
    DistMap _dist_map;
    typename property_traits<DistMap>::value_type _max_dist;
    typename property_traits<DistMap>::value_type _inf;
    size_t _target;
    std::vector<size_t> _unreached;
    std::vector<size_t>& _reached;
};


template <class DistMap>
class djk_max_multiple_targets_visitor:
    public boost::dijkstra_visitor<null_visitor>
{
public:
    djk_max_multiple_targets_visitor(DistMap dist_map,
                                     typename property_traits<DistMap>::value_type max_dist, 
                                     typename property_traits<DistMap>::value_type inf, 
                                     gt_hash_set<std::size_t> target,
                                     std::vector<size_t>& reached)
        : _dist_map(dist_map), _max_dist(max_dist), _inf(inf),
          _target(target), _reached(reached) {}

    ~djk_max_multiple_targets_visitor()
    {
        for (auto v : _unreached)
        {
            if (_dist_map[v] > _max_dist)
                _dist_map[v] = _inf;
        }
    }

    template <class Graph>
    void examine_vertex(typename graph_traits<Graph>::vertex_descriptor u,
                        Graph&)
    {
        if (_dist_map[u] > _max_dist)
            throw stop_search();

        auto iter = _target.find(u);
        if (iter != _target.end())
        {
            _target.erase(iter);
            if (_target.empty())
                throw stop_search();
        };
    }

    template <class Graph>
    void discover_vertex(typename graph_traits<Graph>::vertex_descriptor u,
                         Graph&)
    {
        if (_dist_map[u] > _max_dist)
            _unreached.push_back(u);
    }

    template <class Graph>
    void finish_vertex(typename graph_traits<Graph>::vertex_descriptor u,
                       Graph&)
    {
        if (_dist_map[u] <= _max_dist)
            _reached.push_back(u);
    }

private:
    DistMap _dist_map;
    typename property_traits<DistMap>::value_type _max_dist;
    typename property_traits<DistMap>::value_type _inf;
    gt_hash_set<std::size_t> _target;
    std::vector<size_t> _unreached;
    std::vector<size_t>& _reached;
};

struct do_bfs_search
{
    template <class Graph, class VertexIndexMap, class DistMap, class PredMap>
    void operator()(const Graph& g, size_t source,
                    boost::python::object otarget_list,
                    VertexIndexMap vertex_index, DistMap dist_map,
                    PredMap pred_map, long double max_dist,
                    std::vector<size_t>& reached) const
    {
        typedef typename property_traits<DistMap>::value_type dist_t;

        auto target_list = get_array<int64_t, 1>(otarget_list);
        gt_hash_set<std::size_t> tgt(target_list.begin(),
                                     target_list.end());

        dist_t inf = std::is_floating_point<dist_t>::value ?
            numeric_limits<dist_t>::infinity() :
            numeric_limits<dist_t>::max();

        dist_t max_d = (max_dist > 0) ? max_dist : inf;

        dist_map[source] = 0;

        unchecked_vector_property_map<boost::default_color_type, VertexIndexMap>
        color_map(vertex_index, num_vertices(g));
        try
        {
            if (tgt.size() <= 1)
            {
                size_t target = tgt.empty() ?
                    graph_traits<GraphInterface::multigraph_t>::null_vertex() :
                    *tgt.begin();
                breadth_first_visit(g, vertex(source, g),
                                    visitor(bfs_max_visitor<DistMap, PredMap>
                                            (dist_map, pred_map, max_d,
                                             source, target, reached)).
                                    vertex_index_map(vertex_index).
                                    color_map(color_map));
            }
            else
            {
                breadth_first_visit(g, vertex(source, g),
                                    visitor(bfs_max_multiple_targets_visitor<DistMap, PredMap>
                                            (dist_map, pred_map, max_d,
                                             source, tgt, reached)).
                                    vertex_index_map(vertex_index).
                                    color_map(color_map));
            }

        }
        catch (stop_search&) {}
    }
};

struct do_djk_search
{
    template <class Graph, class VertexIndexMap, class DistMap, class PredMap,
              class WeightMap>
    void operator()(const Graph& g, size_t source,
                    boost::python::object otarget_list,
                    VertexIndexMap vertex_index, DistMap dist_map,
                    PredMap pred_map, WeightMap weight, long double max_dist,
                    std::vector<size_t>& reached, bool dag) const
    {
        auto target_list = get_array<int64_t, 1>(otarget_list);

        typedef typename property_traits<DistMap>::value_type dist_t;

        constexpr dist_t inf = (std::is_floating_point<dist_t>::value) ?
            numeric_limits<dist_t>::infinity() :
            numeric_limits<dist_t>::max();

        dist_t max_d = (max_dist > 0) ? max_dist : inf;

        gt_hash_set<std::size_t> tgt(target_list.begin(),
                                     target_list.end());

        dist_map[source] = 0;

        try
        {
            if (tgt.size() <= 1)
            {
                size_t target = tgt.empty() ?
                    graph_traits<GraphInterface::multigraph_t>::null_vertex() :
                    *tgt.begin();
                if (!dag)
                {
                    dijkstra_shortest_paths_no_color_map_no_init
                        (g, vertex(source, g), pred_map, dist_map, weight,
                         vertex_index, std::less<dist_t>(),
                         boost::closed_plus<dist_t>(), inf, dist_t(),
                         djk_max_visitor<DistMap>(dist_map, max_d, inf, target,
                                                  reached));
                }
                else
                {
                    unchecked_vector_property_map<boost::default_color_type, VertexIndexMap>
                        color_map(vertex_index, num_vertices(g));
                    dag_shortest_paths
                        (g, vertex(source, g), dist_map, weight,
                         color_map, pred_map,
                         djk_max_visitor<DistMap>(dist_map, max_d, inf, target,
                                                  reached),
                         std::less<dist_t>(),
                         boost::closed_plus<dist_t>(), inf, dist_t());
                }
            }
            else
            {
                if (!dag)
                {
                    dijkstra_shortest_paths_no_color_map_no_init
                        (g, vertex(source, g), pred_map, dist_map, weight,
                         vertex_index, std::less<dist_t>(),
                         boost::closed_plus<dist_t>(), inf, dist_t(),
                         djk_max_multiple_targets_visitor<DistMap>(dist_map,
                                                                   max_d, inf,
                                                                   tgt,
                                                                   reached));
                }
                else
                {
                    unchecked_vector_property_map<boost::default_color_type, VertexIndexMap>
                        color_map(vertex_index, num_vertices(g));
                    dag_shortest_paths
                        (g, vertex(source, g), dist_map, weight, color_map,
                         pred_map,
                         djk_max_multiple_targets_visitor<DistMap>(dist_map,
                                                                   max_d, inf,
                                                                   tgt,
                                                                   reached),
                         std::less<dist_t>(), boost::closed_plus<dist_t>(), inf,
                         dist_t());
                }
            }

        }
        catch (stop_search&) {}

    }
};

struct do_bf_search
{
    template <class Graph, class DistMap, class PredMap, class WeightMap>
    void operator()(const Graph& g, size_t source, DistMap dist_map,
                    PredMap pred_map, WeightMap weight) const
    {
        bool ret = bellman_ford_shortest_paths(g, root_vertex(source).
                                               predecessor_map(pred_map).
                                               distance_map(dist_map).
                                               weight_map(weight));
        if (!ret)
            throw ValueException("Graph contains negative loops");

        // consistency with dijkstra
        typedef typename property_traits<DistMap>::value_type dist_t;
        if (std::is_floating_point<dist_t>::value)
        {
            for (auto v : vertices_range(g))
            {
                if (dist_map[v] == numeric_limits<dist_t>::max())
                    dist_map[v] = numeric_limits<dist_t>::infinity();
            }
        }
    }
};

void dequa_get_dists(GraphInterface& gi, size_t source, boost::python::object tgt,
               boost::any dist_map, boost::any weight, boost::any pred_map,
               long double max_dist, bool bf, std::vector<size_t>& reached,
               bool dag)
{
    typedef property_map_type
        ::apply<int64_t, GraphInterface::vertex_index_map_t>::type pred_map_t;

    pred_map_t pmap = any_cast<pred_map_t>(pred_map);

    if (weight.empty())
    {
        run_action<>()
            (gi,
             [&](auto&& graph, auto&& a2)
             {
                 return do_bfs_search()
                     (std::forward<decltype(graph)>(graph), source, tgt,
                      gi.get_vertex_index(), std::forward<decltype(a2)>(a2),
                      pmap.get_unchecked(num_vertices(gi.get_graph())),
                      max_dist, reached);
             },
             writable_vertex_scalar_properties())(dist_map);
    }
    else
    {
        if (bf)
        {
            run_action<>()
                (gi,
                 [&](auto&& graph, auto&& a2, auto&& a3)
                 {
                     return do_bf_search()
                         (std::forward<decltype(graph)>(graph), source,
                          std::forward<decltype(a2)>(a2),
                          pmap.get_unchecked(num_vertices(gi.get_graph())),
                          std::forward<decltype(a3)>(a3));
                 },
                 writable_vertex_scalar_properties(),
                 edge_scalar_properties())(dist_map, weight);
        }
        else
        {
            run_action<>()
                (gi,
                 [&](auto&& graph, auto&& a2, auto&& a3)
                 {
                     return do_djk_search()
                         (std::forward<decltype(graph)>(graph), source, tgt,
                          gi.get_vertex_index(), std::forward<decltype(a2)>(a2),
                          pmap.get_unchecked(num_vertices(gi.get_graph())),
                          std::forward<decltype(a3)>(a3), max_dist, reached,
                          dag);
                 },
                 writable_vertex_scalar_properties(),
                 edge_scalar_properties())(dist_map, weight);
        }
    }
}

BOOST_PYTHON_MODULE(lib_dequadistance)
{
    using namespace boost::python;
    def("dequa_get_dists", dequa_get_dists);
}