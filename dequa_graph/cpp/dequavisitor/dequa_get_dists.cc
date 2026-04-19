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
#include "graph_util.hh"
#include "coroutine.hh"
#include "../generation/sampler.hh"

#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
#include <boost/python.hpp>

#include <fstream>

#if (BOOST_VERSION >= 106000)
#include <boost/math/special_functions/relative_difference.hpp>
#endif

using namespace std;
using namespace boost;
using namespace graph_tool;

struct stop_search
{
};

// TO BE DELETED: Kept for backup
template <class DistMap>
class djk_max_visitor : public boost::dijkstra_visitor<null_visitor>
{
public:
    djk_max_visitor(DistMap dist_map,
                    typename property_traits<DistMap>::value_type max_dist,
                    typename property_traits<DistMap>::value_type inf,
                    size_t target, std::vector<size_t> &reached)
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
                        Graph &)
    {
        if (_dist_map[u] > _max_dist || u == _target)
            throw stop_search();
    }

    template <class Graph>
    void discover_vertex(typename graph_traits<Graph>::vertex_descriptor u,
                         Graph &)
    {
        if (_dist_map[u] > _max_dist)
            _unreached.push_back(u);
    }

    template <class Graph>
    void finish_vertex(typename graph_traits<Graph>::vertex_descriptor u,
                       Graph &)
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
    std::vector<size_t> &_reached;
};

// Keep it for multitarget until we don't have dequa_multiple_target_visitor
template <class DistMap>
class djk_max_multiple_targets_visitor : public boost::dijkstra_visitor<null_visitor>
{
public:
    djk_max_multiple_targets_visitor(DistMap dist_map,
                                     typename property_traits<DistMap>::value_type max_dist,
                                     typename property_traits<DistMap>::value_type inf,
                                     gt_hash_set<std::size_t> target,
                                     std::vector<size_t> &reached)
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
                        Graph &)
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
                         Graph &)
    {
        if (_dist_map[u] > _max_dist)
            _unreached.push_back(u);
    }

    template <class Graph>
    void finish_vertex(typename graph_traits<Graph>::vertex_descriptor u,
                       Graph &)
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
    std::vector<size_t> &_reached;
};

const long double TOTAL_SECONDS_IN_WEEK = 24.0 * 7.0 * 3600.0;

template <class DistMap, class WeightMap>
class dequa_visitor : public boost::dijkstra_visitor<null_visitor>
{
public:
    dequa_visitor(DistMap dist_map,
                  typename property_traits<DistMap>::value_type max_dist,
                  typename property_traits<DistMap>::value_type inf,
                  size_t target, std::vector<size_t> &reached,
                  WeightMap weight,
                  typename vprop_map_t<double>::type time_from_source,
                  typename eprop_map_t<double>::type time_edges,
                  typename vprop_map_t<uint8_t>::type transport_property,
                  typename eprop_map_t<vector<int32_t>>::type timetable_property,
                  typename eprop_map_t<int32_t>::type direction_property,
                  typename vprop_map_t<uint8_t>::type touched_v,
                  double transport_change_penalty,
                  double start_time_seconds)
        : _dist_map(dist_map), _max_dist(max_dist), _inf(inf),
          _target(target), _reached(reached), // reached è touched_v di python?
          _weight(weight),
          _time_from_source(time_from_source),
          _time_edges(time_edges),
          _transport(transport_property),
          _timetable(timetable_property),
          _direction(direction_property),
          _touched_v(touched_v),
          _transport_change_penalty(transport_change_penalty),
          _start_time_seconds(start_time_seconds) {}

    template <class Graph>
    void discover_vertex(typename graph_traits<Graph>::vertex_descriptor u,
                         Graph &g)
    {
        _touched_v[u] = 1;

    }

    template <class Graph>
    void examine_edge(typename graph_traits<Graph>::edge_descriptor e,
                      Graph &g)
    {
        if (_direction[e] == source(e, g))
        {
            // Funziona! Tutto perché weight doveva essere writable edge property
            _weight[e] = _inf;
            return;
        }
        if (_transport[target(e, g)] == true && _transport[source(e, g)] == false)
        {
            long double waiting_time = calculate_waiting_time_cpp(_timetable[e], _time_from_source[source(e,g)], _start_time_seconds);
            _weight[e] = waiting_time;
            double new_time = _time_from_source[source(e,g)] + waiting_time;
            if (_touched_v[target(e,g)]) 
            {
                if (new_time < _time_from_source[target(e,g)])
                {
                    _time_from_source[target(e,g)] = new_time;
                }
            }
            else 
            {
                _time_from_source[target(e,g)] = new_time;
            }
        }
        else
        {
            if (_transport[source(e,g)] == true && _transport[target(e,g)] == false)
            {
                _weight[e] = _transport_change_penalty;
            }
            double new_time_smontando = _time_from_source[source(e,g)] + _time_edges[e];
            if (_touched_v[target(e,g)])
            {
                if (new_time_smontando < _time_from_source[target(e,g)])
                {
                    _time_from_source[target(e,g)] = new_time_smontando;
                }
            }
            else
            {
                _time_from_source[target(e,g)] = new_time_smontando;
            }
        }
    }

    // EDGE RELAXED == PYTHON
    template <class Graph>
    void edge_relaxed(typename graph_traits<Graph>::edge_descriptor e, Graph &g)
    {
        // std::cout << "DEQUA edge relaxed" << "\n";
        // // Aggiungi il tempo al nodo target
        // _time_from_source[source(e, g)];
        if (target(e, g) == _target) {
            // std::cout << "DEQUA edge relaxed - STOP SEARCH" << "\n";
            throw stop_search();
        }
    }

    // template <class Graph>
    // void examine_vertex(typename graph_traits<Graph>::vertex_descriptor u,
    //                     Graph &)
    // {
    //     // std::cout << "DEQUA examine vertex" << "\n";
    //     if (_dist_map[u] > _max_dist || u == _target)
    //         throw stop_search();
    // }

    

    // template <class Graph>
    // void finish_vertex(typename graph_traits<Graph>::vertex_descriptor u,
    //                    Graph &g)
    // {
    //     // std::cout << "DEQUA Finish vertex" << "\n";
    //     if (_dist_map[u] <= _max_dist)
    //         _reached.push_back(u);
    // }

    // Funzione helper per il calcolo dell'attesa (può essere privata o esterna)
    template <class TimetableVec>
    long double calculate_waiting_time_cpp(const TimetableVec& timetable_vec, 
                                           long double current_time_from_source,
                                            long double start_time_seconds) const
    {
        if (timetable_vec.size() == 0) {
            return TOTAL_SECONDS_IN_WEEK;
        }

        long double current_absolute_time = current_time_from_source + _start_time_seconds;
        long double moduled_week_time = fmod(current_absolute_time, TOTAL_SECONDS_IN_WEEK);
        
        long double min_wait = TOTAL_SECONDS_IN_WEEK;
        bool found_future = false;

        for (int32_t departure_time : timetable_vec) {
            long double diff = (long double)departure_time - moduled_week_time;
            if (diff > 0) {
                if (diff < min_wait) {
                    min_wait = diff;
                    found_future = true;
                }
            }
        }

        if (found_future) {
            return min_wait;
        } else {
            long double wait_wrap = TOTAL_SECONDS_IN_WEEK - (current_time_from_source + start_time_seconds);
            if (wait_wrap > 0)
                return timetable_vec[0] + wait_wrap;
        }
        return TOTAL_SECONDS_IN_WEEK;
    }

private:
    DistMap _dist_map;
    typename property_traits<DistMap>::value_type _max_dist;
    typename property_traits<DistMap>::value_type _inf;
    size_t _target;
    std::vector<size_t> _unreached;
    std::vector<size_t> &_reached;
    // DynamicPropertyMapWrap _weight;
    WeightMap _weight;
    typename vprop_map_t<double>::type _time_from_source;
    typename eprop_map_t<double>::type _time_edges;
    typename vprop_map_t<uint8_t>::type _transport;
    typename eprop_map_t<vector<int32_t>>::type _timetable;
    typename eprop_map_t<int32_t>::type _direction;
    typename vprop_map_t<uint8_t>::type _touched_v;
    double _transport_change_penalty;
    double _start_time_seconds;
};

struct dequa_do_djk_search
{
    template <class Graph, class VertexIndexMap, class DistMap, class PredMap,
              class WeightMap>
    void operator()(const Graph &g, size_t source,
                    boost::python::object otarget_list,
                    VertexIndexMap vertex_index, DistMap dist_map,
                    PredMap pred_map, WeightMap aweight, long double max_dist,
                    // PredMap pred_map, boost::any& aweight, long double max_dist,
                    std::vector<size_t> &reached, bool dag,
                    typename vprop_map_t<double>::type time_from_source,
                    typename eprop_map_t<double>::type time_edges,
                    typename vprop_map_t<uint8_t>::type transport_property,
                    typename eprop_map_t<vector<int32_t>>::type timetable_property,
                    typename eprop_map_t<int32_t>::type direction_property,
                    typename vprop_map_t<uint8_t>::type touched_v,
                    double transport_change_penalty,
                    double start_time_seconds) const
    {
        auto target_list = get_array<int64_t, 1>(otarget_list);

        typedef typename property_traits<DistMap>::value_type dist_t;

        constexpr dist_t inf = (std::is_floating_point<dist_t>::value) ? numeric_limits<dist_t>::infinity() : numeric_limits<dist_t>::max();

        dist_t max_d = (max_dist > 0) ? max_dist : inf;

        gt_hash_set<std::size_t> tgt(target_list.begin(),
                                     target_list.end());

        dist_map[source] = 0;

        // WeightMap weight = aweight;

        typedef typename property_traits<DistMap>::value_type dtype_t;
        // dtype_t z = python::extract<dtype_t>(range.first);
        // dtype_t i = python::extract<dtype_t>(range.second);
        typedef typename graph_traits<Graph>::edge_descriptor edge_t;
        // DynamicPropertyMapWrap<dtype_t, edge_t> weight(aweight,
        //                                                edge_properties());

        try
        {
            if (tgt.size() <= 1)
            {
                size_t target = tgt.empty() ? graph_traits<GraphInterface::multigraph_t>::null_vertex() : *tgt.begin();

                dijkstra_shortest_paths_no_color_map_no_init(g, vertex(source, g), pred_map, dist_map, aweight,
                                                             vertex_index, std::less<dist_t>(),
                                                             boost::closed_plus<dist_t>(), inf, dist_t(),
                                                             dequa_visitor<DistMap, WeightMap>(
                                                                 dist_map, max_d, inf, target, reached,
                                                                 aweight,
                                                                 time_from_source,
                                                                 time_edges,
                                                                 transport_property,
                                                                 timetable_property,
                                                                 direction_property,
                                                                 touched_v,
                                                                 transport_change_penalty,
                                                                 start_time_seconds));
            }
            else
            {
                dijkstra_shortest_paths_no_color_map_no_init(g, vertex(source, g), pred_map, dist_map, aweight,
                                                             vertex_index, std::less<dist_t>(),
                                                             boost::closed_plus<dist_t>(), inf, dist_t(),
                                                             djk_max_multiple_targets_visitor<DistMap>(dist_map,
                                                                                                       max_d, inf,
                                                                                                       tgt,
                                                                                                       reached));
            }
        }
        catch (stop_search &)
        {
        }
    }
};

struct DequaProperties {
    boost::any time_from_source;
    boost::any time_edges;
    boost::any transport_property;
    boost::any timetable_property;
    boost::any direction_property;
    boost::any touched_v;
};

void dequa_get_dists(GraphInterface &gi, size_t source, boost::python::object tgt,
                     boost::any dist_map, boost::any weight, boost::any pred_map,
                     long double max_dist, bool bf, std::vector<size_t> &reached,
                     DequaProperties props,
                    double start_time_seconds,
                    double transport_change_penalty)
{
    typedef property_map_type ::apply<int64_t, GraphInterface::vertex_index_map_t>::type pred_map_t;

    pred_map_t pmap = any_cast<pred_map_t>(pred_map);

    typedef vprop_map_t<double>::type vprop_double;
    typedef eprop_map_t<double>::type eprop_double;
    typedef vprop_map_t<uint8_t>::type vprop_bool;
    typedef eprop_map_t<vector<int32_t>>::type eprop_vecint;
    typedef eprop_map_t<int32_t>::type eprop_int;

    // std::cout << "siamo qua!";

    vprop_double time_from_source = any_cast<vprop_double>(props.time_from_source);
    eprop_double time_edges = any_cast<eprop_double>(props.time_edges);
    vprop_bool transport_property = any_cast<vprop_bool>(props.transport_property);
    vprop_bool touched_property = any_cast<vprop_bool>(props.touched_v);
    eprop_vecint timetable_property = any_cast<eprop_vecint>(props.timetable_property);
    eprop_int direction_property = any_cast<eprop_int>(props.direction_property);
    // vprop_double time_from_source;
    // eprop_double time_edges;
    // eprop_vecint timetable_property;
    // eprop_int direction_property;
    // std::cout << "siamo quadopo!";

    bool dag = false;

    run_action<>()(gi,
                   //  [&](auto&& graph, auto&& a2, auto&& a3)
                   //  {
                   //      return do_djk_search()
                   //          (std::forward<decltype(graph)>(graph), source, tgt,
                   //           gi.get_vertex_index(), std::forward<decltype(a2)>(a2),
                   //           pmap.get_unchecked(num_vertices(gi.get_graph())),
                   //           std::forward<decltype(a3)>(a3), max_dist, reached,
                   //           dag,
                   //           time_from_source,
                   //           time_edges,
                   //           transport_property,
                   //           timetable_property,
                   //           direction_property,
                   //           transport_change_penalty);
                   //  },
                   //  writable_vertex_scalar_properties(),
                   //  edge_scalar_properties())(dist_map, weight);
                   [&](auto &&graph, auto &&a2, auto &&weight)
                   { return dequa_do_djk_search()(std::forward<decltype(graph)>(graph), source, tgt,
                                                  gi.get_vertex_index(), std::forward<decltype(a2)>(a2),
                                                  pmap.get_unchecked(num_vertices(gi.get_graph())),
                                                  weight, max_dist, reached,
                                                  dag,
                                                  time_from_source,
                                                  time_edges,
                                                  transport_property,
                                                  timetable_property,
                                                  direction_property,
                                                  touched_property,
                                                  transport_change_penalty,
                                                  start_time_seconds); }, 
                                                  writable_vertex_scalar_properties(),
                                                  writable_edge_scalar_properties())(dist_map, weight);
}

BOOST_PYTHON_MODULE(lib_dequadistance)
{
    using namespace boost::python;
    class_<DequaProperties>("DequaProperties")
        .def_readwrite("time_from_source", &DequaProperties::time_from_source)
        .def_readwrite("time_edges", &DequaProperties::time_edges)
        .def_readwrite("transport_property", &DequaProperties::transport_property)
        .def_readwrite("timetable_property", &DequaProperties::timetable_property)
        .def_readwrite("direction_property", &DequaProperties::direction_property)
        .def_readwrite("touched_v", &DequaProperties::touched_v);
    def("dequa_get_dists", dequa_get_dists);
}