// graph-tool -- a general graph modification and manipulation thingy
// ... (Copyright headers unchanged) ...

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
#include <algorithm> // for std::min
#include <cmath>     // for std::isinf
#include <vector>
#include <limits>

#if (BOOST_VERSION >= 106000)
# include <boost/math/special_functions/relative_difference.hpp>
#endif

using namespace std;
using namespace boost;
using namespace graph_tool;

struct stop_search {};

const long double TOTAL_SECONDS_IN_WEEK = 24.0 * 7.0 * 3600.0;
const long double INF_WEIGHT = std::numeric_limits<long double>::infinity();

// Helper function to calculate waiting time (replicates Python calculate_waiting_time)
template <class TimetableVec>
long double calculate_waiting_time_cpp(const TimetableVec& timetable_vec, 
                                       long double current_time_from_source, 
                                       long double start_time_seconds)
{
    if (timetable_vec.size() == 0) {
        // WARNING: empty array (special days only?)
        return TOTAL_SECONDS_IN_WEEK;
    }

    long double current_absolute_time = current_time_from_source + start_time_seconds;
    long double moduled_week_time = fmod(current_absolute_time, TOTAL_SECONDS_IN_WEEK);
    
    // Find next departure
    long double min_wait = TOTAL_SECONDS_IN_WEEK; // Default to full week if nothing found
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
        // Wrap around: wait until first departure next week
        // Formula: first_departure + (TOTAL_WEEK - current_absolute_time)
        // But since we moduled, it's: first_departure + (TOTAL_WEEK - moduled_week_time)
        // Wait, python code: self.timetable[e].a[0] + TOTAL_SECONDS_IN_WEEK - (self.time_from_source[e.source()] + self.start_time)
        // Note: python uses un-moduled time for the subtraction part in the else branch? 
        // Let's look closely: 
        // else: return self.timetable[e].a[0] + TOTAL_SECONDS_IN_WEEK - (self.time_from_source[e.source()] + self.start_time)
        // This is equivalent to: timetable[0] + (TOTAL_WEEK - current_absolute_time)
        
        long double first_departure = (long double)timetable_vec[0];
        long double wait_wrap = first_departure + (TOTAL_SECONDS_IN_WEEK - current_absolute_time);
        
        // Handle potential negative wrap if current_absolute_time > TOTAL_WEEK (though fmod handles the lookup)
        // The python code subtracts the raw absolute time.
        if (wait_wrap < 0) {
             // Should theoretically not happen if logic holds, but safety check
             wait_wrap += TOTAL_SECONDS_IN_WEEK; 
        }
        return wait_wrap;
    }
}

template <class DistMap, class DynamicPropertyMapWrap>
class dequa_visitor:
    public boost::dijkstra_visitor<null_visitor>
{
public:
    dequa_visitor(DistMap dist_map,
                    typename property_traits<DistMap>::value_type max_dist,
                    typename property_traits<DistMap>::value_type inf,
                    size_t target, std::vector<size_t>& reached,
                    DynamicPropertyMapWrap weight,
                    typename vprop_map_t<double>::type time_from_source,
                    typename eprop_map_t<double>::type time_edges,
                    typename vprop_map_t<uint8_t>::type transport_property,
                    typename eprop_map_t<vector<int32_t>>::type timetable_property,
                    typename eprop_map_t<int32_t>::type direction_property,
                    double transport_change_penalty,
                    double start_time_seconds)
        : _dist_map(dist_map), _max_dist(max_dist), _inf(inf),
          _target(target), _reached(reached), 
          _weight(weight),
          _time_from_source(time_from_source),
          _time_edges(time_edges),
          _transport(transport_property),
          _timetable(timetable_property),
          _direction(direction_property),
          _transport_change_penalty(transport_change_penalty),
          _start_time_seconds(start_time_seconds) {}

    ~dequa_visitor() {}

    // Funzione helper per il calcolo dell'attesa (può essere privata o esterna)
    template <class TimetableVec>
    long double calculate_waiting_time_cpp(const TimetableVec& timetable_vec, 
                                           long double current_time_from_source) const
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
            long double first_departure = (long double)timetable_vec[0];
            long double wait_wrap = first_departure + (TOTAL_SECONDS_IN_WEEK - current_absolute_time);
            if (wait_wrap < 0) wait_wrap += TOTAL_SECONDS_IN_WEEK;
            return wait_wrap;
        }
    }

    template <class Graph>
    void examine_edge(typename graph_traits<Graph>::edge_descriptor e, Graph& g)
    {
        auto u = source(e, g);
        auto v = target(e, g);

        // 1. Controllo Direzione
        // Se la direzione dell'arco punta alla sorgente, è un arco vietato in questo senso
        if (_direction[e] == (int32_t)u) {
            put(_weight, e, INF_WEIGHT);
            return;
        }

        bool is_target_transport = (_transport[v] != 0);
        bool is_source_transport = (_transport[u] != 0);
        
        double edge_time_cost = _time_edges[e]; // Default: usa il tempo di viaggio precalcolato

        // 2. Logica Boarding / Alighting / Normal
        if (is_target_transport && !is_source_transport) {
            // --- BOARDING (Salita) ---
            long double waiting_time = calculate_waiting_time_cpp(_timetable[e], _time_from_source[u]);
            
            // Imposta il peso per Dijkstra
            put(_weight, e, waiting_time);
            
            // Il costo temporale per il "clock" è il tempo di attesa
            edge_time_cost = (double)waiting_time;
        }
        else {
            // --- NON BOARDING (Discesa o Spostamento normale) ---
            
            // Se stiamo scendendo (Sorgente è trasporto, Target non lo è)
            if (is_source_transport && !is_target_transport) {
                // Imposta la penalità sul peso per Dijkstra
                put(_weight, e, _transport_change_penalty);
                // Nota: edge_time_cost rimane _time_edges[e] (tempo di viaggio), 
                // come nella logica Python che usa time_edges per aggiornare il clock.
            }
            // Altrimenti (spostamento normale), il peso rimane quello passato (time_edges)
            // e il costo temporale è time_edges.
        }

        // 3. Aggiornamento del "Clock" (time_from_source) sul nodo target
        // Questo replica la logica Python che aggiorna time_from_source in examine_edge
        double new_time = _time_from_source[u] + edge_time_cost;
        
        // Verifica se il nodo target è già stato "toccato" (scoperto)
        // In assenza della mappa booleana esplicita, usiamo _dist_map != inf come proxy
        if (_dist_map[v] != _inf) {
            // Se già scoperto, prendi il minimo
            if (new_time < _time_from_source[v]) {
                _time_from_source[v] = new_time;
            }
        } else {
            // Se nuovo, assegna direttamente
            _time_from_source[v] = new_time;
        }
    }

    template <class Graph>
    void edge_relaxed(typename graph_traits<Graph>::edge_descriptor e, Graph& g)
    {
        // Stop search se abbiamo raggiunto il target (come nel Python)
        if (target(e, g) == _target) {
            throw stop_search();
        }
    }

    template <class Graph>
    void examine_vertex(typename graph_traits<Graph>::vertex_descriptor u, Graph&)
    {
        if (_dist_map[u] > _max_dist || u == _target) {
            throw stop_search();
        }
    }

    template <class Graph>
    void discover_vertex(typename graph_traits<Graph>::vertex_descriptor u, Graph&)
    {
        if (_dist_map[u] > _max_dist) {
            _unreached.push_back(u);
        }
    }

    template <class Graph>
    void finish_vertex(typename graph_traits<Graph>::vertex_descriptor u, Graph&)
    {
        if (_dist_map[u] <= _max_dist) {
            _reached.push_back(u);
        }
    }

private:
    DistMap _dist_map;
    typename property_traits<DistMap>::value_type _max_dist;
    typename property_traits<DistMap>::value_type _inf;
    size_t _target;
    std::vector<size_t> _unreached;
    std::vector<size_t>& _reached;
    DynamicPropertyMapWrap _weight;
    typename vprop_map_t<double>::type _time_from_source;
    typename eprop_map_t<double>::type _time_edges;
    typename vprop_map_t<uint8_t>::type _transport;
    typename eprop_map_t<vector<int32_t>>::type _timetable;
    typename eprop_map_t<int32_t>::type _direction;
    double _transport_change_penalty;
    double _start_time_seconds;
};

// ... (Rest of the file: dequa_do_djk_search and dequa_get_dists) ...
// Note: You must update the constructor call in dequa_do_djk_search to pass 'start_time_seconds'.
// Since the Python code calculates 'start_seconds' and passes it, you need to add this argument 
// to the C++ python binding function 'dequa_get_dists' and pass it down.

struct dequa_do_djk_search
{
    template <class Graph, class VertexIndexMap, class DistMap, class PredMap,
              class WeightMap>
    void operator()(const Graph& g, size_t source,
                    boost::python::object otarget_list,
                    VertexIndexMap vertex_index, DistMap dist_map,
                    PredMap pred_map, WeightMap aweight, long double max_dist,
                    std::vector<size_t>& reached, bool dag, 
                    typename vprop_map_t<double>::type time_from_source,
                    typename eprop_map_t<double>::type time_edges,
                    typename vprop_map_t<uint8_t>::type transport_property,
                    typename eprop_map_t<vector<int32_t>>::type timetable_property,
                    typename eprop_map_t<int32_t>::type direction_property,
                    double transport_change_penalty,
                    double start_time_seconds) // Added argument
    {
        // ... (existing code) ...
        auto target_list = get_array<int64_t, 1>(otarget_list);
        typedef typename property_traits<DistMap>::value_type dist_t;
        constexpr dist_t inf = (std::is_floating_point<dist_t>::value) ?
            numeric_limits<dist_t>::infinity() : numeric_limits<dist_t>::max();
        dist_t max_d = (max_dist > 0) ? max_dist : inf;
        gt_hash_set<std::size_t> tgt(target_list.begin(), target_list.end());
        dist_map[source] = 0;

        typedef typename property_traits<DistMap>::value_type dtype_t;
        typedef typename graph_traits<Graph>::edge_descriptor edge_t;
        DynamicPropertyMapWrap<dtype_t, edge_t> weight(aweight, edge_properties());

        try {
            if (tgt.size() <= 1) {
                size_t target = tgt.empty() ?
                    graph_traits<GraphInterface::multigraph_t>::null_vertex() :
                    *tgt.begin();
                
                dijkstra_shortest_paths_no_color_map_no_init
                        (g, vertex(source, g), pred_map, dist_map, weight,
                         vertex_index, std::less<dist_t>(),
                         boost::closed_plus<dist_t>(), inf, dist_t(),
                         dequa_visitor<DistMap, WeightMap>(
                            dist_map, max_d, inf, target, reached, 
                            weight,
                            time_from_source,
                            time_edges,
                            transport_property,
                            timetable_property,
                            direction_property,
                            transport_change_penalty,
                            start_time_seconds)); // Pass start_time
            }
            // ... (handle multiple targets if needed, similar update required) ...
        }
        catch (stop_search&) {}
    }
};

// Update dequa_get_dists signature to accept start_time_seconds
void dequa_get_dists(GraphInterface& gi, size_t source, boost::python::object tgt,
               boost::any dist_map, boost::any weight, boost::any pred_map,
               long double max_dist, bool bf, std::vector<size_t>& reached,
               boost::any time_from_source_in,
               boost::any time_edges_in,
               boost::any transport_property_in,
               boost::any timetable_property_in,
               boost::any direction_property_in,
               double start_time_seconds) // Added argument
{
    typedef property_map_type::apply<int64_t, GraphInterface::vertex_index_map_t>::type pred_map_t;
    pred_map_t pmap = any_cast<pred_map_t>(pred_map);
    double transport_change_penalty = 1.0; // Should ideally be passed from Python too

    typedef vprop_map_t<double>::type vprop_double;
    typedef eprop_map_t<double>::type eprop_double;
    typedef vprop_map_t<uint8_t>::type vprop_bool;
    typedef eprop_map_t<vector<int32_t>>::type eprop_vecint;
    typedef eprop_map_t<int32_t>::type eprop_int;

    vprop_double time_from_source = any_cast<vprop_double>(time_from_source_in);
    eprop_double time_edges = any_cast<eprop_double>(time_edges_in);
    vprop_bool transport_property = any_cast<vprop_bool>(transport_property_in);
    eprop_vecint timetable_property = any_cast<eprop_vecint>(timetable_property_in);
    eprop_int direction_property = any_cast<eprop_int>(direction_property_in); 

    bool dag = false;

    run_action<>()
                (gi,
                 [&](auto&& graph, auto&& a2)
                 {
                     return dequa_do_djk_search()
                         (std::forward<decltype(graph)>(graph), source, tgt,
                          gi.get_vertex_index(), std::forward<decltype(a2)>(a2),
                          pmap.get_unchecked(num_vertices(gi.get_graph())),
                          weight, max_dist, reached,
                          dag, 
                          time_from_source,
                          time_edges,
                          transport_property,
                          timetable_property,
                          direction_property,
                          transport_change_penalty,
                          start_time_seconds);
                 },
                 writable_vertex_scalar_properties())(dist_map);
}

BOOST_PYTHON_MODULE(lib_dequadistance)
{
    using namespace boost::python;
    // Update the definition to include the new argument
    def("dequa_get_dists", dequa_get_dists);
}