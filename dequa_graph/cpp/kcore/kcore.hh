#include "graph_tool.hh"

using namespace graph_tool;
using namespace boost;
using namespace std;

// This template function takes a graph 'g' and a vertex property map 'core_map'
// where the core values will be stored. Their exact types are unspecified at
// this point.

template <typename Graph, typename CoreMap>
void kcore_decomposition(Graph& g, CoreMap core_map)
{
    // The vertex index is an internal property map that every graph possesses
    typedef typename property_map<Graph, vertex_index_t>::type
        vertex_index_map_t;
    vertex_index_map_t vertex_index = get(vertex_index_t(), g);

    // Create some auxiliary property maps
    typedef typename vprop_map_t<size_t>::type::unchecked_t vmap_t;

    vmap_t deg(vertex_index, num_vertices(g));  // Remaining degree
    vmap_t pos(vertex_index, num_vertices(g));  // Position in bin (core)

    typedef typename graph_traits<Graph>::vertex_descriptor vertex_t;

    vector<vector<vertex_t>> bins; // Each bin stores the set of vertices of a
                                   // core

    // Put each vertex to the bin corresponding to its degree
    for (auto v : vertices_range(g))
    {
        size_t k = out_degree(v, g);
        deg[v] = k;
        if (k >= bins.size())
            bins.resize(k + 1);
        bins[k].push_back(v);
        pos[v] = bins[k].size() - 1;
    }

    // Proceed from smallest bin to largest. For each vertex in bin, check the
    // neighbours; if any of them have a larger remaining degree, reduce it by
    // one, and put it in the correct bin.
    for (size_t k = 0; k < bins.size(); ++k)
    {
        auto& bins_k = bins[k];
        while (!bins_k.empty())
        {
            auto v = bins_k.back();
            bins_k.pop_back();
            core_map[v] = k;
            for (auto e : out_edges_range(v, g))
            {
                auto u = target(e, g);
                auto& ku = deg[u];
                if (ku > deg[v])
                {
                    auto& bins_ku = bins[ku];
                    auto w = bins_ku.back();
                    auto pos_w = pos[w] = pos[u];
                    bins_ku[pos_w] = w;
                    bins_ku.pop_back();
                    auto& bins_ku_m = bins[ku - 1];
                    bins_ku_m.push_back(u);
                    pos[u] = bins_ku_m.size() - 1;
                    --ku;
                }
            }
        }
    }
}
