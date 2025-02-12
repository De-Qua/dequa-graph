#include <boost/python.hpp>
#include "kcore.hh"

// The function below will take the graph that comes from graph-tool (always an
// instance of GraphInterface) and the property map (always an instance of
// boost::any).

void kcore_bind(GraphInterface& gi, boost::any core_map)
{
    // We don't know the actual type of the graph represented by 'gi' and the
    // property map 'core_map'. We need to evoke the appropriate instance of the
    // algorithm at run-time using the gt_dispatch<>() function.
    //
    // The gt_dispatch<>() function takes as first argument the template
    // function object that will be called with the correct types, and the
    // remaining (variadic) arguments are the list of types that will be
    // considered for every argument of the function passed in the first
    // argument. In the case below 'all_graph_views()' represents all possible
    // graph views (directed, undirected, filtered, reversed, etc.) and
    // 'writable_vertex_scalar_properties' represents all vertex property maps
    // with scalar types that are writable (this excludes the vertex index
    // map). If we had more graphs or property maps to pass, we would simply
    // increase the parameter list accordingly.
    //
    // The gt_dispatch<>() function returns an object that needs to be called
    // with the specific objects that should be used for the dispatched call. In
    // this case we extract the actual view from `gi.get_graph_view()` and pass
    // the `core_map`.

    gt_dispatch<>()
        ([&](auto& g, auto core){ kcore_decomposition(g, core); },
         all_graph_views(), writable_vertex_scalar_properties())
        (gi.get_graph_view(), core_map);
};

// The lines below setup a Python module called 'libkcore' that reflects the
// function 'kcore_bind' above as 'kcore' when imported from Python.

BOOST_PYTHON_MODULE(libkcore)
{
    using namespace boost::python;
    def("kcore", kcore_bind);
}
