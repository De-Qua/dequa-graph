# dequa-graph

## How to install

To install dequa_graph you need graph-tool (version 2.37).

If you are installing dequa-graph together with dequa there should be no problems, otherwise you first need to have graph-tool and pkg-config already installed. The easiest way is to create a conda environment

```
conda create -n dequagraph python=3.9 graph-tool=2.37 pkg-config
```

Then you can pip install dequa_graph:

```
pip install git+https://github.com/De-Qua/dequa-graph.git
```

or from source after cloning the repository, from the main directory

```
pip install .
```

or if you are developing and you want to see changes in the code

```
pip install -e .
```

