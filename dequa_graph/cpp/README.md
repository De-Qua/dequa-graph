To compile (change conda environment path):

```
g++ -std=c++17 -O2 -Wall -I/path/to/conda/envs/dequa/include -L/path/to/conda/envs/dequa/lib dijkstra-example.cpp -o dijkstra-example -lboost_graph
```