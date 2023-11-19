#include "kmb.h"

#include <stdio.h>
#include <stdlib.h>

#include "graph.h"

void run_kmb()
{
    printf("Sequential version of KBM method for determining the Steiner Tree of a graph.\n");

	int V = 9; // Number of nodes in the graph
    Graph* graph = create_graph(V);
    add_edge(graph, 0, 1, 10);
    add_edge(graph, 0, 8, 1);
    add_edge(graph, 1, 2, 8);
    add_edge(graph, 1, 5, 1);
    add_edge(graph, 2, 4, 2);
    add_edge(graph, 2, 3, 9);
    add_edge(graph, 3, 4, 2);
    add_edge(graph, 4, 5, 1);
    add_edge(graph, 4, 8, 1);
    add_edge(graph, 5, 6, 1);
    add_edge(graph, 6, 7, 1);
    add_edge(graph, 7, 8, 1);

    print_graph(graph);

	int S = 4; // Number of steiner points
	int steiner_points[S];
	steiner_points[0] = 0;
	steiner_points[1] = 1;
	steiner_points[2] = 2;
	steiner_points[3] = 3;
	Graph* distance_graph = create_distance_graph(graph, steiner_points, S);
	print_graph(distance_graph);	

    Graph* distance_graph_mst = prim_mst(distance_graph);
    print_graph(distance_graph_mst);

	deallocate_graph_memory(distance_graph);
	deallocate_graph_memory(graph);
}