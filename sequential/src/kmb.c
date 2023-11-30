#include "kmb.h"

#include <stdio.h>
#include <stdlib.h>

#include "graph.h"

void run_kmb()
{
    printf("Sequential version of KBM method for determining the Steiner Tree of a graph.\n");

    /* Step 0: Construct a graph G */
	int V = 9; // Number of nodes in the graph
    Graph* graph = create_graph(V); // increase the edges' weight's by 2
    add_edge(graph, 0, 1, 20);
    add_edge(graph, 0, 8, 2);
    add_edge(graph, 1, 2, 16);
    add_edge(graph, 1, 5, 2);
    add_edge(graph, 2, 4, 4);
    add_edge(graph, 2, 3, 18);
    add_edge(graph, 3, 4, 4);
    add_edge(graph, 4, 5, 2);
    add_edge(graph, 4, 8, 2);
    add_edge(graph, 5, 6, 2);
    add_edge(graph, 6, 7, 1);
    add_edge(graph, 7, 8, 1);

    print_graph(graph);

	int S = 4; // Number of steiner points
	int steiner_points[S];
	steiner_points[0] = 0;
	steiner_points[1] = 1;
	steiner_points[2] = 2;
	steiner_points[3] = 3;

    /* Step 1: Construct the complete undirected distance graph G1 */
	Graph* distance_graph = create_distance_graph(graph, steiner_points, S);
	print_graph(distance_graph);	

    /* Step 2: Find the minimal spanning tree T1 of the distance graph G1 */
    Graph* distance_graph_mst = prim_mst(distance_graph);
    print_graph(distance_graph_mst);

    /* Step 3: Construct the subgraph Gs of G by replacing each edge in T1 by its shortest path in G */
    Graph* subgraph = construct_subgraph_with_distance_graph_mst(graph, distance_graph_mst, steiner_points);
    print_graph(subgraph);

    /* Step 4: Find the minimal spanning tree Ts of Gs */
    Graph* subgraph_mst = prim_mst(subgraph);
    print_graph(subgraph_mst);

    /* Step 5: Construct a steiner tree Th out of tree Ts, by deleting all the edges so that all the steiner points are leaves */
    Graph* steiner_tree = subgraph_mst;
    construct_steiner_tree(steiner_tree, steiner_points, S);
    print_graph(steiner_tree);

    deallocate_graph_memory(steiner_tree); // == subgraph_mst
    deallocate_graph_memory(subgraph);
	deallocate_graph_memory(distance_graph);
	deallocate_graph_memory(graph);
}