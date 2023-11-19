#ifndef GRAPH_H
#define GRAPH_H

#include <stdio.h>
#include <stdlib.h>

/* -------------------- Structure's definitions -------------------- */

// Structure to represent an adjacency list node
typedef struct Edge
{
	int dest;
	int weight;
	struct Edge* next;
} Edge;

// Structure to represent an adjacency list
typedef struct AdjacencyList
{
	Edge* head;
} AdjacencyList;

// Structure to represent a graph. A graph is an array of adjacency lists.
typedef struct Graph
{
	int vertex_count;
	AdjacencyList* adj_list;
} Graph;

/* ----------------------------------------------------------------- */

/* ----------------- Graph's methods' declarations ----------------- */

Edge* create_edge(int dest, int weight);

Graph* create_graph(int vertex_count);

Graph* create_distance_graph(Graph* graph, int steiner_points[], int steiner_points_count);

void add_edge(Graph* graph, int src, int dest, int weight);

void print_graph(Graph* graph);

/*
    Function to find the vertex with minimum distance value, from 
    the set of vertices not yet included in the shortest path
*/
int min_distance(int dist[], int shortest_path[], int vertex_count);

/*
    Function that implements Dijkstra's shortest path algorithm
    for a graph represented using adjacency list
*/
int dijkstra_shortest_path(Graph* graph, int src, int dest);

/*
    Function to find the vertex with the minimum key value
    from the set of vertices not yet included in MST (Minimal Spanning Tree)
*/
int min_key(int key[], int mst_set[], int V);
/*
    Function that implements Prim's algorithm to
    construct and return the minimal spanning tree
*/
Graph* prim_mst(Graph* graph);

void deallocate_adjacency_list_memory(AdjacencyList* list);

void deallocate_graph_memory(Graph* graph);

/* ----------------------------------------------------------------- */

#endif // GRAPH_H