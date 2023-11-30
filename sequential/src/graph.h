#ifndef GRAPH_H
#define GRAPH_H

#include <stdio.h>
#include <stdlib.h>

/* -------------------- Structure's definitions -------------------- */

// Structure to represent an adjacency list node
typedef struct Edge
{
    int src;
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

// Structure to represent a path (sequence of edges)
typedef struct Path
{
    int total_weight;
    Edge* head;
} Path;

/* ----------------------------------------------------------------- */

/* ----------------- Graph's methods' declarations ----------------- */

Edge* create_edge(int src, int dest, int weight);

Graph* create_graph(int vertex_count);

Graph* create_distance_graph(Graph* graph, int steiner_points[], int steiner_points_count);

void add_edge(Graph* graph, int src, int dest, int weight);

void remove_edge(Graph* graph, int src, int dest);

void print_graph(Graph* graph);

/*
    Function to find the vertex with minimum distance value, from 
    the set of vertices not yet included in the shortest path
*/
int min_distance(int dist[], int shortest_path[], int vertex_count);

/*
    Function that implements Dijkstra's shortest path algorithm
    for a graph represented using adjacency list
    (client code needs to deallocate the memory of the returned Path object)
*/
Path* dijkstra_shortest_path(Graph* graph, int src, int dest);

/*
    Function to find the vertex with the minimum key value
    from the set of vertices not yet included in MST (Minimal Spanning Tree)
*/
int min_key(int key[], int mst_set[], int V);

/*
    Function to find weight of an edge from src to dest
*/
int find_weight(Graph* graph, int src, int dest);

/*
    Function that implements Prim's algorithm to
    construct and return the minimal spanning tree
*/
Graph* prim_mst(Graph* graph);

/*
    Function that constructs the subgraph Gs of G by replacing
    each edge in the distance_graph_mst by its corresponding
    shortest path in G.
*/
Graph* construct_subgraph_with_distance_graph_mst(Graph* graph, Graph* distance_graph_mst, int steiner_points[]);

/*
    Function that removes certain edges from the input
    graph (tree) so that it becomes a steiner tree.
*/
void construct_steiner_tree(Graph* graph, int steiner_points[], int S);

/*
    Function that checks whether a node of given vertex in
    the graph is a steiner point. It returns 1 if so, 0 otherwise.
*/
int is_steiner_point(int steiner_points[], int S, int vertex);

/*
    Function that checks whether a node of given vertex in
    the graph is a leaf. It returns 1 if so, 0 otherwise.
*/
int is_leaf(Graph* graph, int vertex);

void deallocate_path_memory(Path* path);

void deallocate_adjacency_list_memory(AdjacencyList* list);

void deallocate_graph_memory(Graph* graph);

/* ----------------------------------------------------------------- */

#endif // GRAPH_H