#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "graph.h"

/* ------------------ Graph's methods' definitions ------------------ */

Edge* create_edge(int dest, int weight)
{
	Edge* new_node = (Edge*) malloc(sizeof(Edge));
	new_node->dest = dest;
	new_node->weight = weight;
	new_node->next = NULL;
	return new_node;
}

Graph* create_graph(int vertex_count)
{
	Graph* graph = (Graph*) malloc(sizeof(Graph));
	graph->vertex_count = vertex_count;

	graph->adj_list = (AdjacencyList*) malloc(vertex_count * sizeof(Edge));

	for (int i = 0; i < vertex_count; ++i)
	{
		graph->adj_list[i].head = NULL;
	}

	return graph;
}

Graph* create_distance_graph(Graph* graph, int steiner_points[], int steiner_points_count)
{
	Graph* distance_graph = create_graph(steiner_points_count);

	for (int i = 0; i < steiner_points_count; ++i)
	{
		for (int j = i + 1; j < steiner_points_count; ++j)
		{
			int src = steiner_points[i];
			int dest = steiner_points[j];
			int min_distance = dijkstra_shortest_path(graph, src, dest);
			add_edge(distance_graph, src, dest, min_distance);
		}
	}

	return distance_graph;
}

void add_edge(Graph* graph, int src, int dest, int weight)
{
	Edge* new_edge = create_edge(dest, weight);
	new_edge->next = graph->adj_list[src].head;
	graph->adj_list[src].head = new_edge;

	new_edge = create_edge(src, weight);
	new_edge->next = graph->adj_list[dest].head;
	graph->adj_list[dest].head = new_edge;
}

void print_graph(Graph* graph)
{
	printf("|----------------------------------------------------------------------|\n");
	for (int v = 0; v < graph->vertex_count; ++v)
	{
		Edge* edge = graph->adj_list[v].head;
		printf("Vertex %d\n head", v);
		while (edge)
		{
			printf(" -> %d(weight = %d)", edge->dest, edge->weight);
			edge = edge->next;
		}
		printf("\n");
	}
	printf("|----------------------------------------------------------------------|\n");
}

int min_distance(int dist[], int shortest_path[], int vertex_count)
{
    int min = INT_MAX;
    int min_index;

    for (int v = 0; v < vertex_count; ++v)
    {
        if (shortest_path[v] == 0 && dist[v] <= min)
        {
            min = dist[v];
            min_index = v;
        }
    }

    return min_index;
}

int dijkstra_shortest_path(Graph* graph, int src, int dest)
{
    int vertex_count = graph->vertex_count;
    int dist[vertex_count]; // Output array, where dist[i] holds the shortest distance from src to i

    // shortest_path[i] will be true if vertex i is included in the shortest path
    // or shortest distance from src to i is finalized
    int shortest_path[vertex_count];

    // Initialize all distances as infinite (INT_MAX) and shortest_path[] elements as false (0)
    for (int i = 0; i < vertex_count; ++i)
    {
        dist[i] = INT_MAX;
        shortest_path[i] = 0;
    }

    // Distance of the source vertex from itself is always 0
    dist[src] = 0;

    // Find shortest path for all vertices
    for (int count = 0; count < vertex_count - 1; ++count)
    {
        int min_index = min_distance(dist, shortest_path, vertex_count);

        // Mark the picked vertex as visited
        shortest_path[min_index] = 1;

        // Update dist value of the adjacent vertices of the picked vertex
        for (Edge* edge = graph->adj_list[min_index].head; edge != NULL; edge = edge->next)
        {
            int v = edge->dest; 
            if (!shortest_path[v] && dist[min_index] != INT_MAX && dist[min_index] + edge->weight < dist[v])
            {
                dist[v] = dist[min_index] + edge->weight;
            }
        }
    }

    printf("Shortest distance from %d to %d is %d\n", src, dest, dist[dest]);

    return dist[dest];
}

int min_key(int key[], int mst_set[], int V)
{
	return min_distance(key, mst_set, V);
}

Graph* prim_mst(Graph* graph)
{
	int vertex_count = graph->vertex_count;
	int parent[vertex_count];	// array to store constructed MST
	int key[vertex_count];		// key values used to pick minimum weight edge in cut
	int mst_set[vertex_count];	// array to represent set of vertices not yet included in MST

	// Initialize all keys as infinite (INT_MAX) and mst_set[] elements as false (0)
	for (int i = 0; i < vertex_count; ++i)
	{
		key[i] = INT_MAX;
		mst_set[i] = 0;
	}

	// Include first vertex in MST. Make key 0 so that this vertex is picked as the first one
	key[0] = 0;
	parent[0] = -1; // first node is always the root of MST

	// The MST will	have vertex_count vertices
	for (int count = 0; count < vertex_count - 1; ++count)
	{
		// Pick the minimum key vertex from the set of vertices not yet included in MST
		int u = min_key(key, mst_set, vertex_count);

		// Add the picked vertex to the MST set
		mst_set[u] = 1;

		// Update key value and parent index of the adjacent vertices of the picked vertex
		for (Edge* edge = graph->adj_list[u].head; edge != NULL; edge = edge->next)
		{
			int dest = edge->dest;

			// mst_set[dest] is false for vertices not yet included in MST
			// Update the key only if the weight of an edge between u and dest is smaller than key[dest]
			if (!mst_set[dest] && edge->weight < key[dest])
			{
				parent[dest] = u;
				key[dest] = edge->weight;
			}
		}
	}

	// Create a graph to represent the MST
	Graph* mst = create_graph(vertex_count);
	for (int i = 1; i < vertex_count; ++i)
	{
		add_edge(mst, parent[i], i, key[i]);
	}

	return mst;
}

void deallocate_adjacency_list_memory(AdjacencyList* list)
{
	while (list->head)
	{
		Edge* temp = list->head;
		list->head = list->head->next;
		free(temp);
	}
}

void deallocate_graph_memory(Graph* graph)
{
	for (int i = 0; i < graph->vertex_count; ++i)
	{
		deallocate_adjacency_list_memory(&graph->adj_list[i]);
	}
	free(graph);
}