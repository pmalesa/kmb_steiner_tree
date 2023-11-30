#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "graph.h"

/* ------------------ Graph's methods' definitions ------------------ */

Edge* create_edge(int src, int dest, int weight)
{
	Edge* new_node = (Edge*) malloc(sizeof(Edge));
	new_node->src = src;
	new_node->dest = dest;
	new_node->weight = weight;
	new_node->next = NULL;
	return new_node;
}

Graph* create_graph(int vertex_count)
{
	Graph* graph = (Graph*) malloc(sizeof(Graph));
	graph->vertex_count = vertex_count;

	graph->adj_list = (AdjacencyList*) malloc(vertex_count * sizeof(AdjacencyList));

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
			Path* path = dijkstra_shortest_path(graph, src, dest);
			int min_distance = path->total_weight;
			deallocate_path_memory(path);
			add_edge(distance_graph, src, dest, min_distance);
		}
	}

	return distance_graph;
}

void add_edge(Graph* graph, int src, int dest, int weight)
{
	Edge* new_edge = create_edge(src, dest, weight);
	new_edge->next = graph->adj_list[src].head;
	graph->adj_list[src].head = new_edge;

	new_edge = create_edge(dest, src, weight);
	new_edge->next = graph->adj_list[dest].head;
	graph->adj_list[dest].head = new_edge;
}

void remove_edge(Graph* graph, int src, int dest)
{
	if (graph->adj_list[src].head == NULL || graph->adj_list[dest].head == NULL)
		return;

	// Remove an edge from src to dest 
	Edge* temp = graph->adj_list[src].head;
	Edge* prev = NULL;

	while (temp != NULL)
	{
		if (temp->dest == dest)
		{
			if (prev == NULL) // Edge is the first node in the list
			{
				graph->adj_list[src].head = temp->next;
			}
			else
			{
				prev->next = temp->next;
			}
			free(temp);
			break;
		}
		prev = temp;
		temp = temp->next;
	}

	// Remove an edge from dest to src
	temp = graph->adj_list[dest].head;
	prev = NULL;

	while (temp != NULL)
	{
		if (temp->dest == src)
		{
			if (prev == NULL)
			{
				graph->adj_list[dest].head = temp->next;
			}
			else
			{
				prev->next = temp->next;
			}
			free(temp);
			break;
		}
		prev = temp;
		temp = temp->next;
	}
}

void print_graph(Graph* graph)
{
	printf("\n|--------------------------------GRAPH---------------------------------|\n");
	for (int v = 0; v < graph->vertex_count; ++v)
	{
		if (graph->adj_list[v].head == NULL)
			continue;
			
		Edge* edge = graph->adj_list[v].head;
		printf("Vertex (%d)\n    connects with:", v);
		while (edge)
		{
			printf("  (%d)[weight = %d]", edge->dest, edge->weight);
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

Path* dijkstra_shortest_path(Graph* graph, int src, int dest)
{
    int vertex_count = graph->vertex_count;
    int dist[vertex_count]; // Output array, where dist[i] holds the shortest distance from src to i
	int prev[vertex_count]; // Array to store the shortest path tree

    // shortest_path[i] will be true if vertex i is included in the shortest path
    // or shortest distance from src to i is finalized
    int shortest_path[vertex_count];

    // Initialize all distances as infinite (INT_MAX) and shortest_path[] elements as false (0)
    for (int i = 0; i < vertex_count; ++i)
    {
        dist[i] = INT_MAX;
        shortest_path[i] = 0;
		prev[i] = -1;
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
				prev[v] = min_index;
                dist[v] = dist[min_index] + edge->weight;
            }
        }
    }

    // printf("Shortest distance from %d to %d is %d\n", src, dest, dist[dest]);

	Path* path = (Path*) malloc(sizeof(Path));
	path->total_weight = dist[dest];
	path->head = NULL;

	// Backtrack from the destination to source to find the edge weights
	Edge* current_edge = NULL;
	int u = dest;
	while (prev[u] != -1)
	{
		int weight = find_weight(graph, prev[u], u);
		Edge* new_edge = create_edge(prev[u], u, weight);

		if (current_edge == NULL)
			path->head = new_edge;
		else
			current_edge->next = new_edge;

		current_edge = new_edge;
		u = prev[u];
	}

	return path;
}

int min_key(int key[], int mst_set[], int V)
{
	return min_distance(key, mst_set, V);
}

int find_weight(Graph* graph, int src, int dest)
{
	if (graph == NULL)
		return -1; // Error value

	Edge* edge = graph->adj_list[src].head;
	while (edge != NULL)
	{
		if (edge->dest == dest)
			return edge->weight;
		edge = edge->next;
	}

	return -1; // Error value
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

Graph* construct_subgraph_with_distance_graph_mst(Graph* graph, Graph* distance_graph_mst, int steiner_points[])
{
	if (graph == NULL || distance_graph_mst == NULL)
		return NULL;

	int vertex_count = graph->vertex_count;
	int steiner_point_count = distance_graph_mst->vertex_count;
	Graph* subgraph = create_graph(vertex_count);

	// Create matrix to memorize already added edges
	int already_added[vertex_count][vertex_count];
	for (int i = 0; i < vertex_count; ++i)
		for (int j = 0; j < vertex_count; ++j)
			already_added[i][j] = 0;

	for (int vertex = 0; vertex < steiner_point_count; ++vertex)
	{
		int path_src = steiner_points[vertex];
		Edge* edge = distance_graph_mst->adj_list[vertex].head;

		// Iterate over all edges in the distance graph MST
		while (edge)
		{
			int path_dest = steiner_points[edge->dest];
			Path* path = dijkstra_shortest_path(graph, path_src, path_dest);

			// Iterate over all path's edges
			while (path->head)
			{
				int src = path->head->src;
				int dest = path->head->dest;
				int weight = path->head->weight;
				if (already_added[src][dest] == 0)
				{
					add_edge(subgraph, src, dest, weight);
					// printf("[%d -> %d (%d)]\n", src, dest, weight);
					already_added[src][dest] = 1;
					already_added[dest][src] = 1;
				}
				path->head = path->head->next;
			}

			deallocate_path_memory(path);
			edge = edge->next;
		}
	}

	return subgraph;
}

void construct_steiner_tree(Graph* graph, int steiner_points[], int S)
{
	if (graph == NULL || S < 1 || S > graph->vertex_count)
		return;

	for (int vertex = 0; vertex < graph->vertex_count; ++vertex)
	{
		if (is_leaf(graph, vertex) && !is_steiner_point(steiner_points, S, vertex))
		{
			int src = graph->adj_list[vertex].head->src;
			int dest = graph->adj_list[vertex].head->dest;
			remove_edge(graph, src, dest);
		}
	} 
}

int is_steiner_point(int steiner_points[], int S, int vertex)
{
	if (vertex < 0 || vertex > S - 1)
		return 0;

	for (int i = 0; i < S; ++i)
	{
		if (vertex == steiner_points[i])
			return 1;
	}	

	return 0;
}

int is_leaf(Graph* graph, int vertex)
{
	if (graph->vertex_count < 1 || vertex < 0 || vertex > graph->vertex_count - 1)
		return 0;

	return graph->adj_list[vertex].head != NULL && graph->adj_list[vertex].head->next == NULL;
}

void deallocate_path_memory(Path* path)
{
	if (path == NULL)
		return;

	while (path->head)
	{
		Edge* current = path->head;
		path->head = path->head->next;
		free(current);
	}
}

void deallocate_adjacency_list_memory(AdjacencyList* list)
{
	if (list->head == NULL)
		return;

	while (list->head)
	{
		Edge* current = list->head;
		list->head = list->head->next;
		free(current);
	}
}

void deallocate_graph_memory(Graph* graph)
{
	for (int i = 0; i < graph->vertex_count; ++i)
	{
		deallocate_adjacency_list_memory(&graph->adj_list[i]);
	}
	free(graph->adj_list);
	free(graph);
}