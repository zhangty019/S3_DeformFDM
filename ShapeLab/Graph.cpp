#include "Graph.h"

// Allocates memory for adjacency list
Graph::Graph(int V, std::vector<int> endNode_NoSet)
{
	this->V = V;
	adj = new std::list<Dijk_Pair>[V];
	this->endNode_indexSet = endNode_NoSet;
}

void Graph::addEdge(int u, double w, int v){

	if (true) {
		//direction graph u->(weight)->v
		adj[u].push_back(std::make_pair(w, v));
	}
	else {
		//un-direction graph u<->(weight)<->v
		adj[u].push_back(std::make_pair(w, v));
		adj[v].push_back(std::make_pair(w, u));
	}
}

// Function to print shortest path from source to j using parent array 
void Graph::printPath(std::vector<int> parent, int j) {

	// Base Case : If j is source 
	if (parent[j] == -1)
		return;

	printPath(parent, parent[j]);

	//printf("%d ", j);
	path.push_back(j);
}

// Prints shortest paths from src to all other vertices
void Graph::shortestPath(int src)
{
	// Create a priority queue to store vertices that
	// are being preprocessed. This is weird syntax in C++.
	// Refer below link for details of this syntax
	// https://www.geeksforgeeks.org/implement-min-heap-using-stl/
	std::priority_queue< Dijk_Pair, std::vector <Dijk_Pair>, std::greater<Dijk_Pair> > pq;

	// Create a vector for distances and initialize all
	// distances as infinite (INF)
	std::vector<double> dist(V, INF);

	// Parent array to store shortest path tree 
	std::vector<int> parent(V, INF);
	parent[src] = -1;

	// Insert source itself in priority queue and initialize
	// its distance as 0.
	pq.push(std::make_pair(0.0, src));
	dist[src] = 0.0;

	std::vector<bool> f(V, false);

	/* Looping till priority queue becomes empty (or all
	distances are not finalized) */
	while (!pq.empty())
	{
		// The first vertex in pair is the minimum distance
		// vertex, extract it from priority queue.
		// vertex label is stored in second of pair (it
		// has to be done this way to keep the vertices
		// sorted distance (distance must be first item
		// in pair)
		int u = pq.top().second;
		pq.pop();
		f[u] = true;

		// 'i' is used to get all adjacent vertices of a vertex
		std::list< std::pair<double, int> >::iterator i;
		for (i = adj[u].begin(); i != adj[u].end(); ++i)
		{
			// Get vertex label and weight of current adjacent
			// of u.

			double weight = (*i).first;
			int v = (*i).second;

			// If there is shorted path to v through u.
			if (f[v] == false && dist[v] > dist[u] + weight)
			{
				// Updating distance of v
				parent[v] = u;
				dist[v] = dist[u] + weight;
				pq.push(std::make_pair(dist[v], v));
			}
		}
	}

	//Print shortest distances stored in dist[] and its path
	//protection
	if (endNode_indexSet.size() < 2) std::cout << "ERROR: the NUM of end node candidate is wrong! "<< std::endl;
	//printf("Vertex\t Distance\tPath");
	//find the index of end Node in Graph with minimum cost
	double minCost = INF; int minCost_Index = -1;
	for (int i = 0; i < endNode_indexSet.size(); i++){

		if (dist[endNode_indexSet[i]] < minCost) {
			minCost = dist[endNode_indexSet[i]];
			minCost_Index = endNode_indexSet[i];
		}
	}
	//find the index of Node in Graph with minimum cost
	//printf("\n%d -> %d \t\t %f\t\t%d ",	src, minCost_Index, minCost, src);
	//printf("%f\n",	minCost);
	//printf("Path:\n");
	path.push_back(src);
	printPath(parent, minCost_Index);
	//printf("\n");
}
