#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include<bits/stdc++.h>
using namespace std;

// A utility function to find the vertex with minimum distance
// value, from the set of vertices not yet included in shortest
// path tree
int minDistance(double dist[], bool sptSet[], int V)
{
    // Initialize min value
    double min = DBL_MAX;
    int min_index;

    for (int v = 0; v < V; v++)
        if (sptSet[v] == false && dist[v] <= min)
            min = dist[v], min_index = v;

    return min_index;
}

// Function to print shortest path from source to j
// using parent array
void printPath(int parent[], int j, vector<int>& p)
{
    // Base Case : If j is source
    if (parent[j]==-1)
        return;

    printPath(parent, parent[j],p);
    p.push_back(j);
}

// A utility function to print the constructed distance
// array
vector<int> printSolution(double dist[], int V, int parent[])
{
    vector<int> p;
    p.push_back(0);
    printPath(parent,V - 1,p);
    return p;
}

// Funtion that implements Dijkstra's single source shortest path
// algorithm for a graph represented using adjacency matrix
// representation
vector<int> dijkstra(double **g, int src, int V)
{
    double dist[V];  // The output array. dist[i] will hold
                  // the shortest distance from src to i

    // sptSet[i] will true if vertex i is included / in shortest
    // path tree or shortest distance from src to i is finalized
    bool sptSet[V];

    // Parent array to store shortest path tree
    int parent[V];

    // Initialize all distances as INFINITE and stpSet[] as false
    for (int i = 0; i < V; i++)
    {
        parent[0] = -1;
        dist[i] = DBL_MAX;
        sptSet[i] = false;
    }

    // Distance of source vertex from itself is always 0
    dist[src] = 0;

    // Find shortest path for all vertices
    for (int count = 0; count < V-1; count++)
    {
        // Pick the minimum distance vertex from the set of
        // vertices not yet processed. u is always equal to src
        // in first iteration.
        int u = minDistance(dist, sptSet, V);

        // Mark the picked vertex as processed
        sptSet[u] = true;

        // Update dist value of the adjacent vertices of the
        // picked vertex.
        for (int v = 0; v < V; v++)

            // Update dist[v] only if is not in sptSet, there is
            // an edge from u to v, and total weight of path from
            // src to v through u is smaller than current value of
            // dist[v]
            if (!sptSet[v] && g[u][v] &&
                dist[u] + g[u][v] < dist[v])
            {
                parent[v]  = u;
                dist[v] = dist[u] + g[u][v];
            }
    }

    // print the constructed distance array
    return printSolution(dist, V, parent);
}

#endif // DIJKSTRA_H
