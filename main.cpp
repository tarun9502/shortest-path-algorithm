#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <algorithm>

struct Edge
{
    int from, to;
    double weight;
};

class Graph
{
    std::vector<std::vector<Edge>> adjList;

public:
    Graph(int n) : adjList(n) {}

    void addEdge(int from, int to, double weight)
    {
        adjList[from].push_back({from, to, weight});
    }

    std::pair<double, std::vector<int>> shortestPath(int start, int end)
    {
        std::vector<double> dist(adjList.size(), INT_MAX);
        std::vector<int> prev(adjList.size(), -1);
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

        dist[start] = 0;
        pq.push({0, start});

        while (!pq.empty())
        {
            int u = pq.top().second;
            pq.pop();

            for (auto &edge : adjList[u])
            {
                int v = edge.to;
                double weight = edge.weight;

                if (dist[u] + weight < dist[v])
                {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }

        std::vector<int> path;
        for (int v = end; v != -1; v = prev[v])
            path.push_back(v);

        std::reverse(path.begin(), path.end());

        return {dist[end], path};
    }
};

int main()
{
    Graph g(5);
    g.addEdge(0, 1, 4);
    g.addEdge(0, 2, 1);
    g.addEdge(1, 3, 1);
    g.addEdge(2, 1, 2);
    g.addEdge(2, 3, 5);
    g.addEdge(3, 4, 3);

    int start, end;
    std::cout << "Enter start and end vertices: ";
    std::cin >> start >> end;

    auto [cost, path] = g.shortestPath(start, end);

    std::cout << "Cost of shortest path: " << cost << "\n";
    std::cout << "Path: ";
    for (int v : path)
        std::cout << v << " ";
    std::cout << "\n";

    return 0;
}
