#include <iostream>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <climits>
#include <string> 
#include <algorithm> // Added for std::reverse

class graph {
private:
    int numvertices;
    int** matrix;

public:
    // Added const qualifier as this function does not modify the object
    int size() const { return numvertices; }
    // Added const qualifier
    const int* const* getmatrix() const { return matrix; }

    // Added const qualifier
    std::vector<int> connected_to(int vertex) const {
        std::vector<int> arr;
        for (int i = 0; i < numvertices; i++) {
            if (matrix[vertex][i] != 0) {
                arr.push_back(i);
            }
        }
        return arr;
    }

    graph(int vertices) {
        srand(static_cast<unsigned int>(time(0))); // Casted to avoid a compiler warning
        numvertices = vertices;
        matrix = new int* [numvertices];
        for (int i = 0; i < numvertices; i++) {
            matrix[i] = new int[numvertices];
            for (int j = 0; j < numvertices; j++) {
                matrix[i][j] = 0;
            }
        }
    }

    // Added a destructor to free allocated memory
    ~graph() {
        for (int i = 0; i < numvertices; ++i) {
            delete[] matrix[i];
        }
        delete[] matrix;
    }

    void weights(int density, int max_dist) {
        for (int i = 0; i < numvertices; i++) {
            for (int j = i + 1; j < numvertices; j++) {
                int n = rand() % 100;
                if (n < density) {
                    int dist = 1 + rand() % max_dist;
                    matrix[i][j] = matrix[j][i] = dist;
                }
            }
        }
    }

    void print() const {
        for (int i = 0; i < numvertices; i++) {
            for (int j = 0; j < numvertices; j++) {
                std::cout << matrix[i][j] << " ";
            }
            std::cout << "\n";
        }
    }
};

// This function finds the unvisited node with the minimum distance.
int min_distance_node(const std::vector<int>& dist_from_start, const std::vector<bool>& visited) {
    int min_dist = INT_MAX;
    int min_index = -1;
    for (int i = 0; i < dist_from_start.size(); i++) {
        if (!visited[i] && dist_from_start[i] <= min_dist) {
            min_dist = dist_from_start[i];
            min_index = i;
        }
    }
    return min_index;
}

// Rewritten dijkstra function with a correct loop structure and path tracking.
void dijkstra(const graph& g, int start, int end) {
    int size = g.size();
    std::vector<bool> visited(size, false);
    std::vector<int> dist_from_start(size, INT_MAX);
    std::vector<int> prev(size, -1); // Vector to store the predecessor of each node

    dist_from_start[start] = 0;

    const int* const* map = g.getmatrix();

    // The main loop for the algorithm.
    for (int count = 0; count < size; count++) {
        int current = min_distance_node(dist_from_start, visited);

        if (current == -1) {
            break;
        }

        visited[current] = true;

        if (current == end) {
            break;
        }

        // Relaxation step: Update distances of adjacent nodes.
        std::vector<int> connected_to = g.connected_to(current);
        for (int neighbor : connected_to) {
            if (!visited[neighbor]) {
                if (dist_from_start[current] + map[current][neighbor] < dist_from_start[neighbor]) {
                    dist_from_start[neighbor] = dist_from_start[current] + map[current][neighbor];
                    prev[neighbor] = current; // Store the predecessor
                }
            }
        }
    }

    std::cout << "Shortest distance from " << start << " to " << end
        << " = " << dist_from_start[end] << "\n";

    // Reconstruct and print the path
    if (dist_from_start[end] != INT_MAX) {
        std::vector<int> path;
        int current_node = end;
        while (current_node != -1) {
            path.push_back(current_node);
            current_node = prev[current_node];
        }
        std::reverse(path.begin(), path.end());

        std::cout << "Shortest path: ";
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << path[i];
            if (i < path.size() - 1) {
                std::cout << " -> ";
            }
        }
        std::cout << "\n";
    }
    else {
        std::cout << "Path is unreachable.\n";
    }
}

// ------------------ MAIN ------------------
int main() {
    graph g(15);
    g.weights(50, 10);
    std::cout << "Adjacency Matrix:\n";
    g.print();
    std::cout << "\n";

    int start = 3, end = 10;
    dijkstra(g, start, end);

    return 0;
}
