#include <iostream>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <climits>
#include <string> 
#include <algorithm> 
#include <raylib.h>
#include <random>
#include <cmath>
#include <map>
#define LIGHTGRAY CLITERAL(Color){ 30, 30, 30, 255 }
#define YELLOW CLITERAL(Color){ 253, 249, 0, 255 }
struct point{
    int x, y;
    point(int x, int y): x(x),y(y) {}

    bool operator==(const point& other) const {
        return (x == other.x && y == other.y);
    }
};

class grid{
    private:
    std::vector<point> points;
    std::vector<point> nodes;
    std::map<int, std::vector<int>> dist_map;
    float dist(point point1, point point2)
    {
        return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));

    }
    public:
    //creating a background grid
    std::vector<point> cordinate_system_generation()
    {
        for(int x = 0;x<16;x++){
            for (int y = 0; y<16;y++){
                points.push_back(point(x,y));
            }
        }
        return points;
    }
    // creating random paths between the nodes
    void random_path(std::vector<point> nodes, int density, int max_dist)
    {
        static bool seeded = false;
        if (!seeded) 
        {
            srand(static_cast<unsigned int>(time(0)));
            seeded = true;
        }
        for (size_t i = 0; i < nodes.size(); ++i) 
        {
            for (size_t j = i + 1; j < nodes.size(); ++j)
            {
                float distance = dist(nodes[i],nodes[j]);
                if (distance<max_dist)
                {
                    DrawLine((nodes[i].x)*60,(nodes[i].y)*60,(nodes[j].x)*60,(nodes[j].y)*60,RAYWHITE);
                }
            }
        }
    }
    //creating the random nodes for the A* algorithm to work on
    std::vector<point> random_nodes()
    {
          srand(static_cast<unsigned int>(time(0)));
        while (nodes.size() < 30) {
            int x_cord = (rand() % 15)+1;
            int y_cord = (rand() % 15)+1;
            point new_point(x_cord, y_cord);
            if (std::find(nodes.begin(), nodes.end(), new_point) == nodes.end()) {
                nodes.push_back(new_point);
            }
        }
        return nodes;
    }
    void points_on_grid(std::vector<point> points)
    {
        for (std::vector<point>::iterator it = nodes.begin(); it != nodes.end(); ++it){
            DrawCircle((it->x)*60,(it->y)*60, 4, YELLOW); // Increased radius and used a visible color
        }
    }
    void on_screen_grid(const int cols, const int rows, const int cellWidth, const int cellHeight)
    {
        for (int i = 0; i < cols; i++) {
            for (int j = 0; j < rows; j++) {
                DrawRectangleLines(
                    i * cellWidth,
                    j * cellHeight,
                    cellWidth,
                    cellHeight,
                    LIGHTGRAY
                );
            }
        }
    }
};
int main()
{
    grid myGrid;
    const int screenWidth = 960;
    const int screenHeight = 960;
    const int rows = 16;
    const int cols = 16;
    const int cellWidth = screenWidth / cols;
    const int cellHeight = screenHeight / rows;

    InitWindow(screenWidth, screenHeight, "Raylib Grid Example");
    SetTargetFPS(60);

    // Generate points only once, before the main loop
    std::vector<point> all_nodes = myGrid.random_nodes();

    //checking start
    for (std::vector<point>::iterator it = all_nodes.begin(); it != all_nodes.end(); ++it){
        std::cout<< it->x << std::endl;
    }
    //chekcing end

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(BLACK);
        myGrid.random_path(all_nodes,50,6.5);
        myGrid.on_screen_grid(cols, rows, cellWidth, cellHeight);
        myGrid.points_on_grid(all_nodes); // Pass the vector of points

        EndDrawing();
    }

    CloseWindow();
    return 0;
}