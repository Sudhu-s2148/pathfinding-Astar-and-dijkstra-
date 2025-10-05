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
    std::map<int, std::vector<std::pair<int,float>>> dist_map;
    std::map<std::pair<int,int>, float> heuristic_dist_map;
    float dist(point point1, point point2)
    {
        return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));

    }
    public:
    std::map<std::pair<int,int>, float>& getheuristicDistMap() 
        {
            return heuristic_dist_map;
        }
    std::map<int, std::vector<std::pair<int,float>>>& getDistMap() 
        {
            return dist_map;
        }
    std::vector<point>& getnodes()
    {
        return nodes;
    }

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
    // creating paths between the nodes which are unders a certain radius for each node
    void random_path(std::vector<point> nodes, int max_dist)
    {
        static bool seeded = false;
        for (size_t i = 0; i < nodes.size(); ++i) 
        {
            for (size_t j = i + 1; j < nodes.size(); ++j)
            {
                float distance = dist(nodes[i],nodes[j]);
                if (distance<max_dist)
                {
                    bool exists = false;
                    for (auto &p : dist_map[i]) 
                    {
                        if (p.first == j) 
                        {   // neighbor already present
                            exists = true;
                            break;
                        }
                    }
                    if (!exists) 
                    {
                        dist_map[i].push_back({j,distance});
                        dist_map[j].push_back({i,distance});
                    }
                    DrawLine((nodes[i].x)*60,(nodes[i].y)*60,(nodes[j].x)*60,(nodes[j].y)*60,RAYWHITE);
                }
            }
        }
    }
    //creating the random nodes for the A* algorithm to work on. nodes are stored in the form of point structure
    std::vector<point> random_nodes()
    {
        srand(static_cast<unsigned int>(time(0)));
        while (nodes.size() < 70) {
            int x_cord = (rand() % 15)+1;
            int y_cord = (rand() % 15)+1;
            point new_point(x_cord, y_cord);
            if (std::find(nodes.begin(), nodes.end(), new_point) == nodes.end()) 
            {
                nodes.push_back(new_point);
            }
        }
        return nodes;
    }
    //heuristic distance stored in a map [end_node,each node]
    std::map<std::pair<int,int>, float> heuristic_distance(int end_node, std::vector<point> nodes)
    {
        for (int i = 0; i < nodes.size(); i++)
        {
           heuristic_dist_map[{end_node, i}] = dist(nodes[end_node],nodes[i]);
        }
        return heuristic_dist_map;
    }
    //displaying the nodes on the screen with one node blue signifying the end node for now
    void points_on_grid(std::vector<point> points, int start_node = -1, int end_node = -1)
    {
    int x_cord = -1;
    int y_cord = -1;
    for (size_t i = 0; i < nodes.size(); ++i) 
    {   
        std::string label = std::to_string(i);
        x_cord = nodes[i].x*60;
        y_cord = nodes[i].y*60;
        if (i == start_node) 
        {
            DrawCircle(x_cord, y_cord, 6, BLUE);  // The 5th node with blue color
            DrawText(label.c_str(),x_cord+8,y_cord-8,7,BLUE);
        } 
        else if (i == end_node) 
        {
            DrawCircle(x_cord, y_cord, 6, RED);
            DrawText(label.c_str(),x_cord+8,y_cord-8,7,RED);
        }
        else 
        {
            DrawCircle(x_cord, y_cord, 6, YELLOW);
            DrawText(label.c_str(),x_cord+8,y_cord-8,7,YELLOW);
        }
    }
    }
    //creating the grid on the screen
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
std::vector<float> g_node(70, std::numeric_limits<float>::infinity());       // distances from start
std::vector<int> camefrom(70, -1);          // parent of each node
std::vector<int> closed_list;              // nodes already explored
std::vector<std::pair<int,float>> priority_queue;
std::vector<int> finalpath;
std::map<std::pair<int,int>, std::vector<int>> history;
std::vector<int> neighbours;

int start_node = 0;
int end_node = 50;

bool A_star(int start_node, int end_node, std::vector<std::pair<int,float>> &priority_q, std::map<int,std::vector<std::pair<int,float>>> &dist_map,std::map<std::pair<int,int>, float> &heuristic_dist_map,std::vector<point> all_nodes)
{
    priority_q.push_back({start_node,0});
    bool no_path = false;
    g_node[start_node] = 0;
    int node = start_node;
    while( node!= end_node)
    {
        float cost_neighbour = -1.0f;
        int neighbour_node = -1.;
        int next_node = -1;
        float g_neighbour = -1.0f;
        float f_neighbour = -1.0f;
        float h_neighbour = -1.0f;
        for(int i = 0;i<dist_map[node].size();i++)
        {
            neighbour_node = dist_map[node][i].first;
            if (std::find(closed_list.begin(), closed_list.end(), neighbour_node) == closed_list.end())
            {
                neighbours.push_back(neighbour_node);
                cost_neighbour = dist_map[node][i].second;
                h_neighbour = heuristic_dist_map[{end_node,neighbour_node}];
                g_neighbour = g_node[node]+cost_neighbour;
                f_neighbour = g_neighbour+h_neighbour;
                if (g_neighbour<g_node[neighbour_node])
                {
                    g_node[neighbour_node] = g_neighbour;
                    priority_q.push_back({neighbour_node, f_neighbour});
                    camefrom[neighbour_node] = node;
                }
            }

        }
        if (!priority_q.empty())
        {
            std::sort(priority_q.begin(), priority_q.end(), [](const std::pair<int,float> &a, const std::pair<int,float> &b)
            {
                return a.second < b.second;
            });
            history[{node,priority_q[0].first}] = neighbours;
            neighbours.clear();
            node = priority_q[0].first;
            priority_q.erase(priority_q.begin());
        }
        else
        {
            no_path = true;
            return no_path;
        }
    }
    for (const auto& entry : history) {
    const std::pair<int, int>& key = entry.first;
    const std::vector<int>& values = entry.second;

    std::cout << "Key: (" << key.first << ", " << key.second << ") -> Values: ";
    for (int v : values) {
        std::cout << v << " ";
    }
    std::cout << std::endl;
    }
    finalpath.clear(); // Clear any previous path data
    node = end_node;
    while (true)
    {
        finalpath.push_back(node);
        node = camefrom[node];
        if(node == start_node)
        {
            return no_path;
        }
    }
}
int main()
{
    grid myGrid;
    std::vector<point> all_nodes = myGrid.random_nodes();
    int clickStage = 0; 
    int checker = 0;
    const int screenWidth = 960;
    const int screenHeight = 960;
    const int rows = 16;
    const int cols = 16;
    const int cellWidth = screenWidth / cols;
    const int cellHeight = screenHeight / rows;
    bool no_path = false;
    float timer = 0.0f;
    float interval = .5f;
    auto historyIterator = history.begin();

    InitWindow(screenWidth, screenHeight, "A*");
    SetTargetFPS(60);
    while (!WindowShouldClose() && no_path == false) {
        BeginDrawing();
        ClearBackground(BLACK);
        myGrid.on_screen_grid(cols, rows, cellWidth, cellHeight);  
        myGrid.random_path(all_nodes,3);
        myGrid.points_on_grid(all_nodes);
        if (checker == 0 && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) 
        {
            int mouseX = GetMouseX();
            int mouseY = GetMouseY();

            for (size_t i = 0; i < all_nodes.size(); i++) 
            {
                int screenX = all_nodes[i].x * 60;
                int screenY = all_nodes[i].y * 60;

                float dx = mouseX - screenX;
                float dy = mouseY - screenY;
                float dist = sqrt(dx * dx + dy * dy);

                if (dist <= 10) 
                {
                    if (clickStage == 0) {
                        start_node = i;
                        clickStage = 1;
                        std::cout << "Start node set: " << start_node << std::endl;
                        DrawCircle(all_nodes[start_node].x,all_nodes[start_node].y , 6, BLUE);
                    } else if (clickStage == 1) {
                        end_node = i;
                        clickStage = 2; // both chosen
                        std::cout << "End node set: " << end_node << std::endl;
                        myGrid.heuristic_distance(end_node, all_nodes);
                    }
                }
            }
            myGrid.points_on_grid(all_nodes,start_node, end_node);
        }
        if (checker == 0 && clickStage == 2 && no_path == false)
        {
            no_path = A_star(start_node,end_node,priority_queue,myGrid.getDistMap(),myGrid.getheuristicDistMap(),all_nodes);
            checker = 1;
            historyIterator = history.begin();
        }
        if (checker == 1)
        {
            timer += GetFrameTime();
            if (timer >= interval && historyIterator != history.end())
            {
                const std::pair<int, int>& key = historyIterator->first;
                const std::vector<int>& neighbors = historyIterator->second;
                for(auto& neighbour:neighbors)
                {
                    Vector2 start = { all_nodes[key.first].x * 60.0f, all_nodes[key.first].y * 60.0f };
                    Vector2 end = { all_nodes[neighbour].x * 60.0f, all_nodes[neighbour].y * 60.0f };

                    std::cout << "Drawing line from " << key.first << " to " << neighbour << std::endl;
                    DrawLineEx(start, end, 3.0f, YELLOW);                
                }
                if (timer>=5.0f)
                {
                    ++historyIterator;
                    timer = 0.0f;
                }

            }
            if (historyIterator == history.end())
            {
                if (no_path == false)
                {

                    for (auto& element : finalpath)
                    {
                        if(element!=start_node)
                        {
                            DrawLineEx({(all_nodes[element].x * 60.0f),( all_nodes[element].y * 60.0f)}, {all_nodes[camefrom[element]].x * 60.0f, all_nodes[camefrom[element]].y * 60.0f},2.5f , RED);   
                        }
                    }
                }
                else
                {
                    std::cout<<"there is no valid path"<<std::endl;
                }
                myGrid.points_on_grid(all_nodes,start_node, end_node);// Pass the vector of points
            }
        }   
        EndDrawing();
    }
    for (auto& element : finalpath)
    {
    }
    CloseWindow();
    return 0;
}