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
    void points_on_grid(std::vector<point> nodes, int start_node = -1, int end_node = -1)
    {
    int x_cord = -1;
    int y_cord = -1;
    //std::cout<<nodes.size()<<std::endl;
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
std::vector<float> dijkstra_g_node(70, std::numeric_limits<float>::infinity());       // distances from start
std::vector<int> dijkstra_camefrom(70, -1);          // parent of each node
std::vector<int> dijkstra_closed_list;              // nodes already explored
std::vector<std::pair<int,float>> dijkstra_priority_queue;
std::vector<int> dijkstra_finalpath;

std::vector<float> A_star_g_node(70, std::numeric_limits<float>::infinity()); 
std::vector<int> A_star_camefrom(70, -1);          // parent of each node
std::vector<int> A_star_closed_list;              // nodes already explored
std::vector<std::pair<int,float>> A_star_priority_queue;
std::vector<int> A_star_finalpath;

int start_node;
int end_node;


void dijkstra_step(int node,int end_node, std::vector<std::pair<int,float>> &priority_q, std::map<int,std::vector<std::pair<int,float>>> &dist_map,std::vector<point> all_nodes)
{
        float cost_neighbour = -1.0f;
        int neighbour_node = -1.;
        int next_node = -1;
        float g_neighbour = -1.0f;
        for(int i = 0;i<dist_map[node].size();i++)
        {
            neighbour_node = dist_map[node][i].first;
            if (std::find(dijkstra_closed_list.begin(), dijkstra_closed_list.end(), neighbour_node) == dijkstra_closed_list.end())
            {
                cost_neighbour = dist_map[node][i].second;
                g_neighbour = dijkstra_g_node[node]+cost_neighbour;
                if (g_neighbour<dijkstra_g_node[neighbour_node])
                {
                    dijkstra_g_node[neighbour_node] = g_neighbour;
                    priority_q.push_back({neighbour_node, g_neighbour});
                    dijkstra_camefrom[neighbour_node] = node;
                }
            }

        }

    /*for (auto& element:camefrom)
    {
        std::cout<<element<<std::endl;
    }*/

}

void A_star_step(int node,int end_node, std::vector<std::pair<int,float>> &priority_q, std::map<int,std::vector<std::pair<int,float>>> &dist_map,std::map<std::pair<int,int>, float> &heuristic_dist_map,std::vector<point> all_nodes)
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
            if (std::find(A_star_closed_list.begin(), A_star_closed_list.end(), neighbour_node) == A_star_closed_list.end())
            {
                cost_neighbour = dist_map[node][i].second;
                h_neighbour = heuristic_dist_map[{end_node,neighbour_node}];
                g_neighbour = A_star_g_node[node]+cost_neighbour;
                f_neighbour = g_neighbour+h_neighbour;
                if (g_neighbour<A_star_g_node[neighbour_node])
                {
                    A_star_g_node[neighbour_node] = g_neighbour;
                    priority_q.push_back({neighbour_node, f_neighbour});
                    A_star_camefrom[neighbour_node] = node;
                }
            }

        }

    /*for (auto& element:camefrom)
    {
        std::cout<<element<<std::endl;
    }*/

}


void dijkstra_path_visualisation(std::vector<point> all_nodes)
{
    int node = -1;
    grid myGrid;
    int clickStage = 0; 
    int checker = 0;
    const int screenWidth = 960;
    const int screenHeight = 960;
    const int rows = 16;
    const int cols = 16;
    const int cellWidth = screenWidth / cols;
    const int cellHeight = screenHeight / rows;
    bool no_path = false;

    static int currentGreenLine = 0;  // index of last revealed green line
    static float lastUpdateTime = 0.0f;
    const float animationDelay = 0.2f;  // seconds between each line

    std::vector<float> lineProgress(dijkstra_closed_list.size(), 0.0f);                  // Animation progress (0.0f to 1.0f)
    float speed = 1.0f; 

    bool showGreenLines = false;

    InitWindow(screenWidth, screenHeight, "dijkstra");
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

            dijkstra_g_node.assign(all_nodes.size(), std::numeric_limits<float>::infinity());
            dijkstra_camefrom.assign(all_nodes.size(), -1);
            dijkstra_priority_queue.clear();
            dijkstra_closed_list.clear();
            dijkstra_finalpath.clear();
            myGrid.points_on_grid(all_nodes,start_node, end_node);
            dijkstra_priority_queue.push_back({start_node,0});
            dijkstra_g_node[start_node] = 0;
            int node = start_node;
        }
        if (checker == 0 && clickStage == 2 && no_path == false)
        {
            while (!dijkstra_priority_queue.empty())
                {
                    std::sort(dijkstra_priority_queue.begin(), dijkstra_priority_queue.end(),
                        [](const std::pair<int,float>& a, const std::pair<int,float>& b) {
                            return a.second < b.second;
                        });
                    node = dijkstra_priority_queue[0].first;
                    /*for (auto element:dijkstra_priority_queue)
                    {
                        std::cout<<element.first<<std::endl;
                    }*/
                    dijkstra_priority_queue.erase(dijkstra_priority_queue.begin());
                    if (node == end_node) {
                        break;
                    }

                    dijkstra_closed_list.push_back(node);
                    dijkstra_step(node, end_node, dijkstra_priority_queue,
                                myGrid.getDistMap(),
                                all_nodes);
                }
            if (node != end_node) {
                no_path = true;
            }
            if (no_path == false)
            {
                dijkstra_finalpath.clear(); 
                node = end_node;
                while (node != start_node && node != -1)
                {
                    dijkstra_finalpath.push_back(node);
                    node = dijkstra_camefrom[node];
                }

                if (node == -1)
                {
                    no_path = true;  
                }
                else{
                    // Ready to animate green lines after path found
                    showGreenLines = true;
                    currentGreenLine = 0;
                    lastUpdateTime = GetTime();
                    lineProgress.resize(dijkstra_closed_list.size(), 0.0f);
                }
            }
            checker = 1;
        }
        if (checker == 1)
        {
            if (no_path == false)
            {// animation(chatgpt)
                if (showGreenLines && currentGreenLine < dijkstra_closed_list.size()) {
                    float now = GetTime();
                    if (now - lastUpdateTime >= animationDelay) {
                        currentGreenLine++;
                        lastUpdateTime = now;
                    }
                }

                // Draw only the lines up to currentGreenLine for animation effect
                for (int i = 0; i < currentGreenLine; ++i) {
                    int element = dijkstra_closed_list[i];
                    for (auto& neighbour : myGrid.getDistMap()[element]) {
                        Vector2 startPoint = { all_nodes[element].x * 60, all_nodes[element].y * 60 };
                        Vector2 endPoint = { all_nodes[neighbour.first].x * 60, all_nodes[neighbour.first].y * 60 };
                    if (lineProgress[i] < 1.0f) {
                        lineProgress[i] += speed * GetFrameTime();
                        if (lineProgress[i] > 1.0f) lineProgress[i] = 1.0f;
                    }

                    Vector2 currentPoint = {
                        startPoint.x + lineProgress[i] * (endPoint.x - startPoint.x),
                        startPoint.y + lineProgress[i] * (endPoint.y - startPoint.y)
                    };
                        DrawLineEx(startPoint, currentPoint,2.0f,GREEN);
                    }
                }
                if (currentGreenLine == dijkstra_closed_list.size())
                {
                    for (auto& element : dijkstra_finalpath)
                    {
                        if(element!=start_node)
                        {
                            DrawLineEx({(all_nodes[element].x * 60.0f),( all_nodes[element].y * 60.0f)}, {all_nodes[dijkstra_camefrom[element]].x * 60.0f, all_nodes[dijkstra_camefrom[element]].y * 60.0f},3.0f , RED);   
                        }
                    }
                }
            }
            else
            {
                std::cout<<"there is no valid path"<<std::endl;
            }
            myGrid.points_on_grid(all_nodes,start_node, end_node);// Pass the vector of points
        }   
        EndDrawing();
    }
    for (auto& element : dijkstra_finalpath)
    {
    }
    CloseWindow();
}
void A_star_path_visualisation(std::vector<point> all_nodes)
{
    int node = -1;
    grid myGrid;
    int clickStage = 0; 
    int checker = 0;
    const int screenWidth = 960;
    const int screenHeight = 960;
    const int rows = 16;
    const int cols = 16;
    const int cellWidth = screenWidth / cols;
    const int cellHeight = screenHeight / rows;
    bool no_path = false;

    static int currentGreenLine = 0;  // index of last revealed green line
    static float lastUpdateTime = 0.0f;
    const float animationDelay = 0.2f;  // seconds between each line

    std::vector<float> lineProgress(A_star_closed_list.size(), 0.0f);                  // Animation progress (0.0f to 1.0f)
    float speed = 1.0f; 

    bool showGreenLines = false;

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

            A_star_g_node.assign(all_nodes.size(), std::numeric_limits<float>::infinity());
            A_star_camefrom.assign(all_nodes.size(), -1);
            A_star_priority_queue.clear();
            A_star_closed_list.clear();
            A_star_finalpath.clear();
            myGrid.points_on_grid(all_nodes,start_node, end_node);
            A_star_priority_queue.push_back({start_node,0});
            A_star_g_node[start_node] = 0;
            int node = start_node;
        }
        if (checker == 0 && clickStage == 2 && no_path == false)
        {
            while (!A_star_priority_queue.empty())
                {
                    std::sort(A_star_priority_queue.begin(), A_star_priority_queue.end(),
                        [](const std::pair<int,float>& a, const std::pair<int,float>& b) {
                            return a.second < b.second;
                        });
                    node = A_star_priority_queue[0].first;
                    /*for (auto element:A_star_priority_queue)
                    {
                        std::cout<<element.first<<std::endl;
                    }*/
                    A_star_priority_queue.erase(A_star_priority_queue.begin());
                    if (node == end_node) {
                        break;
                    }

                    A_star_closed_list.push_back(node);
                    A_star_step(node, end_node, A_star_priority_queue,
                                myGrid.getDistMap(), myGrid.getheuristicDistMap(),
                                all_nodes);
                }
            if (node != end_node) {
                no_path = true;
            }
            if (no_path == false)
            {
                A_star_finalpath.clear(); 
                node = end_node;
                while (node != start_node && node != -1)
                {
                    A_star_finalpath.push_back(node);
                    node = A_star_camefrom[node];
                }

                if (node == -1)
                {
                    no_path = true;  
                }
                else{
                    // Ready to animate green lines after path found
                    showGreenLines = true;
                    currentGreenLine = 0;
                    lastUpdateTime = GetTime();
                    lineProgress.resize(A_star_closed_list.size(), 0.0f);
                }
            }
            checker = 1;
        }
        if (checker == 1)
        {
            if (no_path == false)
            {// animation(chatgpt)
                if (showGreenLines && currentGreenLine < A_star_closed_list.size()) {
                    float now = GetTime();
                    if (now - lastUpdateTime >= animationDelay) {
                        currentGreenLine++;
                        lastUpdateTime = now;
                    }
                }

                // Draw only the lines up to currentGreenLine for animation effect
                for (int i = 0; i < currentGreenLine; ++i) {
                    int element = A_star_closed_list[i];
                    for (auto& neighbour : myGrid.getDistMap()[element]) {
                        Vector2 startPoint = { all_nodes[element].x * 60, all_nodes[element].y * 60 };
                        Vector2 endPoint = { all_nodes[neighbour.first].x * 60, all_nodes[neighbour.first].y * 60 };
                    if (lineProgress[i] < 1.0f) {
                        lineProgress[i] += speed * GetFrameTime();
                        if (lineProgress[i] > 1.0f) lineProgress[i] = 1.0f;
                    }

                    Vector2 currentPoint = {
                        startPoint.x + lineProgress[i] * (endPoint.x - startPoint.x),
                        startPoint.y + lineProgress[i] * (endPoint.y - startPoint.y)
                    };
                        DrawLineEx(startPoint, currentPoint,2.0f,GREEN);
                    }
                }
                if (currentGreenLine == A_star_closed_list.size())
                {
                    for (auto& element : A_star_finalpath)
                    {
                        if(element!=start_node)
                        {
                            DrawLineEx({(all_nodes[element].x * 60.0f),( all_nodes[element].y * 60.0f)}, {all_nodes[A_star_camefrom[element]].x * 60.0f, all_nodes[A_star_camefrom[element]].y * 60.0f},3.0f , RED);   
                        }
                    }
                }
            }
            else
            {
                std::cout<<"there is no valid path"<<std::endl;
            }
            myGrid.points_on_grid(all_nodes,start_node, end_node);// Pass the vector of points
        }   
        EndDrawing();
    }
    CloseWindow();
}

int main() {
    grid myGrid;
    std::vector<point> all_nodes = myGrid.random_nodes();

    // Run Dijkstra visualization
    dijkstra_path_visualisation(all_nodes);

    // Run A* visualization after closing first window
    A_star_path_visualisation(all_nodes);

    return 0;
}

