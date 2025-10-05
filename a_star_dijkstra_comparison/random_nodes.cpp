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
struct point{
    int x, y;
    point(int x, int y): x(x),y(y) {}

    bool operator==(const point& other) const {
        return (x == other.x && y == other.y);
    }
};
int main()
{
    std::vector<point> nodes;
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
    for (auto& element:nodes)
    {
        std::cout<< element.x<<","<<element.y<< std::endl;
    }
}
