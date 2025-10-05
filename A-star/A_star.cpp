#include <iostream>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <climits>
#include <string> 
#include <algorithm> 

struct point{
    int x, y;
    point(int x, int y): x(x),y(y) {}
};

class grid{
    private:
    std::vector<point> points;

    public:
    point cordinate_system_generation()
    {
        for(int x = 0;x<15;x++){
            for (int y = 0; y<15;y++){
                points.push_back(point(x,y));
            }
        }
        return points;
    }

    int on_screen_grid()
    {

    }
}