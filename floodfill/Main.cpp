#include <iostream>
#include <string>

#include "API.h"
#include "maze.h"

void log(const std::string& text) {
    std::cerr << text << std::endl;
}

enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};


//initial values
int x =0;
int y = 0;
Direction dir = NORTH;

int main(int argc, char* argv[]) {
    Maze m;
    //log("Running...");
    std::cerr << NORTH << std::endl;
    API::setColor(0, 0, 'G');
    API::setText(0, 0, "abc");

    API::setWall(0,0,'w');
    API::setWall(0,0, 's');
    while (true) {
        std::cerr << dir << "(" << x << ", " << y << ")" << std::endl;
        //std::cerr << dir_chars[dir] << std::endl;
        if (!API::wallLeft()) {
            
            m.ccw_step();
        }
        while (API::wallFront()) {
            m.cw_step();
            dir = Direction((dir + 1)%4);  

        }
        API::moveForward();

        if (dir == NORTH) y++;
        
        if (dir == SOUTH) y--;
        if (dir == WEST) x--;
        if (dir == EAST) x++;

        if (API::wallFront()) 
        {
            API::setWall(x, y, dir_chars[dir]);
        }
        if (API::wallRight()) 
        {
            API::setWall(x, y, dir_chars[(dir+1)%4]);
        }
        if (API::wallLeft()) 
        {
            API::setWall(x, y, dir_chars[(dir+3)%4]);
        }
    }
}
