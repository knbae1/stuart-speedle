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

int main(int argc, char* argv[]) {
    Maze m;
    //log("Running...");
    std::cerr << NORTH << std::endl;
    API::setColor(0, 0, 'G');
    API::setText(0, 0, "abc");

    API::setWall(0,0,'w');
    API::setWall(0,0, 's');
    //scan walls and calculate distance
    while (true) {
        Direction d = m.dir();
        std::cerr << d << "(" << 0 << ", " << 0 << ")" << std::endl;
        //std::cerr << dir_chars[dir] << std::endl;
        m.scanWalls();
        if (!API::wallLeft()) {
            
            m.ccw_step();
        }
        while (API::wallFront()) {
            m.cw_step();
            d = Direction((d + 1)%4);  

        }
        m.move();
       

       
    }
}
