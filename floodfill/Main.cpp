#include <iostream>
#include <string>

#include "API.h"
#include "maze.h"

void log(const std::string& text) {
    std::cerr << text << std::endl;
}





//k
//initial values

int main(int argc, char* argv[]) {
    Maze *m;
    log("Running...");
    std::cerr << NORTH << std::endl;
    API::setColor(0, 0, 'G');
    API::setText(0, 0, "abc");

    API::setWall(0,0,'w');
    API::setWall(0,0, 's');
    //scan walls and calculate distance
    while (true) {
        Direction d = dir(m);
        std::cerr << d << "(" << 0 << ", " << 0 << ")" << std::endl;
        //std::cerr << dir_chars[dir] << std::endl;
        std::cout << "Hello" << std::endl;
        floodfill(m);
        scanWalls(m);
        if (!API::wallLeft()) {
            
            ccw_step(m);
        }
        while (API::wallFront()) {
            cw_step(m);
            d = Direction((d + 1)%4);  

        }
        move(m);
       

       
    }
}
