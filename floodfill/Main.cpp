#include <iostream>
#include <string>

#include "API.h"

void log(const std::string& text) {
    std::cerr << text << std::endl;
}

enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

char dir_chars[4] = {'n', 'e', 's', 'w'};

//initial values
int x =0;
int y = 0;
Direction dir = NORTH;

int main(int argc, char* argv[]) {
    
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
            API::turnLeft();
            dir = Direction((dir + 3)%4);  
        }
        while (API::wallFront()) {
            API::turnRight();
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
