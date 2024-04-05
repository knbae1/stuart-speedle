#include "maze.h"
#include <array>


Cell* getNeighborCells(Coord c) {


}

Cell getBestCell() {
    //investigate distances (this is matrix of mannhatten distances to the goal cell)
    //check if wall is blocking cell
    int a{distances[mousePos.y][mousePos.x]};
    int a{distances[mousePos.y][mousePos.x]};
    int a{distances[mousePos.y][mousePos.x]};
    int a{distances[mousePos.y][mousePos.x]};
}

Direction cw_step() {
    if (mouseDir == 3) {
			mouseDir = NORTH;
			
	} else mouseDir = (Direction)((mouseDir + 3) % 4);
    API::turnRight();
    return mouseDir;
	
		
}

Direction ccw_step() {
    if (mouseDir==0) {
			mouseDir = WEST;
		}	else mouseDir = (Direction)((mouseDir + 1) % 4);
    API::turnLeft();
	return mouseDir;
}

void setGoalCell() {

}

void rotate() {

}

void move() {
    API::moveForward();
    updatePos();
}

void updateSimulator() {
    scanWalls();
    for (int y = 0; y < 16; ++y) {
        for (int x = 0; x < 16; ++x) {
            API::setText(x, y, std::to_string(distances[y][x]));
            if (cellWalls[y][x] & NORTH_MASK) {
                API::setWall(x, y, 'n');
            }

            if (cellWalls[y][x] & EAST_MASK) {
                API::setWall(x, y, 'e');
            }
            if (cellWalls[y][x] & SOUTH_MASK) {
                API::setWall(x, y, 's');
            }
            if (cellWalls[y][x] & WEST_MASK) {
                API::setWall(x, y, 'w');
            }
            
        }
    }
}

void scanWalls() {
    if (API::wallFront()) {
        cellWalls[mousePos.y][mousePos.x] |= mouseDir;
    }
    if (API::wallRight()) {
        cellWalls[mousePos.y][mousePos.x] |= ((mouseDir+1)%4);
    }
    if (API::wallLeft()) {
        cellWalls[mousePos.y][mousePos.x] |= ((mouseDir+3)%4);
    }
}

void updatePos(){
    if (mouseDir == NORTH)
            mousePos.y++;
        if (mouseDir == SOUTH)
            mousePos.y--;
        if (mouseDir == WEST)
            mousePos.x--;
        if (mouseDir == EAST)
            mousePos.x++;
    std::cerr << "inside function: (" << mousePos.x << ", " << mousePos.y << ")" << std::endl;

}

Direction dir() const {
    return mouseDir;
}


void floodfill() {
    Coord queue[255];
    int h{0}, t{0};

    //resetting to max cost
    for (int y=0; y < 16; ++y) {
        for (int x=0; x<16; ++x) {
            //check if wall,
            distances[x][y] = abs(goalPos->x - mousePos.x) + abs(goalPos->y - mousePos.y);
        }
    }
    //set goal cell distances to 0
    distances[7][7] = 0;
    distances[7][8] = 0;
    distances[8][7] = 0;
    distances[8][8] = 0;

    queue[0] = Coord{7,7};
    queue[1] = Coord{7,8};
    queue[2] = Coord{8,7};
    queue[3] = Coord{8,8};
    t = 4;

    while(t - h > 0) {
        Coord cur_pos = queue[h];
        ++h;
        int newcost = distances[cur_pos.y][cur_pos.x] +1;
        Cell *neighborCells = getNeighborCells(cur_pos);
        for (Cell c : *neighborCells) {
            if (!c.blocked) {
                if(distances[cell.y][cell.x]>newcost) {
                    distances
                }
            }
        }
    }

}