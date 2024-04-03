#include "maze.h"



Cell* Maze::getNeighborCells() {


}

Cell Maze::getBestCell() {
    //investigate distances (this is matrix of mannhatten distances to the goal cell)
    int a,b,c,d
}

Direction Maze::cw_step() {
    if (mouseDir == 3) {
			mouseDir = NORTH;
			
	} else mouseDir = (Direction)((mouseDir + 3) % 4);
    API::turnRight();
    return mouseDir;
	
		
}

Direction Maze::ccw_step() {
    if (mouseDir==0) {
			mouseDir = WEST;
		}	else mouseDir = (Direction)((mouseDir + 1) % 4);
    API::turnLeft();
	return mouseDir;
}

void Maze::setGoalCell() {

}

void Maze::rotate() {

}

void Maze::move() {

}

void Maze::updateSimulator() {
    Maze::scanWalls();
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

void Maze::scanWalls() {
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

void Maze::update_mousePos(){

}