#include "maze.h"
#include <array>
//added to kristen branch


CellList* getNeighborCells(Maze *m, Coord c) {
    CellList neighbors;



}

Cell getBestCell(Maze *m) {
    //investigate distances (this is matrix of mannhatten distances to the goal cell)
    //check if wall is blocking cell
    int a{m->distances[m->mousePos.y][m->mousePos.x]};
    int a{m->distances[m->mousePos.y][m->mousePos.x]};
    int a{m->distances[m->mousePos.y][m->mousePos.x]};
    int a{m->distances[m->mousePos.y][m->mousePos.x]};
}

Direction cw_step(Maze *m) {
    if (m->mouseDir == 3) {
			m->mouseDir = NORTH;
			
	} else m->mouseDir = (Direction)((m->mouseDir + 3) % 4);
    API::turnRight();
    return m->mouseDir;
	
		
}

Direction ccw_step(Maze *m) {
    if (m->mouseDir==0) {
			m->mouseDir = WEST;
		}	else m->mouseDir = (Direction)((m->mouseDir + 1) % 4);
    API::turnLeft();
	return m->mouseDir;
}

void setGoalCell(Maze *m) {

}

void rotate(Maze *m) {

}

void move(Maze *m) {
    API::moveForward();
    updatePos();
}

void updateSimulator(Maze *m) {
    scanWalls(m);
    updatePos(m);
    for (int y = 0; y < 16; ++y) {
        for (int x = 0; x < 16; ++x) {
            API::setText(x, y, std::to_string(m->distances[y][x]));
            if (m->cellWalls[y][x] & NORTH_MASK) {
                API::setWall(x, y, 'n');
            }

            if (m->cellWalls[y][x] & EAST_MASK) {
                API::setWall(x, y, 'e');
            }
            if (m->cellWalls[y][x] & SOUTH_MASK) {
                API::setWall(x, y, 's');
            }
            if (m->cellWalls[y][x] & WEST_MASK) {
                API::setWall(x, y, 'w');
            }
            
        }
    }

}

void scanWalls(Maze *m) {
    if (API::wallFront()) {
        m->cellWalls[m->mousePos.y][m->mousePos.x] |= m->mouseDir;
    }
    if (API::wallRight()) {
        m->cellWalls[m->mousePos.y][m->mousePos.x] |= ((m->mouseDir+1)%4);
    }
    if (API::wallLeft()) {
        m->cellWalls[m->mousePos.y][m->mousePos.x] |= ((m->mouseDir+3)%4);
    }
}

void updatePos(Maze *m){
    if (m->mouseDir == NORTH)
            m->mousePos.y++;
        if (m->mouseDir == SOUTH)
            m->mousePos.y--;
        if (m->mouseDir == WEST)
            m->mousePos.x--;
        if (m->mouseDir == EAST)
            m->mousePos.x++;
    std::cerr << "inside function: (" << m->mousePos.x << ", " << m->mousePos.y << ")" << std::endl;

}

Direction dir(Maze *m){
    return m->mouseDir;
}


void floodfill(Maze *m) {
    Coord queue[255];
    int h{0}, t{0};

    //resetting to max cost
    for (int y=0; y < 16; ++y) {
        for (int x=0; x<16; ++x) {
            //check if wall,
            m->distances[x][y] = abs(m->goalPos->x - m->mousePos.x) + abs(m->goalPos->y - m->mousePos.y);
        }
    }
    //set goal cell distances to 0
    m->distances[7][7] = 0;
    m->distances[7][8] = 0;
    m->distances[8][7] = 0;
    m->distances[8][8] = 0;

    queue[0] = Coord{7,7};
    queue[1] = Coord{7,8};
    queue[2] = Coord{8,7};
    queue[3] = Coord{8,8};
    t = 4;

    while(t - h > 0) {
        Coord cur_pos = queue[h];
        ++h;
        int newcost = m->distances[cur_pos.y][cur_pos.x] +1;
        CellList *neighborCells = getNeighborCells(m, cur_pos);
        for (int i=0; i<neighborCells->size; ++i) {
            if (!neighborCells->cells[i].blocked) {
                if(m->distances[neighborCells->cells[i].pos.x][neighborCells->cells[i].pos.y]>newcost) {
                    m->distances[neighborCells->cells[i].pos.x][neighborCells->cells[i].pos.y] = newcost;
                    queue[t] = Coord{neighborCells->cells[i].pos.x, neighborCells->cells[i].pos.y};
                    ++t;
                }
            }
        }
    }

}