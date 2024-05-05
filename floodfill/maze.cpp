#include <array>
#include <string>
#include <iostream>
#include "API.h"
//added to kristen branch
void log(const std::string& text) {
    std::cerr << text << std::endl;
}
unsigned MAXCOST = 255;
struct Coord 
{
    int x;
    int y;
};


enum Direction 
{
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};


enum DirectionBitmask {
    NORTH_MASK = 0b1000,
    EAST_MASK  = 0b0100,
    SOUTH_MASK = 0b0010,
    WEST_MASK  = 0b0001
};


struct Cell 
{
    Coord pos;
    Direction dir;
    bool blocked;
};

struct CellList {
    int size;
    Cell *cells;
};
struct Maze 
{
    Coord mousePos{0,0};
    Direction mouseDir{NORTH};
    int distances[16][16];
    bool exploredCells[16][16];
    int cellWalls[16][16];
    bool verticalWalls[16][17];
    bool horizontalWalls[17][16];
    Coord * goalPos;
};
    //differentiate bt accessible and blocked cells
    bool _in_bounds(Maze *m, Coord c);
    CellList * getNeighborCells(Maze *m, Coord c);
    //returns best accessible cell for mouse to move to
    Cell getBestCell(Maze *m);

    //functions to step rotation, update API & mouseDir
    void cw_step(Maze *m);
    void ccw_step(Maze *m);

    // sets a certain cell pos as the target cell
    void setGoalCell(Maze *m);
    // only updates the simulator
    void updateSimulator(Maze m);
    void _reset_distances(Maze *m);
    //after having moved a step?, if called api move forward
    void updatePos(Maze *m);
    
    //same thing as update walls
    void scanWalls(Maze *m);
    Direction dir(Maze *m);

    //updates distances based on new scanned walls
    void floodfill(Maze *m);

bool _in_bounds(Maze *m, Coord c) 
{
    return c.x >=0 && c.x < 16 && c.y >=0 && c.y<16;
}

CellList *getNeighborCells(Maze *m, Coord c) {
    log("here now,");
    CellList* neighbors = (CellList*)malloc(sizeof(CellList));
    int i{0};
    // set cell list size
    //check 4 directions, increment size for each valid direction;
    //checking north direction;
    neighbors->size = 4;
    neighbors->cells = (Cell*)malloc(neighbors->size * sizeof(Cell));
    //checking cell directly above target coord;
    if (_in_bounds(m, Coord{c.x + 1, c.y})) {
        log("found north neghbor");
        neighbors->cells[0] = (Cell){(Coord){c.x+1, c.y}, NORTH, m->verticalWalls[c.x+1][c.y]};
    }
    if (_in_bounds(m, Coord{c.x , c.y+1})) {
        log("found east neighbor");
        neighbors->cells[1] = (Cell){(Coord){c.x, c.y+1}, NORTH, m->horizontalWalls[c.x][c.y+1]};
    }
    if (_in_bounds(m, Coord{c.x - 1, c.y})) {
        log("found south neghbor");
        neighbors->cells[2] = (Cell){(Coord){c.x-1, c.y}, NORTH, m->verticalWalls[c.x-1][c.y]};
    }
    if (_in_bounds(m, Coord{c.x, c.y -1})) {
        log("found west neghbor");
        neighbors->cells[3] = (Cell){(Coord){c.x, c.y-1}, NORTH, m->horizontalWalls[c.x][c.y-1]};
    }

    //cell anatomy
    /*
        Coord pos, Direction dir, bool blocked;
    */
   return neighbors;

}

Cell getBestCell(Maze *m) {
    //investigate distances (this is matrix of mannhatten distances to the goal cell)
    //check if wall is blocking cell
    int a{m->distances[m->mousePos.y][m->mousePos.x]};
    int b{m->distances[m->mousePos.y][m->mousePos.x]};
    int c{m->distances[m->mousePos.y][m->mousePos.x]};
    int d{m->distances[m->mousePos.y][m->mousePos.x]};
}

void cw_step(Maze *m) {
    API::turnRight();
    m->mouseDir = (Direction)((m->mouseDir +1)%4);
}

void ccw_step(Maze *m) {
    /*
    if (m->mouseDir == NORTH) {
        m->mouseDir == WEST;
    }
    else if (m->mouseDir == EAST) {
        m->mouseDir == NORTH;
    }
    else if (m->mouseDir == SOUTH) {
        m->mouseDir == EAST;
    }
    else if (m->mouseDir == WEST) {
        m->mouseDir == SOUTH;
    }
    */

    API::turnLeft();
    m->mouseDir = Direction((m->mouseDir+3)%4);
    std::cerr << "turned left , new direction is " << m->mouseDir << std::endl;
}

void setGoalCell(Maze *m) {
    m->distances[7][7] = 0;
    m->distances[7][8] = 0;
    m->distances[8][7] = 0;
    m->distances[8][8] = 0;
}

void updateSimulator(Maze m) {

   // updatePos(&m);
    for (int y = 0; y < 16; ++y) {
        for (int x = 0; x < 16; ++x) {
           // API::setText(x, y, std::to_string(m.distances[y][x]));
           // std::cerr << m.cellWalls[y][x] << " y is " << y << " x is " << x << std::endl;
            
            if (m.cellWalls[y][x] & NORTH_MASK) {
               // std::cerr << "found wall north " << NORTH_MASK << " "<< y<< ", " << x << std::endl;
                API::setWall(x, y, 'n');
            }

            if (m.cellWalls[y][x] & EAST_MASK) {
              //  std::cerr << "found wall east" << EAST_MASK << " " << y<< ", " << x << std::endl;

                API::setWall(x, y, 'e');
            }
            if (m.cellWalls[y][x] & SOUTH_MASK) {
               // std::cerr << "found wall south" << SOUTH_MASK << " " <<y<< ", " << x << std::endl;

                API::setWall(x, y, 's');
            }
            if (m.cellWalls[y][x] & WEST_MASK) {
              //  std::cerr << "found wall west" << WEST_MASK << " " <<y<< ", " << x << std::endl;

                API::setWall(x, y, 'w');
            }
            
            
        }
    }

}

void scanWalls(Maze *m) {
    //scan front
    
    if (m->mouseDir == NORTH) {
        if (API::wallFront()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= NORTH_MASK;
        if (API::wallRight()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= EAST_MASK;
        if (API::wallLeft()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= WEST_MASK;
    }
    else if (m->mouseDir == EAST) {
        if (API::wallFront()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= EAST_MASK;
        if (API::wallRight()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= SOUTH_MASK;
        if (API::wallLeft()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= NORTH_MASK;
    }
    else if (m->mouseDir == SOUTH) {
        if (API::wallFront()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= SOUTH_MASK;
        if (API::wallRight()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= WEST_MASK;
        if (API::wallLeft()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= EAST_MASK;
    }
    else if (m->mouseDir == WEST) {
        if (API::wallFront()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= WEST_MASK;
        if (API::wallRight()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= NORTH_MASK;
        if (API::wallLeft()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= SOUTH_MASK;
    }
   
/*
    if (API::wallFront()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= DirectionBitmask(m->mouseDir);
    if (API::wallRight()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= DirectionBitmask((m->mouseDir+1)%4);
    if (API::wallLeft()) m->cellWalls[m->mousePos.y][m->mousePos.x] |= DirectionBitmask((m->mouseDir+3)%4);

*/
}

void updatePos(Maze *m){
    API::moveForward();
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
    log("called dir");
    Direction d = m->mouseDir;
    if (d==NORTH) {
    std::cerr << d << std::endl;

    }
    return d;

}

void fill(Maze *m) {

}


void floodfill(Maze *m) {
    Coord queue[255];
    int h{0}, t{0};

    //resetting to max cost
    /*
    for (int y=0; y < 16; ++y) {
        for (int x=0; x<16; ++x) {
            //check if wall,
            m->distances[x][y] = abs(m->goalPos->x - m->mousePos.x) + abs(m->goalPos->y - m->mousePos.y);
        }
    }
    */
    //set goal cell distances to 0
    _reset_distances(m);
    setGoalCell(m);
    log("reset distances");
    //add goal cells to queue
    queue[t] = Coord{7,7};
    ++t;
    queue[t] = Coord{7,8};
    ++t;
    queue[t] = Coord{8,7};
    ++t;
    queue[t] = Coord{8,8};
    ++t;
    log("added goal");
//move incrementing head, after each iter of for loop, which checks one cell in the queue
    while(t - h > 0) {
        Coord cur_pos = queue[h];
        ++h;

        int newcost = m->distances[cur_pos.y][cur_pos.x] +1;
        std::cerr << "newcost is " << newcost << std::endl;
        CellList* neighbor = getNeighborCells(m, cur_pos);
        log("got neighbors");
        int n_size = neighbor->size;
        for (int i=0; i<n_size; ++i) {
            std::cerr << "iter " << i << std::endl;
            if (!neighbor->cells[i].blocked && m->distances[neighbor->cells[i].pos.y][neighbor->cells[i].pos.x] >newcost) {
                log("replacing neighbor with newcost");
                    m->distances[neighbor->cells[i].pos.y][neighbor->cells[i].pos.x] = newcost;
                    queue[t] = Coord{neighbor->cells[i].pos.y, neighbor->cells[i].pos.x};
                    ++t;
            }
            
        }
        free(neighbor->cells);
        free(neighbor);
    }
    
}

//ff helper functions
/* 
    1. reset all distances to MAXCOST
        ~ set each distance to base manhatten cost, regardless of walls

    2. set goal cells
        ~ 4x4 in center, or wherever in the maze, set newcost to 0
    
    3. get neighbors
        ~ fill queue with neighbor cells- a CellList of Cell structs.

    4. set the distance of the neighbor cells to the newcost 
        ~check if it has alr been incremented/updated by comparing it to MAXCOST
        ~ increment the value of root cell's distance by 1

    5. check neighbors
        ~ check if cells are blocked, determined by bitmask, if blocked, do not update
        ~ order matters, check in order of bitmask, NESW

    ~additional checks
        ~ add bitmask to all cells sharing a wall
*/

//reset distances

void _reset_distances(Maze *m) 
{
    log("called for resetting distance!");
    for (int i = 0; i < 16; ++i) {
        for (int j = 0; j < 16; ++j) 
            m->distances[i][j] = MAXCOST;
    }
}



void init_cellwalls(Maze *m) 
{
    for (int i=0; i<16; ++i) {
        for (int j=0; j<16; ++j) {
            m->cellWalls[j][i] = 0;
        }
    }
}




//k
//initial values

int main(int argc, char* argv[]) {
    Maze m;
    log("Running...");
        std::cerr << "RUNNNING" << std::endl;

    std::cerr << NORTH << std::endl;
    API::setColor(0, 0, 'G');
    API::setText(0, 0, "abc");
    std::cerr << m.mousePos.x << std::endl;
    API::setWall(0,0,'w');
    API::setWall(0,0, 's');

    init_cellwalls(&m);

    while(true) {
   
    floodfill(&m);
    _reset_distances(&m);
    scanWalls(&m);
    updateSimulator(m);

    if (!API::wallLeft()) {
        ccw_step(&m);
    }
    while (API::wallFront())
        cw_step(&m);

    
    //move(&m);
    updatePos(&m);
    
    }
    
    //scan walls and calculate distance
    /*
    while (true) {
        log("inside while");
        Direction d = dir(&m);
        std::cerr << d << "(" << 0 << ", " << 0 << ")" << std::endl;
        //std::cerr << dir_chars[dir] << std::endl;
        log("before floodfill");
        floodfill(&m);
        log("post floodfill, shouldve updated");
        updateSimulator(m);
        scanWalls(&m);
        if (!API::wallLeft()) {
            
            ccw_step(&m);
            
        }
        while (API::wallFront()) {
            cw_step(&m);
            d = Direction((d + 1)%4);  

        }
        move(&m);
    
    }
    */
}