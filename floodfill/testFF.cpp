#include <array>
#include <string>
#include <iostream>
#include "API.h"
#include <windows.h>
// added to kristen branch
void log(const std::string &text)
{
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

enum DirectionBitmask
{
    NORTH_MASK = 0b1000,
    EAST_MASK = 0b0100,
    SOUTH_MASK = 0b0010,
    WEST_MASK = 0b0001
};

struct Cell
{
    Coord pos;
    Direction dir;
    bool blocked;
};

struct CellList
{
    int size;
    Cell *cells;
};

struct Maze
{
    Coord mousePos{0, 0};
    Direction mouseDir{NORTH};

    int distances[16][16];
    bool exploredCells[16][16];
    int cellWalls[16][16];
    bool verticalWalls[16][17];
    bool horizontalWalls[17][16];

    Coord *goalPos;

    bool *returning;
};

bool _in_bounds(Coord c);
CellList *getNeighborCells(Maze *m, Coord c);
Cell getBestCell(Maze *m);

void cw_step(Maze *m);
void ccw_step(Maze *m);

void setGoalCell(Maze *m);

void updateSimulator(Maze m);
void _reset_distances(Maze *m);
void updatePos(Maze *m);

void scanWalls(Maze *m);
Direction dir(Maze *m);

void floodfill(Maze *m);

// OPTIMIZE
void init_walls(Maze *m)
{
    for (int i = 0; i < 16; ++i)
    {
        for (int j = 0; j < 16; ++j)
        {
            m->cellWalls[j][i] = 0;
        }
    }
    for (int y = 0; y < 17; ++y)
    {
        for (int x = 0; x < 16; ++x)
        {
            m->verticalWalls[x][y] = false;
            m->horizontalWalls[y][x] = false;
            if (x == 0 && y != 16)
            {
                m->horizontalWalls[x][y] = true;
                m->verticalWalls[y][x] = true;
                API::setWall(x, y, 'w');
                API::setWall(y, x, 's');
            }
            else if (x == 15 && y != 16)
            {
                m->horizontalWalls[x+1][y] = true;
                m->verticalWalls[y][x+1] = true;
                API::setWall(y, x, 'n');
                API::setWall(x, y, 'e');
            }
        }
    }
}


bool _in_bounds(Coord c)
{
    return c.x >= 0 && c.x < 16 && c.y >= 0 && c.y < 16;
}

CellList *getNeighborCells(Maze *m, Coord c)
{
    CellList *neighbors = (CellList *)malloc(sizeof(CellList));
    int i{0};
    neighbors->size = 4;
    neighbors->cells = (Cell *)malloc(neighbors->size * sizeof(Cell));

    neighbors->cells[0] = (Cell){(Coord){c.x + 1, c.y}, EAST, m->horizontalWalls[c.x + 1][c.y] || !_in_bounds((Coord){c.x + 1, c.y})};
    neighbors->cells[1] = (Cell){(Coord){c.x, c.y + 1}, NORTH, m->verticalWalls[c.x][c.y + 1] || !_in_bounds((Coord){c.x, c.y + 1})};
    neighbors->cells[2] = (Cell){(Coord){c.x - 1, c.y}, WEST, m->horizontalWalls[c.x][c.y] || !_in_bounds((Coord){c.x - 1, c.y})};
    neighbors->cells[3] = (Cell){(Coord){c.x, c.y - 1}, SOUTH, m->verticalWalls[c.x][c.y] || !_in_bounds((Coord){c.x, c.y - 1})};

    return neighbors;
}

Cell getBestKnownCell(Maze *m)
{
    int mx = m->mousePos.x;
    int my = m->mousePos.y;
    int shortest = m->distances[mx][my];
    CellList *best_cells = getNeighborCells(m, m->mousePos);
    int b_size = best_cells->size;
    Cell best{(Coord){255, 255}, NORTH, true};
    // first choice, explored and less than curr distance, cannot call on first run
    for (int i = 0; i < b_size; ++i)
    {
        int bx = best_cells->cells[i].pos.x;
        int by = best_cells->cells[i].pos.y;
        if (!best_cells->cells[i].blocked && ((m->exploredCells[bx][by] && m->distances[bx][by] <= shortest)))
        {
            best = best_cells->cells[i];
            shortest = m->distances[bx][by];
        }
    }
    free(best_cells->cells);
    free(best_cells);
    return best;
}

Cell getBestCell(Maze *m)
{
    int mx = m->mousePos.x;
    int my = m->mousePos.y;
    int shortest = m->distances[mx][my];
    CellList *best_cells = getNeighborCells(m, m->mousePos);
    int b_size = best_cells->size;
    Cell best{(Coord){255, 255}, NORTH, true};
    // first choice, unexplored and less than curr distance
    for (int i = 0; i < b_size; ++i)
    {
        int bx = best_cells->cells[i].pos.x;
        int by = best_cells->cells[i].pos.y;
        if (!best_cells->cells[i].blocked && ((!m->exploredCells[bx][by] && m->distances[bx][by] < shortest)))
        {
            best = best_cells->cells[i];
            shortest = m->distances[bx][by];
        }
    }

    // second choice, unexplored, less than or equal to
    for (int i = 0; i < b_size; ++i)
    {
        int bx = best_cells->cells[i].pos.x;
        int by = best_cells->cells[i].pos.y;
        if (!best_cells->cells[i].blocked && ((!m->exploredCells[bx][by] && m->distances[bx][by] <= shortest)))
        {
            best = best_cells->cells[i];
            shortest = m->distances[bx][by];
        }
    }
    //2.5 choice unexplored
    if (best.blocked)
    {
        for (int i = 0; i < b_size; ++i)
        {

            int bx = best_cells->cells[i].pos.x;
            int by = best_cells->cells[i].pos.y;
            if (!best_cells->cells[i].blocked && !m->exploredCells[bx][by])
            {
                best = best_cells->cells[i];
                shortest = m->distances[bx][by];
            }
        }
    }
    // third choice, less than curr distance
    if (best.blocked)
    {
        for (int i = 0; i < b_size; ++i)
        {

            int bx = best_cells->cells[i].pos.x;
            int by = best_cells->cells[i].pos.y;
            if (!best_cells->cells[i].blocked && m->distances[bx][by] < shortest)
            {
                best = best_cells->cells[i];
                shortest = m->distances[bx][by];
            }
        }
    }
    // last choice, less than or equal to curr distance and explored
    if (best.blocked)
    {
        for (int i = 0; i < b_size; ++i)
        {

            int bx = best_cells->cells[i].pos.x;
            int by = best_cells->cells[i].pos.y;
            if (!best_cells->cells[i].blocked && m->distances[bx][by] <= shortest)
            {
                best = best_cells->cells[i];
                shortest = m->distances[bx][by];
            }
        }
    }
    if (best.blocked)
    {
        for (int i = 0; i < b_size; ++i)
        {

            int bx = best_cells->cells[i].pos.x;
            int by = best_cells->cells[i].pos.y;
            if (!best_cells->cells[i].blocked)
            {
                best = best_cells->cells[i];
                shortest = m->distances[bx][by];
            }
        }
    }
    free(best_cells->cells);
    free(best_cells);
    return best;
}

void cw_step(Maze *m)
{
    API::turnRight();
    m->mouseDir = (Direction)((m->mouseDir + 1) % 4);
}

void ccw_step(Maze *m)
{
    API::turnLeft();
    m->mouseDir = Direction((m->mouseDir + 3) % 4);
}

void setGoalCenter(Maze *m)
{
    m->distances[7][7] = 0;
    m->distances[7][8] = 0;
    m->distances[8][7] = 0;
    m->distances[8][8] = 0;
}

void updateSimulator(Maze m)
{
    int mx = m.mousePos.x;
    int my = m.mousePos.y;
    for (int y = 0; y < 16; ++y)
    {
        for (int x = 0; x < 16; ++x)
        {
            API::setText(x, y, std::to_string(m.distances[x][y]));
            if (m.cellWalls[x][y] & NORTH_MASK)
            {
                if (x == mx && y == my)
                 API::setWall(x, y, 'n');
            }

            if (m.cellWalls[x][y] & EAST_MASK)
            {
                if (x == mx && y == my)
                 API::setWall(x, y, 'e');
            }
            if (m.cellWalls[x][y] & SOUTH_MASK)
            {
                if (x == mx && y == my)
                 API::setWall(x, y, 's');
            }
            if (m.cellWalls[x][y] & WEST_MASK)
            {
                if (x == mx && y == my)
                 API::setWall(x, y, 'w');
            }
        }
    }
}

// optimize:
void scanWalls(Maze *m)
{
    // scan front
    if (m->mouseDir == NORTH)
    {
        if (API::wallFront())
        {
            m->verticalWalls[m->mousePos.x][m->mousePos.y + 1] = true;
        }
        if (API::wallRight())
        {
            m->horizontalWalls[m->mousePos.x + 1][m->mousePos.y] = true;
        }
        if (API::wallLeft())
        {
            m->horizontalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
    }
    else if (m->mouseDir == EAST)
    {
        if (API::wallFront())
        {
            m->horizontalWalls[m->mousePos.x + 1][m->mousePos.y] = true;
        }
        if (API::wallRight())
        {
            m->verticalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
        if (API::wallLeft())
        {
            m->verticalWalls[m->mousePos.x][m->mousePos.y + 1] = true;
        }
    }
    else if (m->mouseDir == SOUTH)
    {
        if (API::wallFront())
        {
            m->verticalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
        if (API::wallRight())
        {
            m->horizontalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
        if (API::wallLeft())
        {
            m->horizontalWalls[m->mousePos.x + 1][m->mousePos.y] = true;
        }
    }
    else if (m->mouseDir == WEST)
    {
        if (API::wallFront())
        {
            m->horizontalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
        if (API::wallRight())
        {
            m->verticalWalls[m->mousePos.x][m->mousePos.y + 1] = true;
        }
        if (API::wallLeft())
        {
            m->verticalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
    }
}

void updateWalls(Maze *m)
{
    for (int y = 0; y <= 16; ++y)
    {
        for (int x = 0; x < 16; ++x)
        {
            if (m->verticalWalls[x][y])
            {
                if (y == 0)
                    m->cellWalls[x][y] = SOUTH_MASK;
                else if (y == 16)
                    m->cellWalls[x][y - 1] = NORTH_MASK;
                else
                {
                    m->cellWalls[x][y] |= SOUTH_MASK;
                    m->cellWalls[x][y - 1] |= NORTH_MASK;
                }
            }
            if (m->horizontalWalls[y][x])
            {
                if (y == 0)
                    m->cellWalls[y][x] |= WEST_MASK;
                else if (y == 16)
                    m->cellWalls[y - 1][x] |= EAST_MASK;
                else
                {
                    m->cellWalls[y][x] |= WEST_MASK;
                    m->cellWalls[y - 1][x] |= EAST_MASK;
                }
            }
        }
    }
}

void updatePos(Maze *m)
{
    API::moveForward();
    m->exploredCells[m->mousePos.x][m->mousePos.y] = true;
    API::setColor(m->mousePos.x, m->mousePos.y, 'G');
    if (m->mouseDir == NORTH)
        m->mousePos.y++;
    if (m->mouseDir == SOUTH)
        m->mousePos.y--;
    if (m->mouseDir == WEST)
        m->mousePos.x--;
    if (m->mouseDir == EAST)
        m->mousePos.x++;
}


Direction dir(Maze *m)
{
    Direction d = m->mouseDir;
    return d;
}

void floodfill(Maze *m, bool returning)
{
    Coord queue[255];
    int h{0}, t{0};
    // set goal cell distances to 0
    _reset_distances(m);
    if (!returning) {
        setGoalCenter(m);
        queue[t] = Coord{7, 7};
        ++t;
        queue[t] = Coord{7, 8};
        ++t;
        queue[t] = Coord{8, 7};
        ++t;
        queue[t] = Coord{8, 8};
        ++t;
    }   else if (returning) {
        m->distances[0][0] = 0;
        queue[t] = Coord{0,0};
        ++t;
    }
    
    while (t - h > 0)
    {
        Coord cur_pos = queue[h];
        ++h;
        int newcost = m->distances[cur_pos.x][cur_pos.y] + 1;
        CellList *neighbor = getNeighborCells(m, cur_pos);
        int n_size = neighbor->size;
        for (int i = 0; i < n_size; ++i)
        {
            if (!neighbor->cells[i].blocked)
            {
                if (m->distances[neighbor->cells[i].pos.x][neighbor->cells[i].pos.y] > newcost)
                {
                    m->distances[neighbor->cells[i].pos.x][neighbor->cells[i].pos.y] = newcost;
                    queue[t] = Coord{neighbor->cells[i].pos.x, neighbor->cells[i].pos.y};
                    ++t;
                }
            }
        }
        free(neighbor->cells);
        free(neighbor);
    }
}

void _reset_distances(Maze *m)
{
    for (int i = 0; i < 16; ++i)
    {
        for (int j = 0; j < 16; ++j)
            m->distances[i][j] = MAXCOST;
    }
}


void DFS(Maze &m, bool returning) {
    floodfill(&m, returning);
    while (m.distances[m.mousePos.x][m.mousePos.y] != 0)
    {
        floodfill(&m, returning);
        scanWalls(&m);
        updateWalls(&m);
        updateSimulator(m);
        Cell best = getBestCell(&m);

        if (best.dir == m.mouseDir);
        else if (best.dir == (m.mouseDir + 3) % 4) ccw_step(&m);
        else if (best.dir == (m.mouseDir + 1) % 4) cw_step(&m);
        else
        {
            cw_step(&m);
            cw_step(&m);
        }
        updatePos(&m);
    }
}

void quickestKnown(Maze &m, bool returning) {
    floodfill(&m, returning);
    while (m.distances[m.mousePos.x][m.mousePos.y] != 0)
    {
        floodfill(&m, returning);
        scanWalls(&m);
        updateWalls(&m);
        updateSimulator(m);
        Cell best = getBestKnownCell(&m);
        if (best.dir == m.mouseDir) ;
        else if (best.dir == (m.mouseDir + 3) % 4) ccw_step(&m);
        else if (best.dir == (m.mouseDir + 1) % 4) cw_step(&m);
        else
        {
            cw_step(&m);
            cw_step(&m);
        }

        updatePos(&m);
    }
}



int main(int argc, char *argv[])
{
    Maze m;

    log("Running...");
    std::cerr << "RUNNNING" << std::endl;
    API::setColor(0, 0, 'G');
    API::setText(0, 0, "abc");
    API::setWall(0, 0, 'w');
    API::setWall(0, 0, 's');
    init_walls(&m);
    DFS(m, false);
    DFS(m, true);
    DFS(m, false);
    DFS(m, true);
    DFS(m, false);
    DFS(m, true);
    DFS(m, false);
    DFS(m, true);
    
}
