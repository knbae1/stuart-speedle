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
};
// differentiate bt accessible and blocked cells
// void _init_walls(Maze *m);

bool _in_bounds(Coord c);
CellList *getNeighborCells(Maze *m, Coord c);
// returns best accessible cell for mouse to move to
Cell getBestCell(Maze *m);

// functions to step rotation, update API & mouseDir
void cw_step(Maze *m);
void ccw_step(Maze *m);

// sets a certain cell pos as the target cell
void setGoalCell(Maze *m);
// only updates the simulator
void updateSimulator(Maze m);
void _reset_distances(Maze *m);
// after having moved a step?, if called api move forward
void updatePos(Maze *m);

// same thing as update walls
void scanWalls(Maze *m);
Direction dir(Maze *m);

// updates distances based on new scanned walls
void floodfill(Maze *m);

void _init_maze(Maze *m)
{
    for (int y = 0; y <= 16; ++y)
    {
        for (int x = 0; x <= 16; ++x)
        {
            if (x == 0 && y != 16)
            {
                m->horizontalWalls[x][y] = true;
                API::setWall(x, y, 'w');
            }
            else if (x == 16 && y != 16)
            {
                // log("found hor wall 16");
                m->horizontalWalls[x][y] = true;
                API::setWall(x - 1, y, 'e');
            }
            if (y == 0 && x != 16)
            {
                m->verticalWalls[x][y] = true;
                API::setWall(x, y, 's');
            }
            else if (y == 16 && x != 16)
            {
                // log("found vert wall 16");
                m->verticalWalls[x][y] = true;
                API::setWall(x, y - 1, 'n');
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
    // log("here now,");
    CellList *neighbors = (CellList *)malloc(sizeof(CellList));
    int i{0};
    // set cell list size
    // check 4 directions, increment size for each valid direction;
    // checking north direction;
    neighbors->size = 4;
    neighbors->cells = (Cell *)malloc(neighbors->size * sizeof(Cell));
    // checking cell directly above target coord;

    // check if cell to the east is blocked
    //     std::cerr << "east cell at pos x,y " << c.x+1 << ", " << c.y << " is " << "blocked == " << m->horizontalWalls[c.x+1][c.y] << std::endl;
    // if not in bounds, it is blocked
    neighbors->cells[0] = (Cell){(Coord){c.x + 1, c.y}, EAST, m->horizontalWalls[c.x + 1][c.y] || !_in_bounds((Coord){c.x + 1, c.y})};

    neighbors->cells[1] = (Cell){(Coord){c.x, c.y + 1}, NORTH, m->verticalWalls[c.x][c.y + 1] || !_in_bounds((Coord){c.x, c.y + 1})};

    neighbors->cells[2] = (Cell){(Coord){c.x - 1, c.y}, WEST, m->horizontalWalls[c.x][c.y] || !_in_bounds((Coord){c.x - 1, c.y})};

    neighbors->cells[3] = (Cell){(Coord){c.x, c.y - 1}, SOUTH, m->verticalWalls[c.x][c.y] || !_in_bounds((Coord){c.x, c.y - 1})};

    // cell anatomy

    // Coord pos, Direction dir, bool blocked;

    return neighbors;
}

Cell getBestCell(Maze *m)
{
    // investigate distances (this is matrix of mannhatten distances to the goal cell)
    // check if wall is blocking cell
    int mx = m->mousePos.x;
    int my = m->mousePos.y;
    // int a{m->distances[m->mousePos.y][m->mousePos.x]};
    /*
    Cell a = (Cell){(Coord){mx, my+1}, NORTH, m->verticalWalls[mx][my+1]};
    Cell b = (Cell){(Coord){mx + 1, my}, EAST, m->horizontalWalls[mx+1][my]};
    Cell c = (Cell){(Coord){mx, my-1}, SOUTH, m->verticalWalls[mx][my]};
    Cell d = (Cell){(Coord){mx - 1, my}, WEST, m->horizontalWalls[mx][my]};
    */

    // best.pos.x = 0;
    // best.pos.y = 0;
    int shortest = m->distances[mx][my];
    CellList *best_cells = getNeighborCells(m, m->mousePos);
    int b_size = best_cells->size;
    Cell best{(Coord){255, 255}, NORTH, true};
    // first choice, unexplored and less than curr distance
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
    // second choice, unexplored
    if (best.blocked)
    {
        log("second");
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
        log("third");
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
        log("last");
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

    //   ||(m->distances[bx][by] < shortest)

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

void setGoalCell(Maze *m)
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
    // updatePos(&m);
    for (int y = 0; y < 16; ++y)
    {
        for (int x = 0; x < 16; ++x)
        {
            // std::string s = '(' + x + ','+ y + ')';
            std::string s = "(";
            s += std::to_string(x);
            s += ", ";
            s += std::to_string(y);
            s += ")";
            API::setText(x, y, std::to_string(m.distances[x][y]));
            // API::setText(x,y, s);
            // std::cerr << m.cellWalls[y][x] << " y is " << y << " x is " << x << std::endl;

            if (m.cellWalls[x][y] & NORTH_MASK)
            {
                if (x == mx && y == my)
                    std::cerr << "found wall north " << x << ", " << y << std::endl;
                API::setWall(x, y, 'n');
            }

            if (m.cellWalls[x][y] & EAST_MASK)
            {
                if (x == mx && y == my)
                    std::cerr << "found wall east " << x << ", " << y << std::endl;
                API::setWall(x, y, 'e');
            }
            if (m.cellWalls[x][y] & SOUTH_MASK)
            {
                if (x == mx && y == my)
                    std::cerr << "found wall south " << x << ", " << y << std::endl;
                API::setWall(x, y, 's');
            }
            if (m.cellWalls[x][y] & WEST_MASK)
            {
                if (x == mx && y == my)
                    std::cerr << "found wall west " << x << ", " << y << std::endl;
                API::setWall(x, y, 'w');
            }

            if (m.cellWalls[3][4] & NORTH_MASK)
                log("found 3,4 here");
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
            std::cerr << "front north: " << std::endl;
            m->verticalWalls[m->mousePos.x][m->mousePos.y + 1] = true;
        }
        if (API::wallRight())
        {
            std::cerr << "right north: " << std::endl;

            m->horizontalWalls[m->mousePos.x + 1][m->mousePos.y] = true;
        }
        if (API::wallLeft())
        {
            std::cerr << "left north: " << std::endl;

            m->horizontalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
    }
    else if (m->mouseDir == EAST)
    {
        if (API::wallFront())
        {
            std::cerr << "front east: " << std::endl;

            m->horizontalWalls[m->mousePos.x + 1][m->mousePos.y] = true;
        }
        if (API::wallRight())
        {
            std::cerr << "right east: " << std::endl;

            m->verticalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
        if (API::wallLeft())
        {
            std::cerr << "left east: " << std::endl;

            m->verticalWalls[m->mousePos.x][m->mousePos.y + 1] = true;
        }
    }
    else if (m->mouseDir == SOUTH)
    {
        if (API::wallFront())
        {
            std::cerr << "front south: " << std::endl;

            m->verticalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
        if (API::wallRight())
        {
            std::cerr << "right south: " << std::endl;

            m->horizontalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
        if (API::wallLeft())
        {
            std::cerr << "left south: " << std::endl;

            m->horizontalWalls[m->mousePos.x + 1][m->mousePos.y] = true;
        }
    }
    else if (m->mouseDir == WEST)
    {
        if (API::wallFront())
        {
            std::cerr << "front west: " << std::endl;

            m->horizontalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
        if (API::wallRight())
        {
            std::cerr << "right west: " << std::endl;

            m->verticalWalls[m->mousePos.x][m->mousePos.y + 1] = true;
        }
        if (API::wallLeft())
        {
            std::cerr << "left west: " << std::endl;

            m->verticalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
    }
}

void updateWalls(Maze *m)
{
    // turn horizontal walls & vertical walls into bitmask
    // first horizontal
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
    std::cerr << "inside function: (" << m->mousePos.x << ", " << m->mousePos.y << ")" << std::endl;
}

Direction dir(Maze *m)
{
    Direction d = m->mouseDir;
    if (d == NORTH)
    {
        std::cerr << d << std::endl;
    }
    return d;
}

void floodfill(Maze *m)
{
    Coord queue[255];
    int h{0}, t{0};

    // set goal cell distances to 0
    _reset_distances(m);
    setGoalCell(m);
    // add goal cells to queue
    queue[t] = Coord{7, 7};
    ++t;
    queue[t] = Coord{7, 8};
    ++t;
    queue[t] = Coord{8, 7};
    ++t;
    queue[t] = Coord{8, 8};
    ++t;
    // move incrementing head, after each iter of for loop, which checks one cell in the queue
    while (t - h > 0)
    {
        Coord cur_pos = queue[h];
        ++h;

        int newcost = m->distances[cur_pos.x][cur_pos.y] + 1;
        // std::cerr << "newcost is " << newcost << std::endl;
        CellList *neighbor = getNeighborCells(m, cur_pos);
        int n_size = neighbor->size;
        for (int i = 0; i < n_size; ++i)
        {
            // std::cerr << neighbor->cells[i].pos.x << ", " << neighbor->cells[i].pos.y << ")" << std::endl;
            if (!neighbor->cells[i].blocked)
            {

                if (m->distances[neighbor->cells[i].pos.x][neighbor->cells[i].pos.y] > newcost)
                {
                    //   std::cerr << "adding to queue (" <<  neighbor->cells[i].pos.x << ", " << neighbor->cells[i].pos.y << ") oldcost: " << m->distances[neighbor->cells[i].pos.x][neighbor->cells[i].pos.y]
                    //  << " newcost: " << newcost << std::endl;

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
    log("called for resetting distance!");
    for (int i = 0; i < 16; ++i)
    {
        for (int j = 0; j < 16; ++j)
            m->distances[i][j] = MAXCOST;
    }
}

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
        }
    }
}

int main(int argc, char *argv[])
{
    Maze m;
    log("Running...");
    std::cerr << "RUNNNING" << std::endl;

    std::cerr << NORTH << std::endl;
    API::setColor(0, 0, 'G');
    API::setText(0, 0, "abc");
    std::cerr << m.mousePos.x << std::endl;
    API::setWall(0, 0, 'w');
    API::setWall(0, 0, 's');
    _init_maze(&m);
    init_walls(&m);
    std::cerr << "set walls?" << std::endl;

    while (!m->reached_goal())
    {
        floodfill(&m);
        scanWalls(&m);
        updateWalls(&m);
        updateSimulator(m);
        std::cerr << m.verticalWalls[0][1] << " is 0,0 wall north" << std::endl;

        Cell best = getBestCell(&m);
        std::cerr << "curr mouse dir: " << m.mouseDir << " best cell: " << best.dir << " at pos: " << best.pos.x << ", " << best.pos.y << std::endl;

        if (best.dir == m.mouseDir)
        {
            // donothing
            log("moving forward");
        }
        else if (best.dir == (m.mouseDir + 3) % 4)
        {
            ccw_step(&m);
            log("rotate left, moving forward");
        }
        else if (best.dir == (m.mouseDir + 1) % 4)
        {
            cw_step(&m);
            log("rotate right, moving forward");
        }
        else
        {
            log("rotate 180 move back 1");
            cw_step(&m);
            cw_step(&m);
        }

        updatePos(&m);
    }
}