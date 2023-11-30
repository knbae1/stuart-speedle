#include <iostream>
#include <string>

#include "API.h"

void log(const std::string& text) 
{
    std::cerr << text << std::endl;
}

void pointer_demo(int var)
{
    var = 42;
}

enum Direction {
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

struct Coord {
    int x;
    int y;
};

struct Cell {
    Coord pos;
    Direction dir;
    bool blocked;
};

struct CellList {
    int size;
    Cell* cells;
};

struct Maze {
    Coord mouse_pos;
    Direction mouse_dir;

    int distances[16][16];
    int cellWalls[16][16];

    Coord* goalPos;
};

char dir_chars[4] = {'n', 'e', 's', 'w'};
int dir_mask[4] = {0b1000, 0b0100, 0b0010, 0b0001};

// 2. FILL THIS IN
void updateSimulator(Maze maze) // redraws the maze in simulator after each loop in main
{
    for (int y = 0; y < 16; ++y) {
        for (int x = 0; x < 16; ++x) {
            API::setText(x, y, std::to_string(maze.distances[y][x]));
            if (maze.cellWalls[y][x] & NORTH_MASK) {
                API::setWall(x, y, 'n');
            }

            if (maze.cellWalls[y][x] & EAST_MASK) {
                API::setWall(x, y, 'e');
            }
            if (maze.cellWalls[y][x] & SOUTH_MASK) {
                API::setWall(x, y, 's');
            }
            if (maze.cellWalls[y][x] & WEST_MASK) {
                API::setWall(x, y, 'w');
            }
            
        }
    }
}

// 5. FILL THIS IN
void scanWalls(Maze* maze)
{
   // API::setWall(maze->mouse_pos.x, maze->mouse_pos.y, dir_chars[maze->mouse_dir]);
    if (API::wallFront()) {
        maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x] |= dir_mask[maze->mouse_dir];
    }
    if (API::wallRight()) {
        maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x] |= dir_mask[(maze->mouse_dir + 1) % 4];

    }
    if (API::wallLeft()) {
        maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x] |= dir_mask[(maze->mouse_dir + 3) % 4];

    }

}

// 3. FILL THIS IN
void updateMousePos(Coord *pos, Direction dir)
{
    if (dir == NORTH)
            pos->y++;
        if (dir == SOUTH)
            pos->y--;
        if (dir == WEST)
            pos->x--;
        if (dir == EAST)
            pos->x++;
    std::cerr << "inside function: (" << pos->x << ", " << pos->y << ")" << std::endl;
}

// 6. FILL THIS IN
CellList* getNeighborCells(Maze* maze)
{
    CellList* cellList = (CellList*)malloc(sizeof(CellList));
    int i{0};
    // set cell list size
    cellList->size = 3;
    cellList->cells = (Cell*)malloc(cellList->size * sizeof(Cell));

    //check if north cell
   /* if (true) {
        //cellList->cells[i] = (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y + 1}}
        cellList->cells[i] = (Cell){(Coord){24,567}, NORTH, true};
        i++;
    }*/
   /* cellList->cells[i] = (Cell){(Coord){1, 0}, EAST, false};
    i++;
    cellList->cells[i] = (Cell){(Coord){4, 4}, WEST, true};
    i++;
    */
    return cellList;
};

Maze maze;

int temp_value = 20;

int main(int argc, char* argv[]) 
{
    maze.mouse_pos = (Coord){0, 0};
    maze.mouse_dir = NORTH;

    // 4. POINTER DEMO
    // pointer_demo(temp_value);
    // std::cerr << temp_value << std::endl;

    // 1. FILL THIS IN
    for(int x = 0; x < 16; x++) {
        for(int y = 0; y < 16; y++) {
            maze.distances[y][x] = x + y;
        }
    }

    while (true) {
        scanWalls(&maze);
        CellList* adjacentCells = getNeighborCells(&maze);
        
        for(int i =0; i < adjacentCells->size; ++i) {
            std::cerr << adjacentCells->cells[i].pos.x << ", " << adjacentCells->cells[i].pos.y << std::endl;
        }

        free(adjacentCells->cells);
        free(adjacentCells);

        updateSimulator(maze);

        std::cerr << "(" << maze.mouse_pos.x << ", " << maze.mouse_pos.y << ")" << std::endl;

        // Left Wall Follow Code
        if (!API::wallLeft()) 
        {
            API::turnLeft();
            maze.mouse_dir = (Direction)((maze.mouse_dir + 3) % 4);
        }
        while (API::wallFront()) 
        {
            API::turnRight();
            maze.mouse_dir = (Direction)((maze.mouse_dir + 1) % 4);
        }

        API::moveForward();
    
        // 3. UPDATE THIS WITH POINTERS updateMousePos
        updateMousePos(&maze.mouse_pos, maze.mouse_dir);

        // 5. MOVE TO updateSimulator() + scanWalls() FUNCTION
        if (API::wallFront())
            API::setWall(maze.mouse_pos.x, maze.mouse_pos.y, dir_chars[maze.mouse_dir]);
        if (API::wallRight())
            API::setWall(maze.mouse_pos.x, maze.mouse_pos.y, dir_chars[(maze.mouse_dir + 1) % 4]);
        if (API::wallLeft())
            API::setWall(maze.mouse_pos.x, maze.mouse_pos.y, dir_chars[(maze.mouse_dir + 3) % 4]);
    }
}