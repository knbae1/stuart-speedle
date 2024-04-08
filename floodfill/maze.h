//header file for maze class
//following pseudocode from floodfill lec slides
#ifndef MAZE_HPP
#define MAZE_HPP
#include <iostream>
#include <string>
//k
#include "API.h"
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
    Cell* getNeighborCells(Coord c);
    //returns best accessible cell for mouse to move to
    Cell getBestCell();

    //functions to return direction after step rotation
    Direction cw_step();
    Direction ccw_step();

    // sets a certain cell pos as the target cell
    void setGoalCell();

    void rotate();
    void move();
    // only updates the simulator
    void updateSimulator();

    //after having moved a step?, if called api move forward
    void updatePos();
    
    //same thing as update walls
    void scanWalls();
    Direction dir() const;

    //updates distances based on new scanned walls
    void floodfill();




#endif