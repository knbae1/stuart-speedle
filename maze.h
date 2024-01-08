//header file for maze class
//following pseudocode from floodfill lec slides
#include <iostream>
#include <string>
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

struct Cell 
{
    Coord pos;
    Direction dir;
    bool blocked;
};

class Maze 
{
    Coord mousePos;
    Direction mouseDir;
    int distances[16][16];
    bool exploredCells[16][16];
    int cellWalls[16][16];
    bool verticalWalls[16][17];
    bool horizontalWalls[17][16];
    Coord * goalPos;

    //differentiate bt accessible and blocked cells
    Cells* getNeighborCells();
    //returns best accessible cell for mouse to move to
    Cell getBestCell();

    //functions to return direction after step rotation
    Direction clockwise();
    Direction counterClockwiseStep();

    // sets a certain cell pos as the target cell
    void setGoalCell();

    void rotate();
    void move();
    // only updates the simulator
    void updateSimulator();

};
