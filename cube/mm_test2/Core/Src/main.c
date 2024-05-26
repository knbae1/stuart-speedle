/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

typedef enum {
	DIST_FL,
	DIST_FR,
	DIST_L,
	DIST_R
	}dist_t;


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int32_t MAXCOST = 255;
_Bool on = false;

uint16_t enc_left = 0;
uint16_t enc_right = 0;
int16_t enc_left_typeC = 0;	//type-casted encoder values
int16_t enc_right_typeC = 0;
int32_t position_left = 0; //changed to 32bit integer to prevent overflow, may need to typecast?
int16_t speed_left =0;	//speed var for export
int32_t position_right = 0;
int16_t speed_right = 0; //speed var for export into it.c where speed is calculated
extern float speed_diff_left_cm;


//var for angle error control loop
uint16_t goal_angle = 0;
uint16_t angle_from_enc_error = 0;
uint16_t target_speed_left = 50;
uint16_t target_speed_right = 50;

//P and D variables for PID loop for encoders
int enc_error_current = 0;
int enc_error_past = 0;
int encoder_derivative = 0;
int encoder_integral = 0;
float enc_Kp = 1;
float enc_Ki = 0;
float enc_Kd = 0;
float enc_control_signal = 0;





//var's for distance from IR sensors
int16_t dis_FR = 0;
int16_t dis_FL = 0;
int16_t dis_L = 0;
int16_t dis_R = 0;
_Bool a = false;

//function for PD controller for distance
	//void PD_angle_controller(){	//will also need an distance controller
		//encoder_derivative = enc_error_current - enc_error_past;


//}

//function to obtain encoder values and convert them ro encoder revolutions
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	// this is the left encoder timer
	/*if (htim->Instance == TIM3) {
		enc_left = __HAL_TIM_GET_COUNTER(htim);
}*/
	if (htim->Instance == TIM3) {
		enc_left = __HAL_TIM_GET_COUNTER(htim);
		enc_left_typeC = (int16_t)enc_left*(-1);
		position_left =  enc_left_typeC/360;
	}
	if (htim->Instance == TIM4) {
		enc_right = __HAL_TIM_GET_COUNTER(htim);
		enc_right_typeC = (int16_t)enc_right*(-1);
		position_right = enc_right_typeC/360;

	}


	enc_error_current = enc_left_typeC - enc_right_typeC;	//consider moving into SYS tick handler to use interrupts to calculate errors
	encoder_derivative = enc_error_current - enc_error_past; //settles around 0?? sum of errors
	encoder_integral += enc_error_current;
	enc_control_signal = enc_Kd*encoder_derivative + enc_Ki*encoder_integral + enc_Kp*enc_error_current;

	enc_error_past = enc_error_current;




}

// API EQUIVALENT FUNCTIONS START

uint32_t calculatePWM(uint32_t)
{
	//given a goal speed, return the value to set CCR4/CCR3
	return 0;
}

_Bool wallFront()
{
	return (dis_FR > 200 && dis_FL > 200);
}

_Bool wallRight()
{
	return dis_R > 200;
}

_Bool wallLeft()
{
	return dis_L > 200;
}

void move_bwd(uint32_t delay, uint32_t Lspeed, uint32_t Rspeed)
{
	  TIM2->CCR4 = Lspeed;
	  TIM2 ->CCR3 = Rspeed;

	  HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 1); // ml fwd high
	  HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 0);	//mlbwd low

	  HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 1); //mr fwd high
	  HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 0);
	  HAL_Delay(delay);
	  TIM2->CCR4 = 0;
	  	TIM2 ->CCR3 = 0;

}



void move_fwd(uint32_t delay, uint32_t Lspeed, uint32_t Rspeed)	//need a parameter to adjust TIM2 CCR4/3 and pass to while loop
{
	TIM2->CCR4 = Lspeed;
	TIM2 ->CCR3 = Rspeed;
	//bwd
	HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 0); // ml fwd high
	HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 1);	//mlbwd low

	HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 0); //mr fwd high
	HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 1);

	HAL_Delay(delay);
	TIM2->CCR4 = 0;
	TIM2 ->CCR3 = 0;
}

void rotate_right(uint32_t delay, uint32_t speed)
{
	TIM2->CCR4 = speed;
	TIM2 ->CCR3 = speed;
	//bwd
	HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 0); // ml fwd high
	HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 1);	//mlbwd low

	HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 1); //mr fwd high
	HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 0);

	HAL_Delay(delay);
	TIM2->CCR4 = 0;
		TIM2 ->CCR3 = 0;
}

void rotate_left(uint32_t delay, uint32_t speed)
{

	TIM2->CCR4 = speed;
	TIM2 ->CCR3 = speed;
	//bwd
	HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 1); // ml fwd high
	HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 0);	//mlbwd low

	HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 0); //mr fwd high
	HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 1);

	HAL_Delay(delay);
	TIM2->CCR4 = 0;
	TIM2 ->CCR3 = 0;
}

void turn_right(uint32_t speed) {
	//smooth turn
	;
}

void turn_left(uint32_t speed) {;}

// END API FUNCTIONS

//functions to select and initialize adc CHANNELS
static void ADC1_Select_CH4(void){
ADC_ChannelConfTypeDef sConfig = {0};
		sConfig.Channel = ADC_CHANNEL_4;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
	}


static void ADC1_Select_CH9(void){
		ADC_ChannelConfTypeDef sConfig = {0};
			sConfig.Channel = ADC_CHANNEL_9;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
			if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			{
				Error_Handler();
			}
	}

static void ADC1_Select_CH5(void){
		ADC_ChannelConfTypeDef sConfig = {0};
			sConfig.Channel = ADC_CHANNEL_5;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
			if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			{
				Error_Handler();
			}
	}
static void ADC1_Select_CH8(void){
		ADC_ChannelConfTypeDef sConfig = {0};
			sConfig.Channel = ADC_CHANNEL_8;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
			if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			{
				Error_Handler();
			}
	}

uint16_t measure_dist(dist_t dist){
	GPIO_TypeDef* emitter_port;
	uint16_t emitter_pin;
	GPIO_TypeDef* receiver_port;
	uint16_t receiver_pin;
	switch(dist) {
	case DIST_FL:
		emitter_port	= EMIT_FL_GPIO_Port; //assign ports to variables
		emitter_pin		= EMIT_FL_Pin;	//assign pins to variables
		receiver_port	= RECIV_FL_GPIO_Port;
		receiver_pin 	= RECIV_FL_Pin;
		ADC1_Select_CH9();
		break;
	case DIST_FR:
			emitter_port	= EMIT_FR_GPIO_Port;
			emitter_pin		= EMIT_FR_Pin;
			receiver_port	= RECIV_FR_GPIO_Port;
			receiver_pin 	= RECIV_FR_Pin;
			ADC1_Select_CH4();
			break;
	case DIST_R:
				emitter_port	= EMIT_R_GPIO_Port;
				emitter_pin		= EMIT_R_Pin;
				receiver_port	= RECIV_R_GPIO_Port;
				receiver_pin 	= RECIV_R_Pin;
				ADC1_Select_CH5();
				break;
	case DIST_L:
					emitter_port	= EMIT_L_GPIO_Port;
					emitter_pin		= EMIT_L_Pin;
					receiver_port	= RECIV_L_GPIO_Port;
					receiver_pin 	= RECIV_L_Pin;
					ADC1_Select_CH8();
					break;
	default:
			break;
	}
	HAL_GPIO_WritePin(emitter_port, emitter_pin, GPIO_PIN_SET); //this function enables us to write to a pin
	HAL_Delay(5);
	HAL_ADC_Start(&hadc1); //activate the ADC
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return adc_val;
}

// FLOODFILL MAZE LOGIC START

typedef struct Coord {
    int x;
    int y;
}Coord;


typedef enum Direction
{
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
}Direction;

typedef enum DirectionBitmask
{
    NORTH_MASK = 0b1000,
    EAST_MASK = 0b0100,
    SOUTH_MASK = 0b0010,
    WEST_MASK = 0b0001
}DirectionBitmask;


typedef struct Cell {
    Coord pos;
    Direction dir;
    bool blocked;
}Cell;

typedef struct CellList {
    int size;
    Cell* cells;
}CellList;

typedef struct Maze{
    Coord mousePos;
    Direction mouseDir;

    int distances[16][16];
    bool exploredCells[16][16];
    int cellWalls[16][16];
    bool verticalWalls[16][17];
    bool horizontalWalls[17][16];
}Maze;

_Bool _in_bounds(Coord c);
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

void floodfill(Maze *m, _Bool returning);

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
            }
            else if (x == 15 && y != 16)
            {
                m->horizontalWalls[x+1][y] = true;
                m->verticalWalls[y][x+1] = true;
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
	    Cell best = (Cell){(Coord){255, 255}, NORTH, true};
	    // first choice, explored and less than curr distance, cannot call on first run
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

Cell getBestCell(Maze *m)
{
    int mx = m->mousePos.x;
    int my = m->mousePos.y;
    int shortest = m->distances[mx][my];
    CellList *best_cells = getNeighborCells(m, m->mousePos);
    int b_size = best_cells->size;
    Cell best = {(Coord){255, 255}, NORTH, true};
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
    rotate_right(1000, 1300);
    m->mouseDir = (Direction)((m->mouseDir + 1) % 4);
}

void ccw_step(Maze *m)
{
    rotate_left(1000, 1300);
    m->mouseDir = (Direction)((m->mouseDir + 3) % 4);
}

void setGoalCenter(Maze *m)
{
    m->distances[7][7] = 0;
    m->distances[7][8] = 0;
    m->distances[8][7] = 0;
    m->distances[8][8] = 0;
}

// optimize:
void scanWalls(Maze *m)
{
    // scan front
    if (m->mouseDir == NORTH)
    {
        if (wallFront())
        {
            m->verticalWalls[m->mousePos.x][m->mousePos.y + 1] = true;
        }
        if (wallRight())
        {
            m->horizontalWalls[m->mousePos.x + 1][m->mousePos.y] = true;
        }
        if (wallLeft())
        {
            m->horizontalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
    }
    else if (m->mouseDir == EAST)
    {
        if (wallFront())
        {
            m->horizontalWalls[m->mousePos.x + 1][m->mousePos.y] = true;
        }
        if (wallRight())
        {
            m->verticalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
        if (wallLeft())
        {
            m->verticalWalls[m->mousePos.x][m->mousePos.y + 1] = true;
        }
    }
    else if (m->mouseDir == SOUTH)
    {
        if (wallFront())
        {
            m->verticalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
        if (wallRight())
        {
            m->horizontalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
        if (wallLeft())
        {
            m->horizontalWalls[m->mousePos.x + 1][m->mousePos.y] = true;
        }
    }
    else if (m->mouseDir == WEST)
    {
        if (wallFront())
        {
            m->horizontalWalls[m->mousePos.x][m->mousePos.y] = true;
        }
        if (wallRight())
        {
            m->verticalWalls[m->mousePos.x][m->mousePos.y + 1] = true;
        }
        if (wallLeft())
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
    move_fwd(1000, 1300, 1300);
    m->exploredCells[m->mousePos.x][m->mousePos.y] = true;
    //API::setColor(m->mousePos.x, m->mousePos.y, 'G');
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

void floodfill(Maze *m, _Bool returning)
{
    Coord queue[255];
    int h = 0, t = 0;
    // set goal cell distances to 0
    _reset_distances(m);
    if (!returning) {
        setGoalCenter(m);
        queue[t] = (Coord){7, 7};
        ++t;
        queue[t] = (Coord){7, 8};
        ++t;
        queue[t] = (Coord){8, 7};
        ++t;
        queue[t] = (Coord){8, 8};
        ++t;
    }   else if (returning) {
        m->distances[0][0] = 0;
        queue[t] = (Coord){0,0};
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
                    queue[t] = (Coord){neighbor->cells[i].pos.x, neighbor->cells[i].pos.y};
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
        for (int j = 0; j < 16; ++j) m->distances[i][j] = MAXCOST;
    }
}


void DFS(Maze *m, _Bool returning) {
    floodfill(m, returning);
    while (m->distances[m->mousePos.x][m->mousePos.y] != 0)
    {
        floodfill(m, returning);
        scanWalls(m);
        updateWalls(m);

        Cell best = getBestCell(m);

        if (best.dir == m->mouseDir);
        else if (best.dir == (m->mouseDir + 3) % 4) ccw_step(m);
        else if (best.dir == (m->mouseDir + 1) % 4) cw_step(m);
        else
        {
            cw_step(m);
            cw_step(m);
        }
        updatePos(m);
    }
}

void quickestKnown(Maze *m, bool returning) {
    floodfill(m, returning);
    while (m->distances[m->mousePos.x][m->mousePos.y] != 0)
    {
        floodfill(m, returning);
        scanWalls(m);
        updateWalls(m);
        Cell best = getBestCell(m);
        if (best.dir == m->mouseDir) ;
        else if (best.dir == (m->mouseDir + 3) % 4) ccw_step(m);
        else if (best.dir == (m->mouseDir + 1) % 4) cw_step(m);
        else
        {
            cw_step(m);
            cw_step(m);
        }

        updatePos(m);
    }
}

//
//FLOODFILL MAZE LOGIC END

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
                                HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Maze m = {};
  m.mousePos = (Coord){0,0};
  m.mouseDir = NORTH;
      init_walls(&m);

  while (1)
  {
	  //calls to measure distance FR,FL,L,R can adjust delay in timing function to make the IR sensors more responsive
	  dis_FR = measure_dist(DIST_FR);
	  dis_FL = measure_dist(DIST_FL);
	  dis_R = measure_dist(DIST_R);
	  dis_L = measure_dist(DIST_L);


	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET){ //code for reading a button input/output
		  on = true;
	  }


	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET) on = false;

	  if (on) {
		  /*
	  		 DFS(&m, false);
	  		 DFS(&m, true);
	  		 DFS(&m, false);
	  		 DFS(&m, true);
	  		 */
		  if (dis_FR > 100 || dis_FL > 100) {
			  rotate_left(400, 1300);
		  }
		  else {
			  move_fwd(100, 1300, 1300);
		  }
	  }
	  else {
		  //quickestKnown(&m, false);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2047;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 25;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1024;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EMIT_R_Pin|EMIT_L_Pin|EMIT_FL_Pin|MR_FWD_Pin
                          |ML_BWD_Pin|MR_BWD_Pin|EMIT_FR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_2_Pin */
  GPIO_InitStruct.Pin = Button_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EMIT_R_Pin EMIT_L_Pin EMIT_FL_Pin MR_FWD_Pin
                           ML_BWD_Pin MR_BWD_Pin EMIT_FR_Pin */
  GPIO_InitStruct.Pin = EMIT_R_Pin|EMIT_L_Pin|EMIT_FL_Pin|MR_FWD_Pin
                          |ML_BWD_Pin|MR_BWD_Pin|EMIT_FR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ML_FWD_Pin */
  GPIO_InitStruct.Pin = ML_FWD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ML_FWD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_1_Pin */
  GPIO_InitStruct.Pin = Button_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
