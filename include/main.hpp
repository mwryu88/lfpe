/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.hpp
 * Author: robot
 *
 * Created on August 29, 2018, 3:43 PM
 */

#ifndef MAIN_HPP
#define MAIN_HPP

#include <cstdlib>
#include "opencv/cv.hpp"
#include "opencv2/viz.hpp"
#include <vector>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include "icpPointToPlane.h"
//#include "icpPointToPoint.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>

#include <pthread.h>
#include "dynamixel.hpp"

#include <fstream>

using namespace std;
using namespace cv;
using namespace viz;

#define PI 3.1415926
#define RAD2DEG 57.295791433
//#define VIEW_POINTCLOUD
//#define USE_DINAMIXEL
#define VIEW_FLOORMAP
#define FLOORMAP_W 640    
#define FLOORMAP_H 320
#define IMAGE_W    1280
#define IMAGE_H    720

#define BUF_MAX 512

typedef struct
{
    float x;
    float y;
    float z;
}point3f;

typedef struct
{
    float a;
    float b;
    float c;
    float d;
}plane_t;

typedef struct
{
    float data[3][3];
}mat3x3_t;

point3f m_lineDepth[IMAGE_W][IMAGE_H];

typedef struct
{
    int x;
    int y;
    int z;
}point3d;

typedef struct
{
    int x;
    int y;
}point2d;

typedef struct
{
    float f;
    float cx;
    float cy;
}camera_param_t;

camera_param_t m_cam_param;

void initializeRS2(void);
void initializeCamParams(void);
float point2planeICP(vector<point2d> curr_line_points, vector<point2d> prev_line_points);
float kalmanFilter(float curr_angle);
void initUart(void);
void initializeThread(void);
void initialze(void);

void *thread_dynamixel(void *);
unsigned int servo(unsigned char ID,unsigned int Position,unsigned int Speed);

float det(mat3x3_t matrix);
float distPoint2Plane(point3f point, plane_t plane);
plane_t computePlane(point3f p1,point3f p2,point3f p3);

    // Create a simple OpenGL window for rendering:
    //window app(1280, 720, "RealSense Capture Example");
    // Declare two textures on the GPU, one for color and one for depth
texture depth_image, color_image, ir_image;
    // Declare depth colorizer for pretty visualization of depth data
rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
rs2::pipeline rs2_pipe;
    
    // Declare pointcloud object, for calculating pointclouds and texture mappings
rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
rs2::points points;
rs2::config cfg;

int fd;
char buf[BUF_MAX];

#endif /* MAIN_HPP */

