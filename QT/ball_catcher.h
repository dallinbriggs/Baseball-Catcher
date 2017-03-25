#ifndef BALL_CATCHER_H
#define BALL_CATCHER_H

#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

class ball_catcher
{
private:
    int width = 155;
    int height = 140;



public:
    ball_catcher();
    void get_first_image(Mat);
    Point2d process_image(Mat);
    void set_roi(int,int);
    void draw_roi(Mat &);


    Mat image_first;
    Rect rectangle_left;
    Rect rectangle_right;
    Rect roi_rectangle;

    int x_left_home = 275;
    int y_left_home = 88;
    int x_right_home = 210;
    int y_right_home = 88;
};



#endif // BALL_CATCHER_H
