#ifndef BALL_CATCHER_H
#define BALL_CATCHER_H

#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include <fstream>
#include <string>
#include "ball_catcher.h"

using namespace cv;
using namespace std;

class ball_catcher
{


public:
    ball_catcher();
    void get_first_image(Mat);
    Point2d process_image(Mat);
};



#endif // BALL_CATCHER_H
