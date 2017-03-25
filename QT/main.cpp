//#include <iostream>
//#include "opencv2/opencv.hpp"
//#include "opencv2/videoio.hpp"
//#include <fstream>
//#include <string>
#include "ball_catcher.h"

//using namespace cv;
//using namespace std;

int main(int argc, char *argv[])
{

    Mat R1, P1, R2, P2, Q;
    Mat distCoeffs_left;
    Mat distCoeffs_right;
    Mat cameraMatrix_left;
    Mat cameraMatrix_right;

    Mat image_left;
    Mat image_right;
    Mat image_left_color;
    Mat image_right_color;

    string header;
    string tail;
    string filename_left;
    string filename_right;


    FileStorage fs_left("c:\\Projects\\BaseballCatcher - 10\\calibration\\calib_left.yaml", FileStorage::READ);
    fs_left["CameraMatrix"] >> cameraMatrix_left;
    fs_left["DistortionCoefficients"] >> distCoeffs_left;
    FileStorage fs_right("C:\\Projects\\BaseballCatcher - 10\\calibration\\calib_right.yaml", FileStorage::READ);
    fs_right["CameraMatrix"] >> cameraMatrix_right;
    fs_right["DistortionCoefficients"] >> distCoeffs_right;
    FileStorage fs_rectify("C:\\Projects\\BaseballCatcher - 10\\calibration\\Rectification_baseball_2.xml", FileStorage::READ);
    fs_rectify["R1"] >> R1;
    fs_rectify["P1"] >> P1;
    fs_rectify["R2"] >> R2;
    fs_rectify["P2"] >> P2;
    fs_rectify["Q"] >> Q;

    header = "/home/dallin/robotic_vision/HW4/ImagesDallin/Ball_test";
    tail = ".bmp";

    ball_catcher ball_left;
    ball_catcher ball_right;

    for(int i = 1; i < 100; i++)
    {
        filename_left = header + "L" + to_string(i) + tail;
        filename_right = header + "R" + to_string(i) + tail;

        image_left = imread(filename_left,CV_LOAD_IMAGE_GRAYSCALE);
        image_right = imread(filename_right,CV_LOAD_IMAGE_GRAYSCALE);

        if(i == 1)
        {
            ball_left.get_first_image(image_left);
            ball_right.get_first_image(image_right);
        }


        imshow("Left", image_left);
        imshow("right", image_right);
        moveWindow("right",643,23);
        waitKey(50);
    }
    imshow("first",ball_left.image_first);
    imshow("first_right",ball_right.image_first);

    waitKey(0);

    return 0;
}
