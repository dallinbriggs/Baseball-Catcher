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



    return 0;
}
