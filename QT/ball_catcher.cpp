#include "ball_catcher.h"

ball_catcher::ball_catcher()
{
    rectangle_left = Rect(x_left_home, y_left_home, width, height);
    rectangle_right = Rect(x_right_home, y_right_home, width, height);
}

void ball_catcher::get_first_image(Mat image)
{
    image_first = image;
}

void ball_catcher::set_roi(int x, int y)
{
    roi_rectangle = Rect(x,y,width,height);
}

void ball_catcher::draw_roi(Mat &image)
{

}
