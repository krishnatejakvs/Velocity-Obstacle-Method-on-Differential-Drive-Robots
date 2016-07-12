#include <opencv2/core/core.hpp> //Contains the Drawing Mat datatype and drawing functions.
#include <opencv2/highgui/highgui.hpp> //Contains functions for reading, writing and displaying images.

//The following was  used in Opencv v2.1 or before, it will work in v2.3, just to let you know the change in the structure
//#include <opencv/cv.h>
//#include <opencv/highgui.h>

using namespace cv;

// The function called in this program are used in cpp style. For v1.0 functions for most functions just 
// append "cv" before in the start of the function name. Cases not following the forementioned rule will be mentioned.
int main(int argc, char **argv)
{
  //In v1.0 (i.e. for C) has IplImage *img instead of Mat img.
  //and cvLoadImage(argv[1]) instead of imread(argv[1]).
  Mat img = imread(argv[1]);
  namedWindow("Display", CV_WINDOW_AUTOSIZE);

  //Drawing functions. last 3-4 arguments change the way figures are drawn, refer to documentation for more info.
  //For more figures and functions visit: http://opencv.willowgarage.com/documentation/cpp/core_drawing_functions.html
  line(img, Point(img.cols/4, img.rows/4), Point(3*img.cols/4, 3*img.rows/4), Scalar(0,255,0));
  circle(img, Point(img.cols/2, img.rows/2), 10, Scalar(255,0,0), 3);

  imshow("Display", img);// v1.0 - cvShowImage
  char key = waitKey();//If you pass argument than it will wait for that many milliseconds.
  if( key==27 )//27 is ASCII for <Esc>
  {
    return EXIT_SUCCESS;
  }
  imwrite("img_opencv.jpg", img);//v1.0 - cvSaveImage
}
