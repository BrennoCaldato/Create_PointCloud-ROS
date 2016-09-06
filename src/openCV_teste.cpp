#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/video.hpp"
#include "iostream"

using namespace cv;
int main(int argc, char** argv)
{
    VideoCapture cap;
    Mat dst;//dst image
    Size size(70,140);
    double r,g,b;
    namedWindow("output",CV_WINDOW_FREERATIO);
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(0))
        return 0;
        //for(;;)
         //{
               Mat frame;
               cap >> frame;
               //if( frame.empty() ) break; // end of video stream
               imshow("output", frame);
               //waitKey(0);

               //if( waitKey(1) == 27 ) break; // stop capturing by pressing ESC



               resize(frame,dst,size);//resize image
               //imshow("this is you, smile! :)", dst);

               for(int i = 0; i<dst.rows;i++){
                 for(int j = 0; j<dst.cols;j++){
                   r = dst.at<Vec3b>(i,j)[0];
                   g = dst.at<Vec3b>(i,j)[1];
                   b = dst.at<Vec3b>(i,j)[2];

                   std::cout << r << " "<< g <<" "<< b << std::endl;

                 }
               }


               //imshow("output", dst);
               waitKey(0);
         //}
         // the camera will be closed automatically upon exit
         // cap.close();
         return 0;
}
