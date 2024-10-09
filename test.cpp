#include "cv_plot.hpp"

int main()
{
  cv::Mat img1( 300, 600, CV_8UC3, cv::Scalar( 0, 0, 0 ) ); // create a black image
  cv::Mat img2( 300, 600, CV_8UC3, cv::Scalar( 0, 0, 0 ) ); // create a black image
  cv::Mat img3( 300, 600, CV_8UC3, cv::Scalar( 0, 0, 0 ) ); // create a black image

  for ( int i = 0; i < 300; i ++ ) {
    int val1 = rand() % 250 + 50;
    int val2 = rand() % 200 + 50;
    int val3 = rand() % 150 + 50;

    plot::drawChart( img1, -50, 50, 10, 10, false, val1, YELLOW, "val1" ); // draw one line
    plot::drawChart( img2, -50, 50, 10, 10, false, val1, val2, YELLOW, BLUE, "val1", "val2" ); // draw two lines
    plot::drawChart( img3, -50, 50, 10, 10, false, val1, val2, val3, YELLOW, BLUE, RED, "val1", "val2", "val3" ); // draw three lines

    // display
    cv::imshow( "line chart 1", img1 );
    cv::imshow( "line chart 2", img2 );
    cv::imshow( "line chart 3", img3 );

    cv::waitKey( 100 );
  }
  
  return 0;
}
