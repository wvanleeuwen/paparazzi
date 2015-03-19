#include <iostream>
#include <sstream>      // std::stringstream
#include <stdio.h>
using namespace std;

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

int feature_size, border;
int RES = 100;
int x_response, y_response;
uint whole_area;

Mat integral_image;

void get_integral_image( Mat* img, uint width, uint height)
{
	printf("w = %d, h = %d\n",width, height);
	for(uint x = 0; x < width; x++)
	{
		for(uint y = 0; y < height; y++)
		{
			if( x >= 1 && y >= 1 )
			{
				integral_image.at<int>(y,x) = img->at<unsigned char>(y,x) + integral_image.at<int>(y,x-1) + integral_image.at<int>(y-1,x) - integral_image.at<int>(y-1,x-1);
			}
			else if(x >= 1)
			{
				integral_image.at<int>(y,x) = img->at<unsigned char>(y,x) + integral_image.at<int>(y,x-1);
			}
			else if(y >= 1)
			{
				integral_image.at<int>(y,x) = img->at<unsigned char>(y,x) + integral_image.at<int>(y-1,x);
			}
			else
			{
				integral_image.at<int>(y,x) = img->at<unsigned char>(y,x);
			}

			if (integral_image.at<int>(y,x) > pow(2,30))
			{
			    printf("OVERFLOW!");
			    exit(1);
			}
		}
	}
}

int get_sum( uint x_min, uint y_min, uint x_max, uint y_max)
{
	//printf("\n x_response: %d y_response: %d edges: %d, %d, %d, %d\n ", x_response, y_response, integral_image.at<int>(y_min,x_min), integral_image.at<int>(y_max,x_max), integral_image.at<int>(y_min,x_max), integral_image.at<int>(y_max,x_min));
	return (integral_image.at<int>(y_min,x_min) + integral_image.at<int>(y_max,x_max) - integral_image.at<int>(y_min,x_max) - integral_image.at<int>(y_max,x_min));
}

int	px_inner, px_whole, px_border, median_val;

uint get_obs_response()
{
	int sub_area, resp;
	//whole_area = get_sum( x_response - border, y_response, x_response+feature_size+border, y_response+feature_size);
	sub_area = get_sum( x_response, y_response, x_response+feature_size, y_response+feature_size);

/*	printf("px_inner: %d \n",px_inner);
	printf("whole_area: %d \n",whole_area);
	printf("sub_area: %d \n",sub_area);
	printf("px_border: %d \n",px_border);
//	printf("num: %d %d \n",((sub_area*RES) / px_inner), (sub_area * px_border));
//	printf("den: %d %d \n",(((whole_area - sub_area)*RES) / px_border ), ((whole_area - sub_area) * px_inner  ));
	printf("num: %d %d \n",((sub_area) / px_inner), (sub_area * px_border));
	printf("den: %d %d \n",median_val, (((whole_area - sub_area)*RES) / px_border ));
	printf("x_response: %d \n",x_response);
	printf("y_response: %d \n",y_response);
	printf("feature_size: %d \n",feature_size);*/

	// resp =  (RES*((inner_area*RES) / px_inner)) / (((whole_area - inner_area)*RES) / px_border );

	if ((whole_area - sub_area) > 0)
		//resp =  (RES*((sub_area*RES) / px_inner)) / (((whole_area - sub_area)*RES) / px_border );
		//resp =  (sub_area * px_border) / ((whole_area - sub_area) * px_inner  );
		resp =  RES*abs((sub_area / px_inner) - median_val)/255;
	else resp = RES;

	printf("resp: %d \n",resp);
	/*print_number(whole_area, 0);
	print_space();
	print_number(inner_area, 0);
	print_space();
	print_number(resp, 1);*/

	return resp;
}

/*
 * The following code is public domain.
 * Algorithm by Torben Mogensen, implementation by N. Devillard.
 * This code in public domain.
 */

typedef unsigned char elem_type ;

elem_type torben(elem_type m[], int n)
{
    int         i, less, greater, equal;
    elem_type  min, max, guess, maxltguess, mingtguess;

    min = max = m[0] ;
    for (i=1 ; i<n ; i++) {
        if (m[i]<min) min=m[i];
        if (m[i]>max) max=m[i];
    }

    while (1) {
        guess = (min+max)/2;
        less = 0; greater = 0; equal = 0;
        maxltguess = min ;
        mingtguess = max ;
        for (i=0; i<n; i++) {
            if (m[i]<guess) {
                less++;
                if (m[i]>maxltguess) maxltguess = m[i] ;
            } else if (m[i]>guess) {
                greater++;
                if (m[i]<mingtguess) mingtguess = m[i] ;
            } else equal++;
        }
        if (less <= (n+1)/2 && greater <= (n+1)/2) break ;
        else if (less>greater) max = maxltguess ;
        else min = mingtguess;
    }
    if (less >= (n+1)/2) return maxltguess;
    else if (less+equal >= (n+1)/2) return guess;
    else return mingtguess;
}

int median (Mat image)
{
/*	double m=(image.rows*image.cols)/2;
	int bin = 0;
	int med;
	med=-1;

	int histSize = 256;
	float range[] = { 0, 256 };
	const float* histRange = { range };
	bool uniform = true;
	bool accumulate = false;
	cv::Mat hist;

	cv::calcHist( &image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );
	for (int i=0; i<256 && med < 0;i++)
	{
		bin = bin + cvRound(hist.at<float>(i));
		if (bin > m && med < 0)
			med = i;
	}



	printf("old: %d, new: %d\n",med, *(image.data + sizeof(image.data)/2));//image.at<unsigned char>(image.rows/2, image.cols/2));

	return med;
 	 */

	return torben(image.data,image.rows*image.cols);
}

int main( int argc, const char** argv )
{
	int response;
	if (argc < 3)
	{
		cout << "Error : No image argument given!!" << endl;
		//system("pause"); //wait for a key press
		return -1;
	}

	namedWindow("MyWindow", CV_WINDOW_AUTOSIZE); //create a window with the name "MyWindow"

	char file[255];
	for (int i = atoi(argv[1]); i < atoi(argv[1]) + atoi(argv[2]); i++){

	  sprintf(file, "../Archive/img_%04d.jpg", i);
	  printf("%s\n",file);
	  //return 1;

    Mat img = imread(file, CV_LOAD_IMAGE_COLOR); //read the image data in the file "MyPic.JPG" and store it in 'img'

    printf("Image type: %d channels: %d \n", img.type(), img.channels());

    if (img.empty()) //check whether the image is loaded or not
    {
      cout << "Error : Image cannot be loaded..!!" << endl;
      //system("pause"); //wait for a key press
      return -1;
    }

    //integral_image.convertTo( img, 3 ); // CV_8U=0, CV_8S=1, CV_16U=2, CV_16S=3, CV_32S=4, CV_32F=5, CV_64F=6

    //cvtColor(img,img,CV_BGR2GRAY);

    int sub_img_h = 301;
    Mat sub_image;
    cvtColor(img(Rect(0, img.rows - sub_img_h, img.cols, sub_img_h)), sub_image, CV_RGB2YCrCb);
    //cvtColor(img(Rect(0, img.rows - sub_img_h, img.cols, sub_img_h)), sub_image, CV_RGB2HSV);

    printf("Image yuv type: %d channels: %d \n", img.type(), img.channels());

    vector<Mat> channels(3);
    split(sub_image, channels);

    sub_image = channels[0];

    median_val = median(sub_image);

    printf("median! %d\n", median_val);

    integral_image = sub_image.clone();
    integral_image.convertTo( integral_image, 4 ); // CV_8U=0, CV_8S=1, CV_16U=2, CV_16S=3, CV_32S=4, CV_32F=5, CV_64F=6
    get_integral_image( &sub_image, integral_image.cols, integral_image.rows  );

    printf("whole_area = %d width: %d, height: %d\n", whole_area, integral_image.cols, integral_image.rows);

    feature_size = 50;
    border = feature_size*2;

    px_inner = feature_size * feature_size;
    px_whole = (feature_size+2*border)*(feature_size);
    px_border = px_whole - px_inner;

    for (y_response = 0; y_response < integral_image.rows - feature_size; y_response+=feature_size){
      for (x_response = 0; x_response < integral_image.cols - feature_size; x_response+=feature_size){
    	  response = get_obs_response();
        if (response > 12){
          rectangle( img,
                 Point( x_response, img.rows - sub_img_h + y_response),
                 Point( x_response+feature_size, img.rows - sub_img_h + y_response + feature_size),
                 Scalar( 0, 0, 255 ),
                 2,
                 8 );
/*        	 rectangle( sub_image,
					 Point( x_response,  y_response),
					 Point( x_response+feature_size, y_response + feature_size),
					 Scalar( 0, 0, 255 ),
					 2,
					 8 );*/
        }
      }
    }

    cvtColor(img, img, CV_RGB2YCrCb);
    //cvtColor(img, img, CV_RGB2HSV);
    split(img, channels);

    imshow("MyWindow", channels[0]); //display the image which is stored in the 'img' in the "MyWindow" window

    waitKey(0); //wait infinite time for a keypress
	}

	destroyWindow("MyWindow"); //destroy the window with the name, "MyWindow"

	return 0;
}

