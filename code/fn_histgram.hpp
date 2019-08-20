#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "method.hpp"

using namespace std;

typedef cv::Point_<float> Pixel2;
typedef cv::Point3_<float> Pixel3;

#define HIST_BINS 50 //Number of bins for finding thresholds
#define HIST_DIRECTIONS 36 //Number of 2d histogram directions
#define HIST_RESOLUTION 20


class fn_histgram: public method {
	private:
	public:
		fn_histgram (VideoCapture& _video,
					 int _height);
		void run ();
		void create_histogram(Mat current, int hist[HIST_BINS], 
							  int& histsum, int hist2d[HIST_DIRECTIONS][HIST_BINS],
							  int histsum2d[HIST_DIRECTIONS], float& UPPER, 
							  float UPPER2d[HIST_DIRECTIONS], float prop_above_upper[HIST_DIRECTIONS]);
		void display_histogram(int hist2d[HIST_DIRECTIONS][HIST_BINS],int histsum2d[HIST_DIRECTIONS],
							   float UPPER2d[HIST_DIRECTIONS], float UPPER, 
							   float prop_above_upper[HIST_DIRECTIONS]);
};