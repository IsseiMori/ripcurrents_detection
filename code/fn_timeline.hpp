#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "method.hpp"

using namespace std;

typedef cv::Point_<float> Pixel2;

class timeline {
	private:
		vector<Pixel2> vertices;
	public: 
		timeline(){};
		timeline(Pixel2 start, Pixel2 end, int vertices_count);

		// run LK method on each vertex and draw lines
		void runLK(Mat u_prev, Mat u_curr, Mat& out_img);

		// void runFarneBack();
};

class fn_timeline: public method {
	private:
		vector<timeline> timelines;
	public:
		fn_timeline (string file_name,
					 int _height);
		void run(int v_num = 10);
		void add_timeline (Pixel2 start, Pixel2 end, int vertices_count);
};