#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "method.hpp"

using namespace std;

typedef cv::Point_<float> Pixel2;


class fn_grid_buoy: public method {
	private:
		vector<Pixel2> vertices_root;
		vector<Pixel2> vertices;
		float max_len;
	public:
		fn_grid_buoy (VideoCapture& _video,
					 int _height,
					 int v_count,
					 int h_count);
		void runLK ();
		void runFB ();
		void vertices_runLK (UMat u_prev, UMat u_curr, Mat& out_img);
		void vertices_runFB (Mat& flow, Mat& out_img);
};