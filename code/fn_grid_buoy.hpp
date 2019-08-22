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
		int v_count;
		int h_count;
		float max_len;
	public:
		fn_grid_buoy (string _file_name,
					 int _height,
					 int _v_count = 10,
					 int _h_count = 10);
		void runLK ();
		void runFB ();
		void vertices_runLK (Mat u_prev, Mat u_curr, Mat& out_img);
		void vertices_runFB (Mat& flow, Mat& out_img);
};