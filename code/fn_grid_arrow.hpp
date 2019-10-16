#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "method.hpp"

using namespace std;

typedef cv::Point_<float> Pixel2;


class fn_grid_arrow: public method {
	private:
		vector<Pixel2> root_vec;
		vector<Pixel2> v_vec;
		vector<Pixel2> relative_vec;
		vector<float> theta_vec;
		vector<bool> isVisible;
		vector<float> howLikely; // 0 (no rip) to 1 (rip)
		vector<vector<Pixel2>> vec_buffer;
		vector<Pixel2> average_vec;
		int w_count;
		int h_count;
		float max_len;
		int buffer_size;
	public:
		fn_grid_arrow (string _file_name,
					 int _height,
					 int _w_count,
					 int _h_count,
					 int _buffer_size);
		void runLK (bool isNorm = false);
		void runFB ();
		void vertices_runLK (Mat u_prev, Mat u_curr, Mat& out_img, bool isNorm, float framecount);
		void vertices_runFB (Mat& flow, Mat& out_img);
		void filter_total_mean ();
		void filter_row_col_mean ();
		float filter_frequency ();
};