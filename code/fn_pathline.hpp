#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "method.hpp"

using namespace std;


class fn_pathline: public method {
	private:
	public:
		fn_pathline (string _file_name,
					 int _height);
		void run ();
		void runLK (float v);
		void calc_pathline (Mat& out_img, Mat& overlay, Mat& overlay_color, vector<Pixel2>& streampt, int framecount);
};