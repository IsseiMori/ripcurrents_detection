#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "method.hpp"

using namespace std;


class fn_shear: public method {
	private:
		float max_displacement;
		float max_frobeniusNorm;
		float max_dot, min_dot;
		float max_e1, min_e1;
	public:
		fn_shear (string _file_name,
					 int _height);
		void run (int buffer_size, int offset, bool isNorm);
		void shearRateToColor(Mat& current, Mat& out_img, int offset, int framecount);
		void shearRateToColorMulti(Mat& current, Mat& out_img1, Mat& out_img2, Mat& out_img3, Mat& out_img4, Mat& out_img5, Mat& out_img6, int offset, int framecount);
		void shearKernelToColor(Mat& current, Mat& out_img, int offset, int framecount);
};