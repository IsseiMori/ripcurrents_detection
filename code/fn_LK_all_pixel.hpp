#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "method.hpp"

using namespace std;


class fn_LK_all_pixel: public method {
	private:
	public:
		fn_LK_all_pixel (string _file_name,
					 int _height);
		void run ();
		void justrun();
		void LK_to_color (vector<Pixel2>& vertices, vector<Pixel2>& vertices_next, Mat& out_img);
};