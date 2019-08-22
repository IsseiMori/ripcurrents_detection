#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "method.hpp"

using namespace std;


class fn_dir_color: public method {
	private:
	public:
		fn_dir_color (string _file_name,
					 int _height);
		void run (int buffer_size = 1);
		void run_dir (int buffer_size = 1);
};