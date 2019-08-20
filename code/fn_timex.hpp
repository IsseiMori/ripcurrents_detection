#pragma once

#include <opencv2/opencv.hpp>

#include "method.hpp"

using namespace std;


class fn_timex: public method {
	private:
	public:
		fn_timex (VideoCapture& _video,
					 int _height);

		/*
		 * buffer_size - default 0 will average over the entire frame
		 */
		void run (int buffer_size = 0);
};