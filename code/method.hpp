#pragma once

#include <opencv2/opencv.hpp>

using namespace cv;

typedef cv::Point3_<uchar> Pixelc;
typedef cv::Point_<float> Pixel2;

class method {
	protected:
		VideoCapture video;
		double fps;
	public:
		int width;
		int height;

		method (VideoCapture& _video,
				int _height);
		VideoWriter* ini_video_output (String video_name);
		void vector_to_color(Mat& curr, Mat& out_img);
};