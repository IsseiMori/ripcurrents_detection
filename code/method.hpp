#pragma once

#include <opencv2/opencv.hpp>

using namespace cv;

typedef cv::Point3_<uchar> Pixelc;
typedef cv::Point_<float> Pixel2;

class method {
	protected:
		VideoCapture video;
		double fps;
		Mat curr_frame, prev_frame;
		Mat resized_frame;

	// Optical Flow
	protected:
		Mat flow;

	public:
		int width;
		int height;

		method (VideoCapture& _video, int _height);
		VideoWriter* ini_video_output (String video_name);
		void vector_to_color(Mat& curr, Mat& out_img);

		void run_optflow_FB ();
		void ini_frame ();
		void read_frame ();

};