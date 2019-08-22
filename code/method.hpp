#pragma once

#include <string>

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

typedef cv::Point3_<uchar> Pixelc;
typedef cv::Point_<float> Pixel2;

class method {
	protected:
		VideoCapture video;
		string file_name;
		double fps;
		Mat curr_frame, prev_frame;
		Mat resized_frame;

	// Optical Flow
	protected:
		Mat flow;

	public:
		int width;
		int height;

		method (string file_name, int _height);
		VideoWriter* ini_video_output (string video_name);
		void vector_to_color(Mat& curr, Mat& out_img);
		void vector_to_dir_color(Mat& curr, Mat& out_img);

		int ini_frame ();
		int read_frame ();

		void drawFrameCount (Mat& outImg, int framecount);

};