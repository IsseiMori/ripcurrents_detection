#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include <fstream> 

#include <opencv2/opencv.hpp>

#include "fn_dir_color.hpp"

using namespace std;

fn_dir_color::fn_dir_color (VideoCapture& _video, 
							int _height)
							: method(_video, _height) {
}

void fn_dir_color::run (int buffer_size) {
	cout << "Running color map" << endl;

	VideoWriter* video_output = ini_video_output ("dir_color");

	int current_buffer = 0;
	vector<Mat> buffer;
	Mat average_flow = Mat::zeros(height,width,CV_32FC2);

	for ( int i = 0; i < buffer_size; i++ )
	{
		buffer.push_back(Mat::zeros(height,width,CV_32FC2));
	}

	Mat color_wheel = imread("colorWheel.jpg");
    resize(color_wheel, color_wheel, Size(height/8, height/8));

	ini_frame();

	for (int framecount = 1; true; ++framecount) {

		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		calcOpticalFlowFarneback(prev_frame, curr_frame, flow, 0.5, 2, 20, 3, 15, 1.2, OPTFLOW_FARNEBACK_GAUSSIAN);

		Mat out_img;
		resized_frame.copyTo(out_img);


		average_flow -= buffer[current_buffer] / static_cast<float>(buffer_size);
		buffer[current_buffer] = flow.clone();
		average_flow += buffer[current_buffer] / static_cast<float>(buffer_size);

		current_buffer++;
		if ( current_buffer >= buffer_size ) current_buffer = 0;

		vector_to_color (average_flow, out_img);
		drawFrameCount(out_img, framecount);
		
		
		imshow ("grid_buoy", out_img);
		video_output->write (out_img);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output->release();
	destroyAllWindows();

}

void fn_dir_color::run_dir (int buffer_size) {
	cout << "Running direction only color map" << endl;

	VideoWriter* video_output = ini_video_output ("dir_color");

	int current_buffer = 0;
	vector<Mat> buffer;
	Mat average_flow = Mat::zeros(height,width,CV_32FC2);

	for ( int i = 0; i < buffer_size; i++ )
	{
		buffer.push_back(Mat::zeros(height,width,CV_32FC2));
	}

	Mat color_wheel = imread("colorWheel.jpg");
    resize(color_wheel, color_wheel, Size(height/8, height/8));

	ini_frame();

	for (int framecount = 1; true; ++framecount) {

		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		calcOpticalFlowFarneback(prev_frame, curr_frame, flow, 0.5, 2, 20, 3, 15, 1.2, OPTFLOW_FARNEBACK_GAUSSIAN);

		Mat out_img;
		resized_frame.copyTo(out_img);


		average_flow -= buffer[current_buffer] / static_cast<float>(buffer_size);
		buffer[current_buffer] = flow.clone();
		average_flow += buffer[current_buffer] / static_cast<float>(buffer_size);

		current_buffer++;
		if ( current_buffer >= buffer_size ) current_buffer = 0;

		vector_to_dir_color (average_flow, out_img);
		drawFrameCount(out_img, framecount);

		
		
		imshow ("grid_buoy", out_img);
		video_output->write (out_img);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output->release();
	destroyAllWindows();

}