#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include <fstream> 

#include <opencv2/opencv.hpp>

#include "fn_dir_color.hpp"

using namespace std;

fn_dir_color::fn_dir_color (string _file_name, 
							int _height)
							: method(_file_name, _height) {
}

void fn_dir_color::run (int buffer_size) {
	cout << "Running color map" << endl;

	VideoWriter* video_output_color = ini_video_output (file_name + "_dir_color_" + to_string(buffer_size) + "_color");
	VideoWriter* video_output_overlay = ini_video_output (file_name + "_dir_color_" + to_string(buffer_size) + "_overlay");

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
		Mat out_img_overlay;
		resized_frame.copyTo(out_img);
		resized_frame.copyTo(out_img_overlay);


		average_flow -= buffer[current_buffer] / static_cast<float>(buffer_size);
		buffer[current_buffer] = flow.clone();
		average_flow += buffer[current_buffer] / static_cast<float>(buffer_size);

		current_buffer++;
		if ( current_buffer >= buffer_size ) current_buffer = 0;

		vector_to_color (average_flow, out_img);
		drawFrameCount(out_img, framecount);


		addWeighted( out_img, 0.6, out_img_overlay, 0.4, 0.0, out_img_overlay);

		// Draw color wheel
        Mat mat = (Mat_<double>(2,3)<<1.0, 0.0, width - height/8, 0.0, 1.0, 0);
        warpAffine(color_wheel, out_img, mat, out_img.size(), INTER_LINEAR, cv::BORDER_TRANSPARENT);
		warpAffine(color_wheel, out_img_overlay, mat, out_img_overlay.size(), INTER_LINEAR, cv::BORDER_TRANSPARENT);
		
		
		imshow ("grid_buoy color map", out_img);
		imshow ("grid_buoy overlay", out_img_overlay);
		video_output_color->write (out_img);
		video_output_overlay->write (out_img_overlay);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output_color->release();
	video_output_overlay->release();
	destroyAllWindows();

}

void fn_dir_color::run_dir (int buffer_size) {
	cout << "Running direction only color map" << endl;

	VideoWriter* video_output_color = ini_video_output (file_name + "_dir_only_color_" + to_string(buffer_size) + "_color");
	VideoWriter* video_output_overlay = ini_video_output (file_name + "_dir_only_color_" + to_string(buffer_size) + "_overlay");

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
		Mat out_img_overlay;
		resized_frame.copyTo(out_img);
		resized_frame.copyTo(out_img_overlay);


		average_flow -= buffer[current_buffer] / static_cast<float>(buffer_size);
		buffer[current_buffer] = flow.clone();
		average_flow += buffer[current_buffer] / static_cast<float>(buffer_size);

		current_buffer++;
		if ( current_buffer >= buffer_size ) current_buffer = 0;

		vector_to_dir_color (average_flow, out_img);
		drawFrameCount(out_img, framecount);

		addWeighted( out_img, 0.6, out_img_overlay, 0.4, 0.0, out_img_overlay);	

		// Draw color wheel
        Mat mat = (Mat_<double>(2,3)<<1.0, 0.0, width - height/8, 0.0, 1.0, 0);
        warpAffine(color_wheel, out_img, mat, out_img.size(), INTER_LINEAR, cv::BORDER_TRANSPARENT);
		warpAffine(color_wheel, out_img_overlay, mat, out_img_overlay.size(), INTER_LINEAR, cv::BORDER_TRANSPARENT);	
		
		imshow ("grid_buoy color map", out_img);
		imshow ("grid_buoy overlay", out_img_overlay);
		video_output_color->write (out_img);
		video_output_overlay->write (out_img_overlay);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output_color->release();
	video_output_overlay->release();
	destroyAllWindows();

}