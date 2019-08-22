#include <string>
#include <iostream>
#include <math.h>

#include <opencv2/opencv.hpp>

#include "fn_timex.hpp"

using namespace std;

fn_timex::fn_timex (string _file_name, 
					int _height)
					: method(_file_name, _height) {
}

void fn_timex::run (int buffer_size) {
	cout << "Running timex " << endl;

	VideoWriter* video_output = ini_video_output (file_name + "_timex_" + to_string(buffer_size));

	Mat sum_rgb = Mat::zeros(height,width,CV_32FC3);

	ini_frame ();

	for( int framecount = 1; buffer_size == 0 || framecount <= buffer_size; ++framecount){

		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		Mat rgb = Mat::zeros(height,width,CV_32FC3);
		resized_frame.convertTo(rgb, CV_32FC3);

		// get new buffer
		sum_rgb += rgb;

		Mat average_rgb = Mat::zeros(height,width,CV_32FC3);

		average_rgb = sum_rgb / framecount;

		Mat out_img;

		average_rgb.convertTo(out_img, CV_8UC3);

		drawFrameCount(out_img, framecount);

		imshow ("timex",out_img);
		video_output->write (out_img);

		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output->release();
	destroyAllWindows();

}