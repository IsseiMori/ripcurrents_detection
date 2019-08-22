#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "method.hpp"

using namespace std;

method::method (string _file_name, 
				int _height) {

	file_name = _file_name;

	video = cv::VideoCapture(file_name);
	if (!video.isOpened())
	{
		cout << file_name << " File not found" << endl;
		exit(1);
	}

	fps = video.get(CAP_PROP_FPS);

	height = _height;
	width = floor(video.get(cv::CAP_PROP_FRAME_WIDTH) * 
			height / video.get(cv::CAP_PROP_FRAME_HEIGHT));

}

VideoWriter* method::ini_video_output (string video_name) {
	
	VideoWriter* video_output = new VideoWriter (video_name + ".mp4", 
				  0x7634706d, 
				  fps, cv::Size(width,height),true);
	
	if (!video_output->isOpened())
	{
		cout << "!!! Output video could not be opened" << endl;
		return NULL;
	}

	return video_output;

}

void method::vector_to_color(Mat& curr, Mat& out_img) {

	static float max_displacement = 0;
	float max_displacement_new = 0;

	float global_theta = 0;
	float global_magnitude = 0;

	for ( int row = 0; row < curr.rows; row++ ) {
		Pixel2* ptr = curr.ptr<Pixel2>(row, 0);
		Pixelc* ptr2 = out_img.ptr<Pixelc>(row, 0);

		for ( int col = 0; col < curr.cols; col++ ) {
			float theta = atan2(ptr->y, ptr->x)*180/M_PI;	// find angle
			theta += theta < 0 ? 360 : 0;	// enforce strict positive angle
			
			// store vector data
			ptr2->x = theta / 2;
			ptr2->y = 255;
			//ptr2->z = sqrt(ptr->x * ptr->x + ptr->y * ptr->y)*128/max_displacement+128;
            ptr2->z = sqrt(ptr->x * ptr->x + ptr->y * ptr->y)*255/max_displacement;
			//if ( ptr2->z < 30 ) ptr2->z = 0;

			// store the previous max to maxmin next frame
			if ( sqrt(ptr->x * ptr->x + ptr->y * ptr->y) > max_displacement_new ) max_displacement_new = sqrt(ptr->x * ptr->x + ptr->y * ptr->y);

			global_theta += ptr2->x * ptr2->z;
			global_magnitude += ptr2->z;


			ptr++;
			ptr2++;
		}
	}

	max_displacement = max_displacement_new;

	// show as hsv format
	cvtColor(out_img, out_img, COLOR_HSV2BGR);
}

void method::vector_to_dir_color(Mat& curr, Mat& out_img) {

	static float max_displacement = 0;
	float max_displacement_new = 0;

	float global_theta = 0;
	float global_magnitude = 0;

	for ( int row = 0; row < curr.rows; row++ ) {
		Pixel2* ptr = curr.ptr<Pixel2>(row, 0);
		Pixelc* ptr2 = out_img.ptr<Pixelc>(row, 0);

		for ( int col = 0; col < curr.cols; col++ ) {
			float theta = atan2(ptr->y, ptr->x)*180/M_PI;	// find angle
			theta += theta < 0 ? 360 : 0;	// enforce strict positive angle
			
			// store vector data
			ptr2->x = theta / 2;
			ptr2->y = 255;
			ptr2->z = 255;
			//if ( ptr2->z < 30 ) ptr2->z = 0;

			// store the previous max to maxmin next frame
			if ( sqrt(ptr->x * ptr->x + ptr->y * ptr->y) > max_displacement_new ) max_displacement_new = sqrt(ptr->x * ptr->x + ptr->y * ptr->y);

			global_theta += ptr2->x * ptr2->z;
			global_magnitude += ptr2->z;


			ptr++;
			ptr2++;
		}
	}

	max_displacement = max_displacement_new;

	// show as hsv format
	cvtColor(out_img, out_img, COLOR_HSV2BGR);
}

// Load the previous frame
// Named ini_frame because calling read_frame twice is confusing
int method::ini_frame () {
	return read_frame();
}

int method::read_frame () {

	Mat frame, grayscaled_frame;

	curr_frame.copyTo (prev_frame);
	video.read (frame);
	if (frame.empty()) return 1;
	resize (frame, resized_frame, Size(width, height), 0, 0, INTER_LINEAR);
	cvtColor (resized_frame, grayscaled_frame, COLOR_BGR2GRAY);
	grayscaled_frame.copyTo(curr_frame);
	return 0;
}

void method::drawFrameCount (Mat& outImg, int framecount) {
	putText(outImg, to_string(framecount), Point(30,30), 
	FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(250,250,250), 1, false);
}