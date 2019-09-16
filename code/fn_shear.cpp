#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include <fstream> 

#include <opencv2/opencv.hpp>

#include "fn_shear.hpp"

using namespace std;

fn_shear::fn_shear (string _file_name, 
					int _height)
					: method(_file_name, _height) {
	max_displacement = 0.0;
	max_frobeniusNorm = 0.0;
}


void fn_shear::shearRateToColor(Mat& current, Mat& out_img) {

	float max_frobeniusNorm_new = 0.0;

	float global_theta = 0;
	float global_magnitude = 0;

	int offset = 10;

	// Iterate through all pixels except for the very edge
	for ( int row = offset; row < current.rows - offset; row++ ) {
		Pixel2* ptr = current.ptr<Pixel2>(row, offset);
		Pixelc* ptr2 = out_img.ptr<Pixelc>(row, offset);

		for ( int col = offset; col < current.cols - offset; col++ ) {

			// obtain the neighbor vectors
			Pixel2 above = current.at<Pixel2>(row-offset, col);
			Pixel2 below = current.at<Pixel2>(row+offset, col);
			Pixel2 left = current.at<Pixel2>(row, col-offset);
			Pixel2 right = current.at<Pixel2>(row, col+offset);

			// Find the velocity gradient matrix
			/*
			/ | dvx/dx dvx/dy |
 			/ | dvy/dx dvy/dy |
			*/
			Mat jacobian = Mat_<Pixel2>(2,2);
			jacobian.at<float>(0,0) = right.x - left.x;
			jacobian.at<float>(0,1) = above.x - below.x;
			jacobian.at<float>(1,0) = right.y - left.y;
			jacobian.at<float>(1,1) = above.y - below.y;

			// printf("%f\n",sqrt(jacobian.dot(jacobian)));

			/*
			Mat jacobianS = Mat_<Pixel2>(2,2);
			jacobianS.at<float>(0.0) = (jacobian.at<float>(0,0) + jacobian.at<float>(0,0)) / 2;
			jacobianS.at<float>(0,1) = (jacobian.at<float>(0,1) + jacobian.at<float>(1,0)) / 2;
			jacobianS.at<float>(1,0) = (jacobian.at<float>(1,0) + jacobian.at<float>(0,1)) / 2;
			jacobianS.at<float>(1,1) = (jacobian.at<float>(1,1) + jacobian.at<float>(1,1)) / 2;
			*/

			//float frobeniusNorm = sqrt(sum(jacobian.mul(jacobian))[0]);
			float frobeniusNorm = jacobian.at<float>(0,0) * jacobian.at<float>(0,0)
							+ jacobian.at<float>(0,1) * jacobian.at<float>(0,1)
							+ jacobian.at<float>(1,0) * jacobian.at<float>(1,0)
							+ jacobian.at<float>(1,1) * jacobian.at<float>(1,1);
			frobeniusNorm = sqrt(frobeniusNorm);

			float theta = atan2(ptr->y, ptr->x)*180/M_PI;	// find angle
			theta += theta < 0 ? 360 : 0;	// enforce strict positive angle
			
			// store vector data
			ptr2->x = 128 - frobeniusNorm*128/max_frobeniusNorm;
			ptr2->y = 255;
            ptr2->z = 255;
			// ptr2->z = 255;
			//if ( ptr2->z < 30 ) ptr2->z = 0;

			// store the previous max to maxmin next frame
			if ( sqrt(ptr->x * ptr->x + ptr->y * ptr->y) > max_displacement ) max_displacement = sqrt(ptr->x * ptr->x + ptr->y * ptr->y);
			max_frobeniusNorm_new = max(frobeniusNorm, max_frobeniusNorm_new);
	
			global_theta += ptr2->x * ptr2->z;
			global_magnitude += ptr2->z;


			ptr++;
			ptr2++;
		}
	}

	max_frobeniusNorm = max_frobeniusNorm_new;

	// show as hsv format
	cvtColor(out_img, out_img, COLOR_HSV2BGR);
}

void fn_shear::run (int buffer_size) {
	cout << "Running shear color map" << endl;

	VideoWriter* video_output_color = ini_video_output (file_name + "_shear_" + to_string(buffer_size) + "_color");
	VideoWriter* video_output_overlay = ini_video_output (file_name + "_shear_" + to_string(buffer_size) + "_overlay");

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


		// recommended 5 5 1.1
		// recommended 5 7 1.5
		// no banding 20 (3) 15 1.2
		calcOpticalFlowFarneback(prev_frame, curr_frame, flow, 0.5, 2, 20, 3, 15, 1.2, OPTFLOW_FARNEBACK_GAUSSIAN);
		// calcOpticalFlowFarneback(prev_frame, curr_frame, flow, 0.5, 2, 5, 3, 5, 1.1, OPTFLOW_FARNEBACK_GAUSSIAN);

		Mat out_img;
		Mat out_img_overlay;
		resized_frame.copyTo(out_img);
		resized_frame.copyTo(out_img_overlay);


		float mean = 0;
		float std = 0;
		int n = 0;
		float diff_sum = 0;

		flow.forEach<Pixel2>([&](Pixel2& px, const int pos[]) -> void {
			mean += sqrt(px.x * px.x + px.y*px.y);
			n++;
		});

		mean = mean / n;

		flow.forEach<Pixel2>([&](Pixel2& px, const int pos[]) -> void {
			diff_sum += pow(mean - sqrt(px.x * px.x + px.y*px.y),2);
		});

		std = sqrt(diff_sum / (n-1));

		flow.forEach<Pixel2>([&](Pixel2& px, const int pos[]) -> void {

			float theta = atan2 (px.y, px.x);

			if ( sqrt(px.x * px.x + px.y*px.y) - mean > std * 5) {
			// if ( abs(sqrt(px.x * px.x + px.y*px.y) - mean) > std * 3) {	
				px.x = 0;
				px.y = 0;
			}
			

			px.x = cos(theta);
			px.y = sin(theta);

		});



		average_flow -= buffer[current_buffer] / static_cast<float>(buffer_size);
		buffer[current_buffer] = flow.clone();
		average_flow += buffer[current_buffer] / static_cast<float>(buffer_size);

		current_buffer++;
		if ( current_buffer >= buffer_size ) current_buffer = 0;

		shearRateToColor (average_flow, out_img);
		//vector_to_color (average_flow, out_img_overlay);

		drawFrameCount(out_img, framecount);


		addWeighted( out_img, 0.6, out_img_overlay, 0.4, 0.0, out_img_overlay);

		// Draw color wheel
        Mat mat = (Mat_<double>(2,3)<<1.0, 0.0, width - height/8, 0.0, 1.0, 0);
        warpAffine(color_wheel, out_img, mat, out_img.size(), INTER_LINEAR, cv::BORDER_TRANSPARENT);
		warpAffine(color_wheel, out_img_overlay, mat, out_img_overlay.size(), INTER_LINEAR, cv::BORDER_TRANSPARENT);
		
		
		imshow ("shear color map", out_img);
		imshow ("shear overlay", out_img_overlay);
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