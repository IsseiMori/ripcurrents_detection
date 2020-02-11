#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include <fstream> 
#include <algorithm>

#include <opencv2/opencv.hpp>

#include "fn_shear.hpp"

using namespace std;

fn_shear::fn_shear (string _file_name, 
					int _height)
					: method(_file_name, _height) {
	max_displacement = 0.0;
	max_frobeniusNorm = 0.0;
}


void fn_shear::shearRateToColor(Mat& current, Mat& out_img, int offset, int framecount) {

	float max_frobeniusNorm_new = 0.0;

	float global_theta = 0;
	float global_magnitude = 0;

	
	/*
	ofstream outfile;
	String filenamecsv = "eigen.csv";


	if (framecount == 20) {
		filenamecsv = "eigen20.csv";
	}
	else if (framecount == 100) {
		filenamecsv = "eigen100.csv";
	}
	else if (framecount == 300) {
		filenamecsv = "eigen300.csv";
	}
	else if (framecount == 600) {
		filenamecsv = "eigen600.csv";
	}
	else if (framecount == 800) {
		filenamecsv = "eigen800.csv";
	}

	outfile.open(filenamecsv, std::ios::app);
	*/
	

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

			float dot_hor = right.x * left.x + right.y * left.y;
			float dot_ver = above.x * below.x + above.y * below.y;

			//float dot = sqrt(dot_hor * dot_hor + dot_ver * dot_ver);
            float dot = max(dot_hor, dot_ver);

			// Find the velocity gradient matrix
			/*
			/ | dvx/dx dvx/dy |
 			/ | dvy/dx dvy/dy |
			*/
			Mat jacobian = Mat_<float>(2,2);
			jacobian.at<float>(0,0) = right.x - left.x;
			jacobian.at<float>(0,1) = above.x - below.x;
			jacobian.at<float>(1,0) = right.y - left.y;
			jacobian.at<float>(1,1) = above.y - below.y;

			/*
			cout << " " << endl;
			cout << right << endl;
			cout << left << endl;
			cout << above << endl;
			cout << below << endl;
			*/


			// printf("%f\n",sqrt(jacobian.dot(jacobian)));

			
			Mat jacobianS = Mat_<float>(2,2);
			jacobianS.at<float>(0.0) = (jacobian.at<float>(0,0) + jacobian.at<float>(0,0)) / 2;
			jacobianS.at<float>(0,1) = (jacobian.at<float>(0,1) + jacobian.at<float>(1,0)) / 2;
			jacobianS.at<float>(1,0) = (jacobian.at<float>(1,0) + jacobian.at<float>(0,1)) / 2;
			jacobianS.at<float>(1,1) = (jacobian.at<float>(1,1) + jacobian.at<float>(1,1)) / 2;
			
            Mat e,v;
            eigen(jacobianS, e, v);
            float emax = max(abs(e.at<float>(0)), abs(e.at<float>(1)));
            float emin = min(abs(e.at<float>(0)), abs(e.at<float>(1)));
            float eratio = emax / emin;
			//float eratio = log(emax / emin) / log(1.01);

			float fa = (emax - emin) / sqrt(emax * emax + emin * emin);

			
			/*
			if (framecount == 800 || framecount == 20
				|| framecount == 100 || framecount == 300
				|| framecount == 600 || framecount == 800) {
				outfile << abs(right.x) << ","
						<< abs(right.y) << ","
						<< abs(left.x) << ","
						<< abs(left.y) << ","
						<< abs(above.x) << ","
						<< abs(above.y) << ","
						<< abs(below.x) << ","
						<< abs(below.y) << ","
						<< emax << ","
						<< emin << ","
						<< eratio << ","
						<< fa << endl;
			}
			*/

			//float frobeniusNorm = sqrt(sum(jacobian.mul(jacobian))[0]);
			float frobeniusNorm = jacobianS.at<float>(0,0) * jacobianS.at<float>(0,0)
							+ jacobianS.at<float>(0,1) * jacobianS.at<float>(0,1)
							+ jacobianS.at<float>(1,0) * jacobianS.at<float>(1,0)
							+ jacobianS.at<float>(1,1) * jacobianS.at<float>(1,1);
			frobeniusNorm = sqrt(frobeniusNorm);

			float theta = atan2(ptr->y, ptr->x)*180/M_PI;	// find angle
			theta += theta < 0 ? 360 : 0;	// enforce strict positive angle
			
			// store vector data
			// ptr2->x = 128 - dot*128/max_frobeniusNorm;
			// ptr2->x = 128 - frobeniusNorm*128/max_frobeniusNorm;
            // ptr2->x = 128 - emax*128/max_frobeniusNorm;
			// ptr2->x = 128 - eratio*128/max_frobeniusNorm;
			
			// High anisotropy only
			/*
			if (fa > 0.9) {
				ptr2->x = 0;
				ptr2->y = 0;
            	ptr2->z = 255;
			}
			*/
			
			// High e1
			if (emax > 1) {
				ptr2->x = 0;
				ptr2->y = 0;
            	ptr2->z = 255;
			}
			
			// ptr2->x = 128 - fa*128/max_frobeniusNorm;

			// 0 is red

			// high eratio
			/*
			if ( eratio >= 30) {
				ptr2->x = 0;
				ptr2->y = 0;
            	ptr2->z = 255;
			}
			*/

			// Low e2
			// if (emin < 0.08) ptr2->x = 120;
			// else ptr2->x = 0;

			// x hue 0-180 red to red
			// y saturation 0-255
			// z value 0-255
			// ptr2->x = 120;
			// ptr2->y = 255;
            // ptr2->z = 255;
			// ptr2->z = 255;
			//if ( ptr2->z < 30 ) ptr2->z = 0;

			// store the previous max to maxmin next frame
			if ( sqrt(ptr->x * ptr->x + ptr->y * ptr->y) > max_displacement ) max_displacement = sqrt(ptr->x * ptr->x + ptr->y * ptr->y);
			//max_frobeniusNorm_new = max(frobeniusNorm, max_frobeniusNorm_new);
			//max_frobeniusNorm_new = max(dot, max_frobeniusNorm_new);
            // max_frobeniusNorm_new = max(eratio, max_frobeniusNorm_new);
			max_frobeniusNorm_new = max(fa, max_frobeniusNorm_new);
	
			global_theta += ptr2->x * ptr2->z;
			global_magnitude += ptr2->z;


			ptr++;
			ptr2++;
		}
	}

	max_frobeniusNorm = max_frobeniusNorm_new;

	// show as hsv format
	// cvtColor(out_img, out_img, COLOR_HSV2BGR);

	// outfile.close();
}

void fn_shear::shearKernelToColor(Mat& current, Mat& out_img, int offset, int framecount) {

	float max_dot_new = 0;
	float min_dot_new = 0;

	// Iterate through all pixels except for the very edge
	for ( int row = offset; row < current.rows - offset; row++ ) {
		Pixel2* ptr = current.ptr<Pixel2>(row, offset);
		Pixelc* ptr2 = out_img.ptr<Pixelc>(row, offset);

		for ( int col = offset; col < current.cols - offset; col++ ) {

			/*
			 *  |a b c|
			 *  |d e f|
			 *  |g e i|
			 * 
			 * 	dot = [dot(a,e) + dot(b,e) + ... dot(i,e)] / 8
			*/
			Pixel2 center_ptr = current.at<Pixel2>(row, col);
			float dot_sum = 0;
			for (int i = row - offset; i <= row + offset; ++i) {
				for (int j = col - offset; j <= col + offset; ++j) {
					if (i != row && j != col) {
						Pixel2 n_ptr = current.at<Pixel2>(i, j);
						// cout << n_ptr.dot(center_ptr) << endl;
						dot_sum += n_ptr.dot(center_ptr);
					}
				}
			}
			float dot = dot_sum / static_cast<float>((offset + 1) * (offset + 1) - 1);

			max_dot_new = max(max_dot_new, dot);
			min_dot_new = min(min_dot_new, dot);

			float range = max_dot - min_dot;

			float h = 128 * (max_dot - dot) / range;
			if (h < 0) h = 0;
			if (h > 128) h = 128;
			ptr2->x = h;
			ptr2->y = 255;
			ptr2->z = 255;


			ptr++;
			ptr2++;
		}
	}

	max_dot = max_dot_new;
	min_dot = min_dot_new;

	// show as hsv format
	cvtColor(out_img, out_img, COLOR_HSV2BGR);
}

void fn_shear::run (int buffer_size, int offset, bool isNorm) {
	cout << "Running shear color map" << endl;

	string n_str = isNorm? "norm_" : "";

	VideoWriter* video_output_color = ini_video_output (file_name + "_shear_" + n_str + to_string(buffer_size) + "_" + to_string(offset) + "_color");
	VideoWriter* video_output_overlay = ini_video_output (file_name + "_shear_" + n_str + to_string(buffer_size) + "_" + to_string(offset) + "_overlay");

	ini_frame();
	ini_buffer(buffer_size);

	Mat color_wheel = imread("colorChart.jpg");
    resize(color_wheel, color_wheel, Size(height/8, height/8));


	for (int framecount = 1; true; ++framecount) {

		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		calc_FB ();

		Mat out_img;
		Mat out_img_overlay;
		resized_frame.copyTo(out_img);
		resized_frame.copyTo(out_img_overlay);

		eliminate_std(5);

		if (isNorm) {
			normalize_flow();
		}

		update_buffer (buffer_size);

		//shearRateToColor (average_flow, out_img, offset, framecount);
		shearKernelToColor (average_flow, out_img, offset, framecount);
		//vector_to_color (average_flow, out_img_overlay);

		drawFrameCount(out_img, framecount);


		addWeighted( out_img, 0.4, out_img_overlay, 0.6, 0.0, out_img_overlay);

		// Draw color wheel
		/*
        Mat mat = (Mat_<double>(2,3)<<1.0, 0.0, width - height/8, 0.0, 1.0, 0);
        warpAffine(color_wheel, out_img, mat, out_img.size(), INTER_LINEAR, cv::BORDER_TRANSPARENT);
		warpAffine(color_wheel, out_img_overlay, mat, out_img_overlay.size(), INTER_LINEAR, cv::BORDER_TRANSPARENT);
		*/
		
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