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
	max_e1 = 0;
	min_e1 = 0;
}

void fn_shear::shearRateToColor(Mat& current, Mat& out_img, int offset, int framecount) {

	float max_frobeniusNorm_new = 0.0;

	float global_theta = 0;
	float global_magnitude = 0;

	float max_e1_new = 0;
	float min_e1_new = 0;

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

	outfile.open(filenamecsv, std::ios::app);*/
	

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
			}*/

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

			float e1_range = max_e1 - min_e1;

			// High e1
			if (emax > 0.2) {
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
	
			// global_theta += ptr2->x * ptr2->z;
			// global_magnitude += ptr2->z;

			max_e1_new = max(max_e1_new, emax);
			min_e1_new = min(min_e1_new, emax);


			ptr++;
			ptr2++;
		}
	}

	max_frobeniusNorm = max_frobeniusNorm_new;


	max_e1 = max_e1_new;
	min_e1 = min_e1_new;

	// show as hsv format
	// cvtColor(out_img, out_img, COLOR_HSV2BGR);

	// outfile.close();
}

float top3 = 0;
float top5 = 0;
float top8 = 0;
float top10 = 0;
float top20 = 0;
float top30 = 0;
float top40 = 0;
float top50 = 0;

void fn_shear::shearRateToColorMulti(Mat& current, Mat& out_img1,Mat& out_img2,Mat& out_img3,Mat& out_img4,Mat& out_img5,Mat& out_img6, int offset, int framecount) {

	float max_frobeniusNorm_new = 0.0;

	float global_theta = 0;
	float global_magnitude = 0;

	float max_e1_new = 0;
	float min_e1_new = 0;


	ofstream outfile;
	String filenamecsv = file_name + "eigen.csv";


	if (framecount == 20) {
		filenamecsv = file_name + "eigen20.csv";
	}
	else if (framecount == 100) {
		filenamecsv = file_name + "eigen100.csv";
	}
	else if (framecount == 300) {
		filenamecsv = file_name + "eigen300.csv";
	}
	else if (framecount == 600) {
		filenamecsv = file_name + "eigen600.csv";
	}
	else if (framecount == 800) {
		filenamecsv = file_name + "eigen800.csv";
	}

	outfile.open(filenamecsv, std::ios::app);

	vector<float> emax_vec;

	// Iterate through all pixels except for the very edge
	for ( int row = offset; row < current.rows - offset; row++ ) {
		Pixel2* ptr = current.ptr<Pixel2>(row, offset);
		Pixelc* ptr21 = out_img1.ptr<Pixelc>(row, offset);
		Pixelc* ptr22 = out_img2.ptr<Pixelc>(row, offset);
		Pixelc* ptr23 = out_img3.ptr<Pixelc>(row, offset);
		Pixelc* ptr24 = out_img4.ptr<Pixelc>(row, offset);
		Pixelc* ptr25 = out_img5.ptr<Pixelc>(row, offset);
		Pixelc* ptr26 = out_img6.ptr<Pixelc>(row, offset);

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

			emax_vec.push_back(emax);

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

			float e1_range = max_e1 - min_e1;

			// High e1
			/*
			if (emax > 0.2) {
				ptr21->x = 0;
				ptr21->y = 0;
            	ptr21->z = 255;
			}
			*/

			/*
			// High e1 top 5%
			if (emax > top3) {
				ptr21->x = 0;
				ptr21->y = 0;
            	ptr21->z = 255;
			}

			// High e1 top 10%
			if (emax > top5) {
				ptr22->x = 0;
				ptr22->y = 0;
            	ptr22->z = 255;
			}

			// High e1 top 20%
			if (emax > top8) {
				ptr23->x = 0;
				ptr23->y = 0;
            	ptr23->z = 255;
			}

			// High e1 top 30%
			if (emax > top10) {
				ptr24->x = 0;
				ptr24->y = 0;
            	ptr24->z = 255;
			}

			// High e1 top 40%
			if (emax > top20) {
				ptr25->x = 0;
				ptr25->y = 0;
            	ptr25->z = 255;
			}

			// High e1 top 50%
			if (emax > top30) {
				ptr26->x = 0;
				ptr26->y = 0;
            	ptr26->z = 255;
			}*/

			// High e1 top 5% red
			if (emax > top3) {
				ptr21->x = 0;
				ptr21->y = 0;
            	ptr21->z = 255;
			}

			// High e1 top 10% orenge
			else if (emax > top5) {
				ptr21->x = 0;
				ptr21->y = 128;
            	ptr21->z = 255;
			}

			// High e1 top 20% yellow
			else if (emax > top8) {
				ptr21->x = 0;
				ptr21->y = 255;
            	ptr21->z = 255;
			}

			// High e1 top 30%
			else if (emax > top10) {
				ptr21->x = 0;
				ptr21->y = 255;
            	ptr21->z = 0;
			}

			// High e1 top 40% green
			else if (emax > top20) {
				ptr21->x = 100;
				ptr21->y = 255;
            	ptr21->z = 0;
			}

			// High e1 top 50% light blue green
			else if (emax > top30) {
				ptr21->x = 200;
				ptr21->y = 255;
            	ptr21->z = 0;
			}

			// High e1 top 50% light blue
			else if (emax > top40) {
				ptr21->x = 255;
				ptr21->y = 255;
            	ptr21->z = 0;
			}

			// High e1 top 50% Blue
			else if (emax > top50) {
				ptr21->x = 255;
				ptr21->y = 128;
            	ptr21->z = 0;
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
	
			// global_theta += ptr2->x * ptr2->z;
			// global_magnitude += ptr2->z;

			max_e1_new = max(max_e1_new, emax);
			min_e1_new = min(min_e1_new, emax);

			ptr++;
			ptr21++;
			ptr22++;
			ptr23++;
			ptr24++;
			ptr25++;
			ptr26++;
		}
	}

	max_frobeniusNorm = max_frobeniusNorm_new;

	sort(emax_vec.begin(), emax_vec.end());

	float all_count = (current.rows - 2 * offset) * (current.cols - 2 * offset);
	top3  = emax_vec[static_cast<int>(all_count * 0.98)];
	top5  = emax_vec[static_cast<int>(all_count * 0.95)];
	top8  = emax_vec[static_cast<int>(all_count * 0.92)];
	top10 = emax_vec[static_cast<int>(all_count * 0.9)];
	top20 = emax_vec[static_cast<int>(all_count * 0.8)];
	top30 = emax_vec[static_cast<int>(all_count * 0.7)];
	top40 = emax_vec[static_cast<int>(all_count * 0.6)];
	top50 = emax_vec[static_cast<int>(all_count * 0.5)];

	cout << "top  3% : " << top3  << " < " << endl;
	cout << "top  5% : " << top5  << " < " << endl;
	cout << "top  8% : " << top8  << " < " << endl;
	cout << "top 10% : " << top10 << " < " << endl;
	cout << "top 20% : " << top20 << " < " << endl;
	cout << "top 30% : " << top30 << " < " << endl;

	max_e1 = max_e1_new;
	min_e1 = min_e1_new;

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

	VideoWriter* video_output_overlay1 = ini_video_output (file_name + "_shear_" + n_str + to_string(buffer_size) + "_" + to_string(offset) + "_overlay1");
	VideoWriter* video_output_overlay2 = ini_video_output (file_name + "_shear_" + n_str + to_string(buffer_size) + "_" + to_string(offset) + "_overlay2");
	VideoWriter* video_output_overlay3 = ini_video_output (file_name + "_shear_" + n_str + to_string(buffer_size) + "_" + to_string(offset) + "_overlay3");
	VideoWriter* video_output_overlay4 = ini_video_output (file_name + "_shear_" + n_str + to_string(buffer_size) + "_" + to_string(offset) + "_overlay4");
	VideoWriter* video_output_overlay5 = ini_video_output (file_name + "_shear_" + n_str + to_string(buffer_size) + "_" + to_string(offset) + "_overlay5");
	VideoWriter* video_output_overlay6 = ini_video_output (file_name + "_shear_" + n_str + to_string(buffer_size) + "_" + to_string(offset) + "_overlay6");

	ini_frame();
	ini_buffer(buffer_size);

	Mat color_wheel = imread("colorChart.jpg");
    resize(color_wheel, color_wheel, Size(height/8, height/8));


	for (int framecount = 1; true; ++framecount) {

		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		calc_FB ();

		Mat out_img1;
		Mat out_img2;
		Mat out_img3;
		Mat out_img4;
		Mat out_img5;
		Mat out_img6;
		Mat out_img_overlay1;
		Mat out_img_overlay2;
		Mat out_img_overlay3;
		Mat out_img_overlay4;
		Mat out_img_overlay5;
		Mat out_img_overlay6;
		resized_frame.copyTo(out_img1);
		resized_frame.copyTo(out_img2);
		resized_frame.copyTo(out_img3);
		resized_frame.copyTo(out_img4);
		resized_frame.copyTo(out_img5);
		resized_frame.copyTo(out_img6);
		resized_frame.copyTo(out_img_overlay1);
		resized_frame.copyTo(out_img_overlay2);
		resized_frame.copyTo(out_img_overlay3);
		resized_frame.copyTo(out_img_overlay4);
		resized_frame.copyTo(out_img_overlay5);
		resized_frame.copyTo(out_img_overlay6);

		eliminate_std(5);

		if (isNorm) {
			normalize_flow();
		}

		update_buffer (buffer_size);

		shearRateToColorMulti (average_flow, out_img1, out_img2, out_img3, out_img4, out_img5, out_img6, offset, framecount);
		// shearKernelToColor (average_flow, out_img, offset, framecount);
		//vector_to_color (average_flow, out_img_overlay);

		drawFrameCount(out_img1, framecount);
		drawFrameCount(out_img2, framecount);
		drawFrameCount(out_img3, framecount);
		drawFrameCount(out_img4, framecount);
		drawFrameCount(out_img5, framecount);
		drawFrameCount(out_img6, framecount);


		addWeighted( out_img1, 0.4, out_img_overlay1, 0.6, 0.0, out_img_overlay1);
		addWeighted( out_img2, 0.4, out_img_overlay2, 0.6, 0.0, out_img_overlay2);
		addWeighted( out_img3, 0.4, out_img_overlay3, 0.6, 0.0, out_img_overlay3);
		addWeighted( out_img4, 0.4, out_img_overlay4, 0.6, 0.0, out_img_overlay4);
		addWeighted( out_img5, 0.4, out_img_overlay5, 0.6, 0.0, out_img_overlay5);
		addWeighted( out_img6, 0.4, out_img_overlay6, 0.6, 0.0, out_img_overlay6);

		// Draw color wheel
		/*
        Mat mat = (Mat_<double>(2,3)<<1.0, 0.0, width - height/8, 0.0, 1.0, 0);
        warpAffine(color_wheel, out_img, mat, out_img.size(), INTER_LINEAR, cv::BORDER_TRANSPARENT);
		warpAffine(color_wheel, out_img_overlay, mat, out_img_overlay.size(), INTER_LINEAR, cv::BORDER_TRANSPARENT);
		*/
		
		imshow ("shear overlay1", out_img_overlay1);
		imshow ("shear overlay2", out_img_overlay2);
		imshow ("shear overlay3", out_img_overlay3);
		imshow ("shear overlay4", out_img_overlay4);
		imshow ("shear overlay5", out_img_overlay5);
		imshow ("shear overlay6", out_img_overlay6);
		video_output_overlay1->write (out_img_overlay1);
		video_output_overlay2->write (out_img_overlay2);
		video_output_overlay3->write (out_img_overlay3);
		video_output_overlay4->write (out_img_overlay4);
		video_output_overlay5->write (out_img_overlay5);
		video_output_overlay6->write (out_img_overlay6);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output_overlay1->release();
	video_output_overlay2->release();
	video_output_overlay3->release();
	video_output_overlay4->release();
	video_output_overlay5->release();
	video_output_overlay6->release();
	destroyAllWindows();

}