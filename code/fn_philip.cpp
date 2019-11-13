#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include <fstream> 
#include <time.h>

#include <opencv2/opencv.hpp>

#include "fn_philip.hpp"

using namespace std;

fn_philip::fn_philip (string _file_name, 
							int _height)
							: method(_file_name, _height) {
}

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

bool compareContourAreas ( vector<Point> contour1, vector<Point> contour2 ) {
	double i = fabs( contourArea(Mat(contour1)) );
	double j = fabs( contourArea(Mat(contour2)) );
	return ( i > j );
}

void fn_philip::run (int buffer_size, int angle1, int angle2) {
	cout << "Philip (2016)" << endl;

	VideoWriter* video_output_overlay = ini_video_output (file_name + "_philip");

	ofstream outfile;
	outfile.open(file_name+"bbs.txt");

	ini_frame();

	vector<Mat> red_buffer_vec;
	Mat red_buffer;

	current_buffer = 0;
	red_buffer = Mat::zeros(height, width, CV_8UC3);
	for (int i = 0; i < buffer_size; ++i) {
		red_buffer_vec.push_back (Mat::zeros(height, width, CV_8UC3));
	}

	int hist[HIST_BINS] = {0}; //histogram
	int histsum = 0;
	float UPPER = 100.0; //UPPER can be determined programmatically
	
	int hist2d[HIST_DIRECTIONS][HIST_BINS] = {{0}};
	int histsum2d[HIST_DIRECTIONS] = {0};
	float UPPER2d[HIST_DIRECTIONS] = {0};
	float prop_above_upper[HIST_DIRECTIONS] = {0};

	for (int framecount = 1; true; ++framecount) {

		// cout << "framecount : " << framecount << endl;

		if (read_frame()) break;

		//calcOpticalFlowFarneback(prev_frame, curr_frame, flow, 0.5, 2, 20, 3, 15, 1.2, OPTFLOW_FARNEBACK_GAUSSIAN);
		calcOpticalFlowFarneback(prev_frame, curr_frame, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
		Mat current = flow.clone();
		Mat red = Mat::zeros(height, width, CV_8UC3);
		histogram(current, red, angle1, angle2);

		red_buffer -= red_buffer_vec[current_buffer];
		red_buffer_vec[current_buffer] = red;
		red_buffer += red;

		Mat red_thresh = red_buffer.clone();

		for ( int row = 0; row < red.rows; row++ ) {
			Pixelc* ptr = red_thresh.ptr<Pixelc>(row, 0);
			for ( int col = 0; col < red_thresh.cols; col++ ) {
				float r = ptr->z;
				if (framecount < buffer_size) {
					if (r < framecount / 2.0) ptr->z = 0;
					else ptr->z = r / static_cast<float>(framecount) * 255;
				}
				else {
					if (r < buffer_size / 2.0) ptr->z = 0;
					else ptr->z = r / static_cast<float>(buffer_size) * 255;
				}

				ptr++;
			}
		}

		Mat red_gray;
		Mat red_blur;
		bilateralFilter(red_thresh, red_blur, 15, 80, 80);
		cvtColor(red_blur, red_gray, COLOR_BGR2GRAY);
		threshold(red_gray, red_gray, 0, 255, THRESH_BINARY);

		vector<vector<Point>> contours;
		Mat contourOutput = red_gray.clone();
		findContours( contourOutput, contours, RETR_LIST, CHAIN_APPROX_NONE );

		Mat contourImage(red_thresh.size(), CV_8UC3, Scalar(0,0,0));
		Scalar color = Scalar(0, 0, 255);

		sort(contours.begin(), contours.end(), compareContourAreas);
		
		outfile << framecount << "";
		for (size_t idx = 0; idx < 2; ++idx) {
			if (contours.size() > idx) {
				drawContours(contourImage, contours, idx, color, FILLED);
				outfile << ","
						<< boundingRect(contours[idx]).tl().x << ","
					    << boundingRect(contours[idx]).tl().y << "," 
						<< boundingRect(contours[idx]).br().x << ","
						<< boundingRect(contours[idx]).br().y;
			}
		}
		outfile << endl;

		current_buffer++;
		if ( current_buffer >= buffer_size ) current_buffer = 0;

		Mat out_img;
		Mat out_color;
		Mat out_overlay;
		resized_frame.copyTo(out_color);

		vector_to_dir_color(flow, out_color);
		// vector_to_dir_color(current, out_img);


		addWeighted( resized_frame, 1.0, contourImage, 1.0, 0.0, out_overlay);
		
		drawFrameCount(out_overlay, framecount);
		
		//imshow ("0", red_thresh);
		//imshow ("1", red_buffer);
		//imshow ("color", out_color);
		//imshow ("3", red_gray);
		//imshow ("4", contourImage);
		imshow ("overlay", out_overlay);
		video_output_overlay->write (out_overlay);
		imwrite("images/" + file_name + "_philip_" + to_string(framecount) + ".jpg", out_overlay);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output_overlay->release();
	destroyAllWindows();
	outfile.close();

}

#define HIST_DIR_BIN 36
#define HIST_SPEED_BIN 50

void fn_philip::histogram(Mat& current, Mat& red, int angle1, int angle2) {

	Mat current_dir = Mat::zeros(height, width, CV_32F);
	Mat current_spd = Mat::zeros(height, width, CV_32F);
	Mat current_red = Mat::ones(height, width, CV_8U);

	// 36-bin histogram for direction
	int hist_dir[HIST_DIR_BIN] = {0};
	for (int y = 0; y < height; y++) {
		Pixel2* ptr = current.ptr<Pixel2>(y, 0);
		float* ptr_dir = current_dir.ptr<float>(y, 0);
		float* ptr_spd = current_spd.ptr<float>(y, 0);
		int* ptr_red = current_red.ptr<int>(y,0);
		const Pixel2* ptr_end = current.ptr<Pixel2>(y+1, 0);
		for (; ptr < ptr_end; ++ptr, ++ptr_dir, ++ptr_spd, ++ptr_red) {
			float theta = atan2(ptr->y, ptr->x) + M_PI;
			int bin = floor(theta / (2*M_PI) * HIST_DIR_BIN);
			++hist_dir[bin];
			*ptr_dir = bin;
			*ptr_spd = sqrt(ptr->x * ptr->x + ptr->y * ptr->y);
		}
	}

	// bins classified as rip
	vector<int> rip_range;
	for (int d = angle1; d <= angle2; ++d) {
		rip_range.push_back(d);
	}

	// 1. Delete ptr->xy of non-rip direction
	// 2. Find max_speed
	// 3. Count number
	float spd_max = 0;
	int rip_count = 0;
	for (int y = 0; y < height; y++) {
		Pixel2* ptr = current.ptr<Pixel2>(y, 0);
		float* ptr_dir = current_dir.ptr<float>(y, 0);
		float* ptr_spd = current_spd.ptr<float>(y, 0);
		int* ptr_red = current_red.ptr<int>(y,0);
		const Pixel2* ptr_end = current.ptr<Pixel2>(y+1, 0);
		for (; ptr < ptr_end; ++ptr, ++ptr_dir, ++ptr_spd, ++ptr_red) {
			bool isRip = false;
			for (auto iter = rip_range.begin(); iter < rip_range.end(); ++iter) {
				if (*iter == *ptr_dir) isRip = true;
			}
			if (isRip) {
				if (spd_max < *ptr_spd) spd_max = *ptr_spd;
				++rip_count;
			}
			else {
				ptr->x = 0;
				ptr->y = 0;
				*ptr_spd = 0;
			}
		}
	}

	// Construct a speed histogram
	int hist_spd[HIST_SPEED_BIN] = {0};
	for (int y = 0; y < height; y++) {
		Pixel2* ptr = current.ptr<Pixel2>(y, 0);
		float* ptr_dir = current_dir.ptr<float>(y, 0);
		float* ptr_spd = current_spd.ptr<float>(y, 0);
		int* ptr_red = current_red.ptr<int>(y,0);
		const Pixel2* ptr_end = current.ptr<Pixel2>(y+1, 0);
		for (; ptr < ptr_end; ++ptr, ++ptr_dir, ++ptr_spd, ++ptr_red) {
			if (*ptr_spd != 0) {
				int bin = floor(*ptr_spd / spd_max * HIST_SPEED_BIN);
				hist_spd[bin]++;
			}
		}
	}

	/*
	for (int i = 0; i < HIST_SPEED_BIN; ++i) {
		cout << i << " :> " << hist_spd[i] << endl;
	}
	*/

	// Find speed threshold
	float spd_thresh = spd_max;
	int curr_count = 0;
	for (int i = HIST_SPEED_BIN - 1; i >= 0 ; --i) {
		if (curr_count < rip_count / 2.0) {
			curr_count += hist_spd[i];
			// cout << curr_count << " : " << rip_count << endl;
			// cout << spd_max / static_cast<float>(HIST_SPEED_BIN) * i << endl;
			spd_thresh = spd_max / static_cast<float>(HIST_SPEED_BIN) * i;
		} else break;
	}

	// Delete slow speed points

	for ( int row = 0; row < red.rows; row++ ) {
		Pixel2* ptr = current.ptr<Pixel2>(row, 0);
		Pixelc* ptr_red = red.ptr<Pixelc>(row, 0);
		float* ptr_spd = current_spd.ptr<float>(row, 0);

		for ( int col = 0; col < red.cols; col++ ) {

			if (*ptr_spd != 0 && *ptr_spd > spd_thresh) {
				ptr_red->z = 1;
			} else {
				ptr_red->z = 0;
				*ptr_spd = 0;

			}
			ptr_red->x = 0;
			ptr_red->y = 0;
			ptr->x = 0;
			ptr->y = 0;

			ptr++;
			ptr_red++;
			ptr_spd++;
		}
	}

}


// Mat current
// int hist[]		-
// int histsum		-
// int hist2d[][]		-
// int histsum2d[]		-
// float UPPER		-
// float UPPER2d[]		-
// float prop_above_upper[]		-
void fn_philip::create_histogram(Mat current, int hist[HIST_BINS], int& histsum, int hist2d[HIST_DIRECTIONS][HIST_BINS]
	 ,int histsum2d[HIST_DIRECTIONS], float& UPPER, float UPPER2d[HIST_DIRECTIONS], float prop_above_upper[HIST_DIRECTIONS]){
	
	//Construct histograms to get thresholds
	//Figure out what "slow" or "fast" is
	for (int y = 0; y < this->height; y++) {
		Pixel3* ptr = current.ptr<Pixel3>(y, 0);
		//const Pixel3* ptr_end = ptr + (int)XDIM;
		const Pixel3* ptr_end = current.ptr<Pixel3>(y+1, 0);
		for (; ptr < ptr_end; ++ptr) {
			int bin = (ptr->y) * HIST_RESOLUTION;
			int angle = (ptr->x * HIST_DIRECTIONS)/ 360; //order matters, truncation
			if(bin < HIST_BINS &&  bin >= 0){
				hist[bin]++; histsum++;
				hist2d[angle][bin]++;
				histsum2d[angle]++;
			}
		}
	}
	
	//Use histogram to create overall threshold
	int threshsum = 0;
	int bin = HIST_BINS-1;
	while(threshsum < (histsum*.05)){
		threshsum += hist[bin];
		bin--;
	}
	UPPER = bin/float(HIST_RESOLUTION);
	int targetbin = bin;
	
	
	//As above, but per-direction
	//This is more of a visual aid, allowing small motion in directions with 
	//little movement to not be drowned out by large-scale motion
	for(int angle = 0; angle < HIST_DIRECTIONS; angle++){
		int threshsum2 = 0;
		int bin = HIST_BINS-1;
		while(threshsum2 < (histsum2d[angle]*.05)){
			threshsum2 += hist2d[angle][bin];
			bin--;
		}
		UPPER2d[angle] = bin/float(HIST_RESOLUTION);
		if(UPPER2d[angle] < 0.01) {UPPER2d[angle] = 0.01;}//avoid division by 0
		
		int threshsum3 = 0;
		bin = HIST_BINS-1;
		while(bin > targetbin){
			threshsum3 += hist2d[angle][bin];
			bin--;
		}
		prop_above_upper[angle] = ((float)threshsum3)/threshsum;
		
		
		//printf("Angle %d; prop_above_upper: %f\n",angle,prop_above_upper[angle]);
	}
}


void fn_philip::display_histogram(int hist2d[HIST_DIRECTIONS][HIST_BINS],int histsum2d[HIST_DIRECTIONS]
					,float UPPER2d[HIST_DIRECTIONS], float UPPER, float prop_above_upper[HIST_DIRECTIONS]){
	namedWindow("Color Histogram", WINDOW_AUTOSIZE );
	
	Mat foo = Mat::ones(480, 480, CV_32FC3);
	
	foo.forEach<Pixel3>([&](Pixel3& pixel, const int position[]) -> void {
		
		
		float tx = (position[1]-240.0)/240.0;
		float ty = (position[0]-240.0)/240.0;
		
		float theta = atan2(ty,tx)*180/M_PI;//find angle
		theta += theta < 0 ? 360 : 0; //enforce strict positive angle
		float r = sqrt(tx*tx + ty*ty);
		
		int direction = ((int)((theta * HIST_DIRECTIONS)/360));
		pixel.x = direction * 360/HIST_DIRECTIONS;
		
		//float proportion = ((float)hist2d[direction][(int)(r*HIST_BINS)])/histsum2d[direction];
		
		pixel.y = r > UPPER2d[direction]*HIST_RESOLUTION/HIST_BINS ? 0 : 1;
		pixel.z = r > prop_above_upper[direction]*10 ? 0 : 1;
	});
	
	
	cvtColor(foo,foo,COLOR_HSV2BGR);
	imshow("Color Histogram",foo);
	
	return;

}