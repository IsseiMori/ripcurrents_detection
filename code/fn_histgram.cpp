#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "fn_histgram.hpp"

using namespace std;

fn_histgram::fn_histgram (string file_name, 
							int _height)
							: method(file_name, _height) {

}


void fn_histgram::run () {
	cout << "Running histgram" << endl;

	VideoWriter* video_output = ini_video_output (file_name + "_histgram");
	

	int hist[HIST_BINS] = {0}; //histogram
	int histsum = 0;
	float UPPER = 100.0; //UPPER can be determined programmatically
	
	int hist2d[HIST_DIRECTIONS][HIST_BINS] = {{0}};
	int histsum2d[HIST_DIRECTIONS] = {0};
	float UPPER2d[HIST_DIRECTIONS] = {0};
	float prop_above_upper[HIST_DIRECTIONS] = {0};

	Mat frame, resized_frame, grayscaled_frame;
	Mat u_curr, u_prev;

	video.read (frame);
	if (frame.empty()) exit(1);
	resize (frame, resized_frame, Size(width, height), 0, 0, INTER_LINEAR);
	cvtColor (resized_frame, grayscaled_frame, COLOR_BGR2GRAY);
	grayscaled_frame.copyTo(u_prev);

	for (int framecount = 1; true; ++framecount) {

		cout << "Frame " << framecount << endl;

		video.read (frame);
		if (frame.empty()) break;
		resize (frame, resized_frame, Size(width, height), 0, 0, INTER_LINEAR);
		cvtColor (resized_frame, grayscaled_frame, COLOR_BGR2GRAY);
		grayscaled_frame.copyTo(u_curr);

		
		
		// Mat out_img_overlay;
		// resized_frame.copyTo(out_img_overlay);

		
		
		Mat flow;
		calcOpticalFlowFarneback(u_prev, u_curr, flow, 0.5, 2, 20, 3, 15, 1.2, OPTFLOW_FARNEBACK_GAUSSIAN);


		Mat out_img = Mat::zeros(cv::Size(width, height), CV_8UC3);

		create_histogram(flow,  hist, histsum, hist2d, histsum2d, UPPER, UPPER2d, prop_above_upper);
		display_histogram(hist2d,histsum2d,UPPER2d, UPPER,prop_above_upper);

		
		imshow ("grid_buoy", resized_frame);
		// imshow ("color", out_img_overlay);
		video_output->write (resized_frame);

		u_curr.copyTo (u_prev);
		if ( waitKey(1) == 27) break;

	}

	// flow.release();

	// clean up
	video_output->release();
	destroyAllWindows();

}


// Mat current
// int hist[]		-
// int histsum		-
// int hist2d[][]		-
// int histsum2d[]		-
// float UPPER		-
// float UPPER2d[]		-
// float prop_above_upper[]		-
void fn_histgram::create_histogram(Mat current, int hist[HIST_BINS], int& histsum, int hist2d[HIST_DIRECTIONS][HIST_BINS]
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


void fn_histgram::display_histogram(int hist2d[HIST_DIRECTIONS][HIST_BINS],int histsum2d[HIST_DIRECTIONS]
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
