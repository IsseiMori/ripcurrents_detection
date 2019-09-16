#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include <fstream> 

#include <opencv2/opencv.hpp>

#include "fn_pathline.hpp"

using namespace std;

fn_pathline::fn_pathline (string _file_name, 
					int _height)
					: method(_file_name, _height) {
}


void fn_pathline::run () {
	cout << "Running color map" << endl;

	VideoWriter* video_output = ini_video_output (file_name + "_pathline");

	ini_frame();

	vector<Pixel2> streampt;
	for(int s = 0; s < 10; s++){
		streampt.push_back (Pixel2(rand()%width,rand()%height));
	}

	Mat overlay = Mat::zeros(Size(width, height), CV_8UC1);
	Mat overlay_color = Mat::zeros(Size(width, height), CV_8UC3);

	for (int framecount = 1; true; ++framecount) {
		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		calc_FB ();

		Mat out_img;
		resized_frame.copyTo(out_img);

		eliminate_std (5);

		calc_pathline (out_img, overlay, overlay_color, streampt, framecount);

		drawFrameCount(out_img, framecount);	
		
		imshow ("grid_buoy overlay", out_img);
		video_output->write (out_img);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output->release();
	destroyAllWindows();

}



void fn_pathline::calc_pathline (Mat& out_img, Mat& overlay, Mat& overlay_color, vector<Pixel2>& streampt, int framecount) {

	Scalar color(framecount*(255.0/total_frame));
	
	for (auto& pt : streampt) {

		for( int i = 0; i< 100; i++){
	
			float x = pt.x;
			float y = pt.y;
			
			int xind = (int) floor(x);
			int yind = (int) floor(y);
			float xrem = x - xind;
			float yrem = y - yind;
			
			if(xind < 1 || yind < 1 || xind + 2 > flow.cols || yind  + 2 > flow.rows)  //Verify array bounds
			{
				return;
			}
			
			//Bilinear interpolation
			Pixel2 delta =		(*flow.ptr<Pixel2>(yind,xind))		* (1-xrem)*(1-yrem) +
			(*flow.ptr<Pixel2>(yind,xind+1))	* (xrem)*(1-yrem) +
			(*flow.ptr<Pixel2>(yind+1,xind))	* (1-xrem)*(yrem) +
			(*flow.ptr<Pixel2>(yind+1,xind+1))	* (xrem)*(yrem) ;
			
			Pixel2 newpt = pt + delta*0.1;
			
			cv::line(overlay, pt, newpt, color, 1, 8, 0);
			
			pt = newpt;
		}
	}

	Mat tmp = Mat::zeros(Size(width, height), CV_8UC3);
	applyColorMap(overlay, overlay_color, COLORMAP_RAINBOW);
	add(overlay_color, tmp, tmp, overlay, -1);
	addWeighted( out_img, 0.5, tmp, 0.5, 0.0, out_img);

}


void fn_pathline::runLK (float v) {
	cout << "Running color map" << endl;

	VideoWriter* video_output = ini_video_output (file_name + "_pathline");

	ini_frame();

	vector<Pixel2> vertices;
	for(int i = 1; i < v; i++){
		for(int j = 1; j < v; j++){
			vertices.push_back (Pixel2(width/v*i,height/v*j));
		}
	}

	Mat overlay = Mat::zeros(Size(width, height), CV_8UC1);
	Mat overlay_color = Mat::zeros(Size(width, height), CV_8UC3);

	for (int framecount = 1; true; ++framecount) {
		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		Mat out_img;
		resized_frame.copyTo(out_img);

		// return status values of calcOpticalFlowPyrLK
		vector<uchar> status;
		vector<float> err;
		vector<Pixel2> vertices_next;

		// run LK for all vertices
		calcOpticalFlowPyrLK(prev_frame, curr_frame, vertices, 
						 vertices_next, status, err, 
						 Size(50,50),3, 
						 TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.1), 
						 10, 1e-4 );
		

		// eliminate any large movement
		for ( int i = 0; i < (int)vertices_next.size(); i++) {
			if ( abs(vertices[i].x - vertices_next[i].x) > 20
				|| abs(vertices[i].y - vertices_next[i].y) > 20 ) {
					vertices_next[i] = vertices[i];
				}
		}

		Scalar color(framecount*(255.0/total_frame));

		// draw edges
		for ( int i = 0; i < (int)vertices.size(); i++ ) {
			line(overlay,Point(vertices[i].x,vertices[i].y),Point(vertices_next[i].x,vertices_next[i].y),color,1,8,0);
		}

		Mat tmp = Mat::zeros(Size(width, height), CV_8UC3);
		applyColorMap(overlay, overlay_color, COLORMAP_RAINBOW);
		add(overlay_color, tmp, tmp, overlay, -1);
		addWeighted( out_img, 0.5, tmp, 0.5, 0.0, out_img);

		// copy the result for the next frame
		vertices = vertices_next;

		drawFrameCount(out_img, framecount);	
		
		imshow ("pathline overlay", out_img);
		video_output->write (out_img);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output->release();
	destroyAllWindows();

}