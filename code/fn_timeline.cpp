#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "fn_timeline.hpp"

using namespace std;

fn_timeline::fn_timeline (VideoCapture& _video, 
					int _height): method(_video, _height) {
}

void fn_timeline::run () {
	cout << "Running timeline" << endl;

	VideoWriter* video_output = ini_video_output ("timelines");

	Mat frame, resized_frame, grayscaled_frame;
	UMat u_curr, u_prev;

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

		Mat out_img;
		resized_frame.copyTo (out_img);
	
		for (timeline& tl : timelines) {
			tl.runLK(u_prev, u_curr, out_img);
		}
		
		imshow ("timelines", out_img);
		video_output->write (out_img);

		u_curr.copyTo (u_prev);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output->release();
	destroyAllWindows();

}

void fn_timeline::add_timeline (Pixel2 start, Pixel2 end, int vertices_count) {
	timelines.push_back(timeline(start, end, vertices_count));
}

timeline::timeline (Pixel2 start, Pixel2 end, int vertices_count) {
	
	// define the distance between each vertices
	float diffX = (end.x - start.x) / vertices_count;
	float diffY = (end.y - start.y) / vertices_count;

	// create and push Pixel2 points
	for (int i = 0; i <= vertices_count; ++i) {
		vertices.push_back(Pixel2(start.x + diffX * i, start.y + diffY * i));
	}

}


void timeline::runLK(UMat u_prev, UMat u_curr, Mat& out_img) {

	// return status values of calcOpticalFlowPyrLK
	vector<uchar> status;
	vector<float> err;


	// output locations of vertices
	vector<Pixel2> vertices_next;

	// run LK for all vertices
	calcOpticalFlowPyrLK(u_prev, u_curr, vertices, 
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
	
	// copy the result for the next frame
	vertices = vertices_next;

	/*
	// delete out of bound vertices
	for ( int i = 0; i < (int)vertices.size(); i++) {
		// if vertex is not in the image
		//printf("%d %f \n", YDIM, vertices.at(i).y);
		if (vertices.at(i).x <= 0 || vertices.at(i).x >= XDIM || vertices.at(i).y <= 0 || vertices.at(i).y >= YDIM) {
			vertices.erase(vertices.begin(), vertices.begin() + i);
		}
	}
	*/

	// draw edges
	circle(out_img,Point(vertices[0].x,vertices[0].y),4,CV_RGB(0,0,100),-1,8,0);
	for ( int i = 0; i < (int)vertices.size() - 1; i++ ) {
		line(out_img,Point(vertices[i].x,vertices[i].y),Point(vertices[i+1].x,vertices[i+1].y),CV_RGB(100,0,0),2,8,0);
		circle(out_img,Point(vertices[i+1].x,vertices[i+1].y),4,CV_RGB(0,0,100),-1,8,0);
	}
}