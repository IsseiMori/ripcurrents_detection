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

	Pixel2 lineStart(10,10);
	Pixel2 lineEnd(100,100);
	float numberOfVertices = 10.0;

	// define the distance between each vertices
	float diffX = (lineEnd.x - lineStart.x) / numberOfVertices;
	float diffY = (lineEnd.y - lineStart.y) / numberOfVertices;

	// create and push Pixel2 points
	for (int i = 0; i <= numberOfVertices; i++)
	{
		streampt.push_back(Pixel2(lineStart.x + diffX * i, lineStart.y + diffY * i));
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
	applyColorMap(overlay, overlay_color, COLORMAP_JET);
	add(overlay_color, tmp, tmp, overlay, -1);
	addWeighted( out_img, 1.0, tmp, 0.5, 0.0, out_img);

}


void mouse_callback_pathline(int event, int x, int y, int, void *userdata)
{
    if (event == EVENT_LBUTTONDOWN) {
		cout << x << " " << y << endl;
		pair<vector<pair<Pixel2, Pixel2>>, Pixel2*> *p = static_cast<pair<vector<pair<Pixel2, Pixel2>>, Pixel2*>*>(userdata);
		if (p->second == nullptr) {
			p->second = new Pixel2(x,y);
		} 
		else {
			p->first.push_back(make_pair(*(p->second), Pixel2(x,y)));
			p->second = nullptr;
		}

    }
}

void fn_pathline::runLK (float v, bool isLine) {
	cout << "Running color map" << endl;

	string str = isLine? "_line" : "";

	VideoWriter* video_output = ini_video_output (file_name + "_pathline_" + to_string(static_cast<int>(v)) + str);

	ini_frame();

	Pixel2 lineStart;
	Pixel2 lineEnd;
	float numberOfVertices = v;
	vector<Pixel2> vertices;

	imshow ("click start and end", resized_frame);

	if (isLine) {
		pair<vector<pair<Pixel2, Pixel2>>, Pixel2*> st_ed;

		setMouseCallback("click start and end", mouse_callback_pathline, &st_ed);

		while (st_ed.first.size() == 0) {
			waitKey();
		}

		lineStart = st_ed.first[0].first;
		lineEnd = st_ed.first[0].second;

		// define the distance between each vertices
		float diffX = (lineEnd.x - lineStart.x) / numberOfVertices;
		float diffY = (lineEnd.y - lineStart.y) / numberOfVertices;

		// create and push Pixel2 points
		for (int i = 0; i <= numberOfVertices; i++)
		{
			vertices.push_back(Pixel2(lineStart.x + diffX * i, lineStart.y + diffY * i));
		}
	}
	else {
		for(int i = 1; i < v; i++){
			for(int j = 1; j < v; j++){
				vertices.push_back (Pixel2(width/v*i,height/v*j));
			}
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
			
			
			float dx = vertices_next[i].x - vertices[i].x;
			float dy = vertices_next[i].y - vertices[i].y;
			
			// theta for the delta displacement between 2 frames
			float theta = atan2 (dy, dx);

			float dt = max(0.1, 50.0/total_frame) * 3;
		
			vertices_next[i].x = vertices[i].x + cos(theta) * dt;
			vertices_next[i].y = vertices[i].y + sin(theta) * dt;
			
		}

		Scalar color(framecount*(255.0/total_frame));

		// draw edges
		for ( int i = 0; i < (int)vertices.size(); i++ ) {
			line(overlay,Point(vertices[i].x,vertices[i].y),Point(vertices_next[i].x,vertices_next[i].y),color,2,8,0);
		}

		Mat tmp = Mat::zeros(Size(width, height), CV_8UC3);
		applyColorMap(overlay, overlay_color, COLORMAP_RAINBOW);
		add(overlay_color, tmp, tmp, overlay, -1);
		addWeighted( out_img, 1.0, tmp, 0.5, 0.0, out_img);

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