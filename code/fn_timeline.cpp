#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "fn_timeline.hpp"

using namespace std;

void mouse_callback(int event, int x, int y, int flags, void *userdata)
{
    if (event == EVENT_LBUTTONDOWN) {
		cout << x << " " << y << endl;
		pair<Pixel2*, Pixel2*> *p = static_cast<pair<Pixel2*, Pixel2*>*>(userdata);
		if (p->first == nullptr) {
			p->first = new Pixel2(x,y);
		} 
		else {
			p->second = new Pixel2(x,y);
		}

    }
}

fn_timeline::fn_timeline (VideoCapture& _video, 
					int _height,
					int _v_num): method(_video, _height), v_num(_v_num){
}

void fn_timeline::run () {
	cout << "Running timeline" << endl;

	VideoWriter* video_output = ini_video_output ("timelines");

	ini_frame();

	imshow ("click start and end", resized_frame);

	pair<Pixel2*, Pixel2*> st_ed;

	setMouseCallback("click start and end", mouse_callback, &st_ed);

	while (st_ed.first == nullptr || st_ed.second == nullptr) {
		waitKey();
	}


	add_timeline (*st_ed.first, *st_ed.second, v_num);


	for (int framecount = 1; true; ++framecount) {

		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		Mat out_img;
		resized_frame.copyTo (out_img);
	
		for (timeline& tl : timelines) {
			tl.runLK(prev_frame, curr_frame, out_img);
		}
		
		imshow ("timelines", out_img);
		video_output->write (out_img);

		curr_frame.copyTo (prev_frame);
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


void timeline::runLK(Mat u_prev, Mat u_curr, Mat& out_img) {

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