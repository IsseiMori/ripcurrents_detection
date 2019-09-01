#include <string>
#include <iostream>
#include <math.h>

#include <opencv2/opencv.hpp>

#include "fn_grid_buoy.hpp"

using namespace std;

fn_grid_buoy::fn_grid_buoy (string _file_name, 
							int _height,
							int _v_count,
							int _h_count)
							: method(_file_name, _height), 
							v_count(_v_count), 
							h_count(_h_count) {

	// define the distance between each vertices
	float diff_x = width  / (h_count + 1);
	float diff_y = height / (v_count + 1);

	max_len = min(diff_x, diff_y);

	// create and push Pixel2 points
	for (int i = 1; i <= v_count; ++i) {
		for (int j = 1; j <= h_count; ++j) {
			vertices_root.push_back(Pixel2(diff_x * j, diff_y * i));
			vertices.push_back(Pixel2(diff_x * j, diff_y * i));
		}
	}
}

void fn_grid_buoy::runLK (bool isNorm) {
	cout << "Running grid buoy (LK)" << endl;

	string n_str = isNorm? "normalized_" : "";
	VideoWriter* video_output = ini_video_output (file_name +  "_grid_buoy_LK_"
		+ n_str + to_string(v_count) + "_" + to_string(h_count));

	ini_frame ();

	for (int framecount = 1; true; ++framecount) {

		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		Mat out_img;
		resized_frame.copyTo (out_img);
	
		
		vertices_runLK (prev_frame, curr_frame, out_img, isNorm);

		drawFrameCount(out_img, framecount);
		
		imshow ("grid_buoy", out_img);
		video_output->write (out_img);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output->release();
	destroyAllWindows();

}

void fn_grid_buoy::runFB () {
	cout << "Running grid buoy (FB)" << endl;

	VideoWriter* video_output = ini_video_output (file_name + "_grid_buoy_FB_"
		+ to_string(v_count) + "_" + to_string(h_count));

	ini_frame ();

	for (int framecount = 1; true; ++framecount) {

		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		calcOpticalFlowFarneback(prev_frame, curr_frame, flow, 0.5, 2, 20, 3, 15, 1.2, OPTFLOW_FARNEBACK_GAUSSIAN);
		
		Mat out_img_overlay;
		Mat out_img;
		resized_frame.copyTo(out_img_overlay);
		resized_frame.copyTo(out_img);
		
		vector_to_color (flow, out_img_overlay);

	
		vertices_runFB (flow, out_img);

		drawFrameCount(out_img, framecount);
		
		imshow ("grid_buoy", out_img);
		imshow ("color", out_img_overlay);
		video_output->write (out_img);

		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output->release();
	destroyAllWindows();

}


void fn_grid_buoy::vertices_runLK (Mat u_prev, Mat u_curr, Mat& out_img, bool isNorm) {

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
		
		if ( abs(vertices_next[i].x - vertices_root[i].x) > max_len
			|| abs(vertices_next[i].y - vertices_root[i].y) > max_len ) {
			
			vertices_next[i] = vertices[i];
		}

		if (isNorm) {
			float dt = 0.05;
			vertices_next[i].x = (vertices_next[i].x - vertices[i].x) > 0 ? vertices[i].x + dt: vertices[i].x - dt;
			vertices_next[i].y = (vertices_next[i].y - vertices[i].y) > 0 ? vertices[i].y + dt: vertices[i].y - dt;
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

	// draw grid
	for ( int i = 0; i < (int)vertices.size(); i++ ) {
		circle(out_img,Point(vertices_root[i].x,vertices_root[i].y),4,CV_RGB(0,0,100),-1,8,0);
		circle(out_img,Point(vertices[i].x,vertices[i].y),4,CV_RGB(100,0,0),-1,8,0);
		line(out_img,Point(vertices_root[i].x,vertices_root[i].y),Point(vertices[i].x,vertices[i].y),CV_RGB(100,0,0),2,8,0);
	}
}



void fn_grid_buoy::vertices_runFB (Mat& flow, Mat& out_img) {

	// output locations of vertices
	vector<Pixel2> vertices_next;

	int count = 0;
	for (Pixel2& v : vertices) {

		int xind = floor(v.x);
		int yind = floor(v.y);
		float xrem = v.x - xind;
		float yrem = v.y - yind;

		Pixel2 flow_vec =	(*flow.ptr<Pixel2>(yind,xind)) * (1-xrem)*(1-yrem) +
		(*flow.ptr<Pixel2>(yind,xind+1))	* (xrem)*(1-yrem) +
		(*flow.ptr<Pixel2>(yind+1,xind))	* (1-xrem)*(yrem) +
		(*flow.ptr<Pixel2>(yind+1,xind+1))	* (xrem)*(yrem) ;


		float f_y = flow_vec.y == flow_vec.y? flow_vec.y:0;		
		float f_x = flow_vec.x == flow_vec.x? flow_vec.x:0;

		float dt = 0.05;

		f_x = f_x > 0 ? dt:-dt;
		f_y = f_y > 0 ? dt:-dt;
		
		vertices_next.push_back (Pixel2 (v.x + f_x * 10, v.y + f_y * 10));
		++count;
	}



	// eliminate any large movement
	for ( int i = 0; i < (int)vertices_next.size(); i++) {
		if ( abs(vertices[i].x - vertices_next[i].x) > 20
			|| abs(vertices[i].y - vertices_next[i].y) > 20 ) {
			
			vertices_next[i] = vertices[i];
		}
		
		if ( abs(vertices_next[i].x - vertices_root[i].x) > max_len
			|| abs(vertices_next[i].y - vertices_root[i].y) > max_len ) {
			
			vertices_next[i] = vertices[i];
		}

		if (vertices_next[i].x < 0 || width < vertices_next[i].x ||
			vertices_next[i].y < 0 || height < vertices_next[i].y) {
			
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

	// draw grid
	for ( int i = 0; i < (int)vertices.size(); i++ ) {

		circle(out_img,Point(vertices_root[i].x,vertices_root[i].y),4,CV_RGB(0,0,100),-1,8,0);
		circle(out_img,Point(vertices[i].x,vertices[i].y),4,CV_RGB(100,0,0),-1,8,0);
		line(out_img,Point(vertices_root[i].x,vertices_root[i].y),Point(vertices[i].x,vertices[i].y),CV_RGB(100,0,0),2,8,0);
	
	}
}