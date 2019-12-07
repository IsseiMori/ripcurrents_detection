#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "fn_virtual_dyes_line.hpp"

using namespace std;

void mouse_callback_virtual_dyes_line(int event, int x, int y, int, void *userdata)
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

fn_virtual_dyes_line::fn_virtual_dyes_line (string file_name, 
					int _height,
					int _vnum,
					float _born_distance,
					int _max_num): method(file_name, _height) {
	vnum = _vnum;
	born_distance = _born_distance;
	max_num = _max_num;
}

void fn_virtual_dyes_line::run (bool isNorm) {
	cout << "Running timeline" << endl;

	ini_frame();

	imshow ("click start and end", resized_frame);

	pair<vector<pair<Pixel2, Pixel2>>, Pixel2*> vec_and_pixel;

	setMouseCallback("click start and end", mouse_callback_virtual_dyes_line, &vec_and_pixel);

	while (vec_and_pixel.first.size() == 0) {
		waitKey();
	}

	for (auto pixels : vec_and_pixel.first) {
		Pixel2 start = pixels.first;
		Pixel2 end = pixels.second;

		// define the distance between each vertices
		float diffX = (end.x - start.x) / vnum;
		float diffY = (end.y - start.y) / vnum;

		// create and push Pixel2 points
		for (int i = 0; i <= vnum; ++i) {
			streaklines.push_back(streakline(Pixel2(start.x + diffX * i, start.y + diffY * i)));
		}

	}

	string n_str = isNorm? "norm_" : "";

	VideoWriter* video_output = ini_video_output (file_name +  "_VDL_" + n_str
		+ to_string(static_cast<int>(vec_and_pixel.first[0].first.x)) + "_" 
		+ to_string(static_cast<int>(vec_and_pixel.first[0].first.y)) + "_"
		+ to_string(static_cast<int>(vec_and_pixel.first[0].second.x)) + "_" 
		+ to_string(static_cast<int>(vec_and_pixel.first[0].second.y)) + "_"
		+ to_string(vnum));

	for (int framecount = 1; true; ++framecount) {

		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		Mat out_img;
		resized_frame.copyTo (out_img);

		for (auto& s : streaklines) s.runLK (prev_frame, curr_frame, out_img, born_distance, max_num, isNorm);

		drawFrameCount(out_img, framecount);
		
		imshow ("timelines", out_img);
		video_output->write (out_img);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output->release();
	destroyAllWindows();

}

fn_virtual_dyes_line::streakline::streakline (Pixel2 _root) {
	root = _root;
	vertices.push_back (root);
}


void fn_virtual_dyes_line::streakline::runLK(Mat& u_prev, Mat& u_curr, Mat& out_img, float born_d, int max_n, bool isNorm) {

	// return status values of calcOpticalFlowPyrLK
	vector<uchar> status;
	vector<float> err;


	// output locations of vertices
	vector<Pixel2> vertices_next;


	Pixel2 p = *(vertices.end() - 1);
	if (sqrt(pow(p.x - root.x,2) + pow(p.y - root.y,2) > born_d)) {
		vertices.push_back (root);
		if ((int)(vertices.size()) > max_n) vertices.erase (vertices.begin()); 
	}


	// run LK for all vertices
	calcOpticalFlowPyrLK(u_prev, u_curr, vertices, 
						 vertices_next, status, err, 
						 Size(20,20),3, 
						 TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.1), 
						 10, 1e-4 );


	// eliminate any large movement
	for ( int i = 0; i < (int)vertices_next.size(); i++) {
		if ( abs(vertices[i].x - vertices_next[i].x) > 20
			|| abs(vertices[i].y - vertices_next[i].y) > 20 ) {
				vertices_next[i] = vertices[i];
			}
		
		if (isNorm) {
			float x = vertices_next[i].x - vertices[i].x;
			float y = vertices_next[i].y - vertices[i].y;
			
			float theta = atan2 (y, x);

			float dt = 0.5;
		
			vertices_next[i].x = vertices[i].x + cos(theta) * dt;
			vertices_next[i].y = vertices[i].y + sin(theta) * dt;
		}
	}

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
	Mat overlay;
    double opacity = 0.03;
    for ( int i = 0; i < (int)vertices.size(); i++ ) {
        out_img.copyTo(overlay);
        circle(overlay,Point(vertices[i].x,vertices[i].y),40,CV_RGB(100,0,0),-1,8,0);
        addWeighted(overlay, opacity, out_img, 1 - opacity, 0, out_img, -1);
    }
}