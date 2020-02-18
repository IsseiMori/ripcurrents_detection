#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>

#include <opencv2/opencv.hpp>

#include "fn_virtual_dyes.hpp"

using namespace std;

void mouse_callback_virtual_dyes(int event, int x, int y, int, void *userdata)
{
    if (event == EVENT_LBUTTONDOWN) {
		cout << x << " " << y << endl;
		//pair<vector<pair<Pixel2, Pixel2>>, Pixel2*> *p = static_cast<pair<vector<pair<Pixel2, Pixel2>>, Pixel2*>*>(userdata);
		Pixel2 *p = static_cast<Pixel2*>(userdata);
        p->x = x;
        p->y = y;
    }
}

fn_virtual_dyes::fn_virtual_dyes (string file_name, 
					int _height,
					int _birth_rate,
					int _max_num,
                    float _draw_r,
					float _opacity,
					float _dt): method(file_name, _height) {
	birth_rate = _birth_rate;
	max_num = _max_num;
    draw_r = _draw_r;
	opacity = _opacity;
	dt = _dt;
}

void fn_virtual_dyes::run (bool isNorm) {
	cout << "Running virtual_dyes" << endl;

	ini_frame();

	imshow ("click start and end", resized_frame);

	Pixel2 root = Pixel2(100,100);

	setMouseCallback("click start and end", mouse_callback_virtual_dyes, &root);

    waitKey();

    Dyes dyes = Dyes();

    dyes.root = Pixel2(root.x, root.y);

    cout << root.x << endl;

	string n_str = isNorm? "norm_" : "";

	ostringstream opacity_str;
	opacity_str << fixed << setprecision(1) << opacity;

	ostringstream dt_str;
	dt_str << fixed << setprecision(1) << dt;

	VideoWriter* video_output = ini_video_output (file_name +  "_virtual_dyes_" + n_str
		+ to_string(static_cast<int>(root.x)) + "_" 
		+ to_string(static_cast<int>(root.y)) + "_"
		+ to_string(birth_rate) + "_"
		+ to_string(max_num) + "_"
		+ to_string(draw_r) + "_"
		+ opacity_str.str() + "_"
		+ dt_str.str());

	for (int framecount = 1; true; ++framecount) {

		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		Mat out_img;
		resized_frame.copyTo (out_img);

        if (((framecount - 1) % birth_rate) == 0) dyes.add(max_num);
		dyes.runLK (prev_frame, curr_frame, out_img, draw_r, isNorm, opacity, dt);

		drawFrameCount(out_img, framecount);
		
		imshow ("vitrual dyes", out_img);
		video_output->write (out_img);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output->release();
	destroyAllWindows();

}

fn_virtual_dyes::Dyes::Dyes () {
}


void fn_virtual_dyes::Dyes::add(int max_n) {
    vertices.push_back(root);
    // if (vertices.size() > max_n) vertices.erase(vertices.begin());
}

void fn_virtual_dyes::Dyes::runLK(Mat& u_prev, Mat& u_curr, Mat& out_img, float r, bool isNorm, float opacity, float dt) {

	// return status values of calcOpticalFlowPyrLK
	vector<uchar> status;
	vector<float> err;


	// output locations of vertices
	vector<Pixel2> vertices_next;


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

    Mat overlay;
    for ( int i = 0; i < (int)vertices.size(); i++ ) {
        out_img.copyTo(overlay);
        circle(overlay,Point(vertices[i].x,vertices[i].y),r,CV_RGB(100,0,0),-1,8,0);
        addWeighted(overlay, opacity, out_img, 1 - opacity, 0, out_img, -1);
    }

	// Draw gray point as initial position of the dye
	circle(out_img,Point(root.x,root.y),10,CV_RGB(50,50,50),-1,8,0);

    //overlay.copyTo(out_img);

}