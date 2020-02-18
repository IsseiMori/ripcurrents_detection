#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <algorithm>

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
					int _birth_rate,
					int _max_num,
					float _opacity,
					float _dt): method(file_name, _height) {
	vnum = _vnum;
	birth_rate = _birth_rate;
	max_num = _max_num;
	opacity = _opacity;
	dt = _dt;

	// For grid arrows
	w_count = 20;
	h_count = 20;
	buffer_size = 300;

	// define the distance between each vertices
	float diff_x = width  / (h_count + 1);
	float diff_y = height / (w_count + 1);

	max_len = min(diff_x, diff_y);

	current_buffer = 0;


	// create and push Pixel2 points
	for (int i = 1; i <= w_count; ++i) {
		for (int j = 1; j <= h_count; ++j) {
			root_vec.push_back (Pixel2(diff_x * j, diff_y * i));
			v_vec.push_back (Pixel2(diff_x * j, diff_y * i));
			relative_vec.push_back (Pixel2(0,0));
			average_vec.push_back (Pixel2(0,0));
			isVisible.push_back (true);
			howLikely.push_back (0);
			theta_vec.push_back (0);
		}
	}

	for (int i = 0; i < buffer_size; ++i) {
		vector<Pixel2> vec;
		for (int w = 1; w <= w_count; ++w) {
			for (int h = 1; h <= h_count; ++h) {
				vec.push_back (Pixel2(0,0));
			}
		}
		vec_buffer.push_back (vec);
	}
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

	ostringstream opacity_str;
	opacity_str << fixed << setprecision(1) << opacity;

	ostringstream dt_str;
	dt_str << fixed << setprecision(1) << dt;

	VideoWriter* video_output = ini_video_output (file_name +  "_VDL_" + n_str
		+ to_string(static_cast<int>(vec_and_pixel.first[0].first.x)) + "_" 
		+ to_string(static_cast<int>(vec_and_pixel.first[0].first.y)) + "_"
		+ to_string(static_cast<int>(vec_and_pixel.first[0].second.x)) + "_" 
		+ to_string(static_cast<int>(vec_and_pixel.first[0].second.y)) + "_"
		+ to_string(vnum) + "_"
		+ to_string(birth_rate) + "_"
		+ to_string(max_num) + "_"
		+ opacity_str.str() + "_"
		+ dt_str.str());

	for (int framecount = 1; true; ++framecount) {

		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		Mat out_img;
		resized_frame.copyTo (out_img);

		// Grid arrow
		for (size_t i = 0; i < average_vec.size(); ++i) {
			average_vec[i] -= vec_buffer[current_buffer][i];
		}
		vec_buffer[current_buffer].clear ();
		vertices_runLK (prev_frame, curr_frame, out_img, isNorm, framecount);

		// count total v
		int total_vnum = 0;
		for (auto& s : streaklines) {
			if ((framecount % birth_rate) == 0) s.add(max_num);
			total_vnum += s.vertices.size();
		}
		
		for (auto& s : streaklines) {
			s.runLK (prev_frame, curr_frame, out_img, max_num, isNorm, opacity, dt, total_vnum, vnum);
		}

		// Draw gray lines as initial position of the VDL line
		for (auto pixels : vec_and_pixel.first) {
			line(out_img,Point(pixels.first.x,pixels.first.y),Point(pixels.second.x,pixels.second.y),CV_RGB(50,50,50),3,8,0);
		}

		drawFrameCount(out_img, framecount);
		
		imshow ("timelines", out_img);
		video_output->write (out_img);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

		current_buffer++;
        if ( current_buffer >= buffer_size ) current_buffer = 0;

	}

	// clean up
	video_output->release();
	destroyAllWindows();

}


int fn_virtual_dyes_line::filter_frequency6 () {
	int hist[6] = {0,0,0,0,0,0};

	float ave_magnitude = 0;

	// Create a histgram and find average magnitude
	for (int i = 0; i < static_cast<int>(theta_vec.size()); ++i) {
		ave_magnitude += sqrt(average_vec[i].x * average_vec[i].x 
							+ average_vec[i].y * average_vec[i].y);
	}

	ave_magnitude /= average_vec.size();

	for (int i = 0; i < static_cast<int>(theta_vec.size()); ++i) {
		float magnitude = sqrt(average_vec[i].x * average_vec[i].x 
							 + average_vec[i].y * average_vec[i].y);
		if (magnitude > ave_magnitude * 0.5) {
			int bin = static_cast<int>((theta_vec[i] + 180) / 360 * 6);
			if (bin == 6) bin = 0;
			hist[bin]++;
		}
		
	}

	int max_hist = 0;
	int max_id = 0;
	//cout << "BIN" << endl;
	for (int i = 0; i < 6; ++i) {
		if (hist[i] > max_hist) {
			max_id = i;
			max_hist = hist[i];
		}
	}

	int oppose1 = (max_id + 3 > 5) ? max_id - 3 : max_id + 3;
	int opp_near1 = (max_id + 4 > 5) ? max_id - 2 : max_id + 4;
	int opp_near2 = (max_id + 2 > 5) ? max_id - 4 : max_id + 2;
	int max_near1 = (max_id + 1 > 5) ? max_id - 5 : max_id + 1;
	int max_near2 = (max_id + 5 > 5) ? max_id - 1 : max_id + 5;

	for (int i = 0; i < static_cast<int>(theta_vec.size()); ++i) {
		if (sqrt(average_vec[i].x * average_vec[i].x 
			   + average_vec[i].y * average_vec[i].y) < ave_magnitude * 0.5) {
			howLikely[i] = 0;
			isVisible[i] = false;

		} else {
			int bin = static_cast<int>((theta_vec[i] + 180) / 360 * 6);
			if (bin == 6) bin = 0;
			// howLikely[i] = bin;
			// isVisible[i] = true;
			if (bin == oppose1 ) {
				howLikely[i] = 1;
				isVisible[i] = true;
			}
			else if (bin == opp_near1 || bin == opp_near2 ) {
				howLikely[i] = 0.5;
				isVisible[i] = true;
			}
			else if (bin == max_id){
				howLikely[i] = 0;
				isVisible[i] = false;
			}
			else if (bin == max_near1 || bin == max_near2) {
				howLikely[i] = 0;
				isVisible[i] = false;
			}
			else {
				howLikely[i] = 0;
				isVisible[i] = false;
				cout << "exception : " <<  bin << endl;
			}
		}
	 	// isVisible[i] = true;
	}
	return max_id;
}

void fn_virtual_dyes_line::streakline::vertices_filter(int max_id) {

	int max_near1 = (max_id + 1 > 5) ? max_id - 5 : max_id + 1;
	int max_near2 = (max_id + 5 > 5) ? max_id - 1 : max_id + 5;

	for (auto iter = vertices.begin(); iter != vertices.end();) {

		float dx = root.x - iter->x;
		float dy = root.y - iter->y;

		float e_dt = 50;

		if (sqrt(dx * dx + dy * dy) >  e_dt) {

			// No +180 for side way filtering. Bug?
			int bin = static_cast<int>((atan2 (dx, dy) * 180 / M_PI + 180) / 360 * 6);
			// int bin = static_cast<int>((atan2 (dx, dy) * 180 / M_PI ) / 360 * 6);
			if (bin == max_id || bin == max_near1 || bin == max_near2) {
				iter = vertices.erase(iter);
			}
			else {
				++iter;
			}
		}
		else {
			++iter;
		}
	}
}


void fn_virtual_dyes_line::vertices_runLK (Mat u_prev, Mat u_curr, Mat& out_img, bool isNorm, float framecount) {
	// return status values of calcOpticalFlowPyrLK
	vector<uchar> status;
	vector<float> err;


	// output locations of vertices
	vector<Pixel2> v_next_vec;

	// run LK for all vertices
	calcOpticalFlowPyrLK(u_prev, u_curr, v_vec, 
						 v_next_vec, status, err, 
						 Size(20,20),3, 
						 TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.1), 
						 10, 1e-4 );

	// eliminate any large movement
	for ( int i = 0; i < (int)v_next_vec.size(); i++) {
        /*
		if ( abs(v_vec[i].x - v_next_vec[i].x) > 20
			|| abs(v_vec[i].y - v_next_vec[i].y) > 20 ) {
			
			v_next_vec[i] = v_vec[i];
		}
        */

	
		if (isNorm) {
			float dx = v_next_vec[i].x - v_vec[i].x;
			float dy = v_next_vec[i].y - v_vec[i].y;
			
			// theta for the delta displacement between 2 frames
			float theta = atan2 (dy, dx);

			float dt = max(0.1, 50.0/total_frame);
		
			v_next_vec[i].x = v_vec[i].x + cos(theta) * dt;
			v_next_vec[i].y = v_vec[i].y + sin(theta) * dt;
		} 


		/*
		if ( abs(v_next_vec[i].x - root_vec[i].x) > max_len
			|| abs(v_next_vec[i].y - root_vec[i].y) > max_len ) {
			
			v_next_vec[i] = v_vec[i];
		}
		*/


		float xp = v_next_vec[i].x - root_vec[i].x;
		float yp = v_next_vec[i].y - root_vec[i].y;

		vec_buffer[current_buffer].push_back (Pixel2(xp,yp) * 200);
		average_vec[i] += (Pixel2(xp,yp) * 200);

		relative_vec[i] = Pixel2(xp,yp);
		theta_vec[i] = atan2 (average_vec[i].y, average_vec[i].x) * 180 / M_PI;
	}
	


	float max_angle = filter_frequency6 ();
	float max_x = -cos(max_angle) * 10;
	float max_y = -sin(max_angle) * 10;


	// Filter out
	for (auto& s : streaklines) {
		s.vertices_filter(max_angle);
	}


	arrowedLine(out_img, Point(20,20), 
						Point(20+max_x*1.2,20+max_y*1.2), 
						Scalar(255,255,255), 3, 8, 0, 0.4);
}


fn_virtual_dyes_line::streakline::streakline (Pixel2 _root) {
	root = _root;
	vertices.push_back (root);
}

void fn_virtual_dyes_line::streakline::add(int max_n) {
    vertices.push_back(root);
    if (max_n != 0 && vertices.size() > max_n) vertices.erase(vertices.begin());
}


void fn_virtual_dyes_line::streakline::runLK(Mat& u_prev, Mat& u_curr, Mat& out_img, int max_n, bool isNorm, float opacity, float dt, int total_vnum, int vnum) {

	if (vertices.size() == 0) return;

	// return status values of calcOpticalFlowPyrLK
	vector<uchar> status;
	vector<float> err;


	// output locations of vertices
	vector<Pixel2> vertices_next;


	/*
	Pixel2 p = *(vertices.end() - 1);
	if (sqrt(pow(p.x - root.x,2) + pow(p.y - root.y,2) > born_d)) {
		vertices.push_back (root);
		if ((int)(vertices.size()) > max_n) vertices.erase (vertices.begin()); 
	}
	*/


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
		
		// Delete v if in the incoming waves direction
		

		
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

	//opacity = 5 / static_cast<float>(total_vnum) ;

	// draw edges
	Mat overlay;
	out_img.copyTo(overlay);
    for ( int i = 0; i < (int)vertices.size(); i++ ) {
        circle(overlay,Point(vertices[i].x,vertices[i].y),40,CV_RGB(100,0,0),-1,8,0);
    }
	addWeighted(overlay, 0.4, out_img, 1 - 0.4, 0, out_img, -1);


/*
	Mat overlay;
    for ( int i = 0; i < (int)vertices.size(); i++ ) {
        out_img.copyTo(overlay);
        circle(overlay,Point(vertices[i].x,vertices[i].y),40,CV_RGB(100,0,0),-1,8,0);
        addWeighted(overlay, opacity, out_img, 1 - opacity, 0, out_img, -1);
    }
    //overlay.copyTo(out_img);*/
}

