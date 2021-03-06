#include <string>
#include <iostream>
#include <math.h>

#include <opencv2/opencv.hpp>

#include "fn_grid_buoy.hpp"

using namespace std;

fn_grid_buoy::fn_grid_buoy (string _file_name, 
							int _height,
							int _w_count,
							int _h_count)
							: method(_file_name, _height), 
							w_count(_w_count), 
							h_count(_h_count) {

	// define the distance between each vertices
	float diff_x = width  / (h_count + 1);
	float diff_y = height / (w_count + 1);

	max_len = min(diff_x, diff_y);

	// create and push Pixel2 points
	for (int i = 1; i <= w_count; ++i) {
		for (int j = 1; j <= h_count; ++j) {
			root_vec.push_back (Pixel2(diff_x * j, diff_y * i));
			v_vec.push_back (Pixel2(diff_x * j, diff_y * i));
			relative_vec.push_back (Pixel2(0,0));
			isVisible.push_back (true);
			howLikely.push_back (0);
			theta_vec.push_back (0);
		}
	}
}

void fn_grid_buoy::runLK (bool isNorm) {
	cout << "Running grid buoy (LK)" << endl;

	string n_str = isNorm? "norm_" : "";
	VideoWriter* video_output = ini_video_output (file_name +  "_grid_buoy_LK_"
		+ n_str + to_string(w_count) + "_" + to_string(h_count));

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
		+ to_string(w_count) + "_" + to_string(h_count));

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

/*
right 0 degree
and clockwise

vector alignment
0 1 2
3 4 5

*/


void fn_grid_buoy::filter_total_mean () {

	Pixel2 sum(0,0);
	for (auto p : relative_vec) sum += p;
	Pixel2 mean = sum / static_cast<float>(relative_vec.size());

	float theta_mean = atan2 (mean.y, mean.x) * 180 / M_PI;

	for (int i = 0; i < static_cast<int>(theta_vec.size()); ++i) {
		float theta_diff = abs(theta_vec[i] - theta_mean);
		if (theta_diff > 180) theta_diff = 360 - theta_diff;
		
		howLikely[i] = theta_diff / 180;
		if (theta_diff > 60) isVisible[i] = true;
		else isVisible[i] = false;
	}
}

void fn_grid_buoy::filter_row_col_mean () {
	
	for (int row = 0; row < h_count; ++row) {

		Pixel2 sum(0,0);
		for (int col = 0; col < w_count; ++col) {
			sum += relative_vec[w_count * row + col];
		}
		Pixel2 mean = sum / static_cast<float>(w_count);
		float theta_mean = atan2 (mean.y, mean.x) * 180 / M_PI;

		for (int col = 0; col < w_count; ++col) {
			sum += relative_vec[w_count * row + col];

			float theta_diff = abs(theta_vec[w_count * row + col] - theta_mean);
			if (theta_diff > 180) theta_diff = 360 - theta_diff;

			howLikely[w_count * row + col] = theta_diff / 180;
			if (theta_diff > 60) isVisible[w_count * row + col] = true;
			else isVisible[w_count * row + col] = false;
		}


	}

}


float fn_grid_buoy::filter_frequency () {

	#define BIN 6
	int hist[6] = {0,0,0,0,0,0};

	float ave_magnitude = 0;

	// Create a histgram
	for (int i = 0; i < static_cast<int>(theta_vec.size()); ++i) {
		ave_magnitude += sqrt(relative_vec[i].x * relative_vec[i].x 
							+ relative_vec[i].y * relative_vec[i].y);
	}

	ave_magnitude /= relative_vec.size();

	for (int i = 0; i < static_cast<int>(theta_vec.size()); ++i) {
		float magnitude = sqrt(relative_vec[i].x * relative_vec[i].x 
							+ relative_vec[i].y * relative_vec[i].y);
		if (magnitude > ave_magnitude * 0.5) {
			int bin = static_cast<int>((theta_vec[i] + 180) / 360 * 6);
			if (bin == 6) bin = 0;
			hist[bin]++;
		}
		
	}

	int max_hist = 0;
	int max_id = 0;
	//cout << "BIN" << endl;
	for (int i = 0; i < BIN; ++i) {
		if (hist[i] > max_hist) {
			max_id = i;
			max_hist = hist[i];
		}
		//cout << hist[i] << endl;
	}

	int oppose1 = (max_id + 3 > 5) ? max_id - 3 : max_id + 3;
	int opp_near1 = (max_id + 4 > 5) ? max_id - 2 : max_id + 4;
	int opp_near2 = (max_id + 2 > 5) ? max_id - 4 : max_id + 2;
	int max_near1 = (max_id + 1 > 5) ? max_id - 5 : max_id + 1;
	int max_near2 = (max_id + 5 > 5) ? max_id - 1 : max_id + 5;

	for (int i = 0; i < static_cast<int>(theta_vec.size()); ++i) {
		if (sqrt(relative_vec[i].x * relative_vec[i].x 
			   + relative_vec[i].y * relative_vec[i].y) < ave_magnitude * 0.3) {
			howLikely[i] = 0;
			isVisible[i] = false;

		} else {
			int bin = static_cast<int>((theta_vec[i] + 180) / 360 * 6);
			if (bin == 6) bin = 0;
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
				cout << "exception" << endl;
			}
		}
        // isVisible[i] = true;
	}

	return (max_id / 6.0 * M_PI * 2);
}

void fn_grid_buoy::vertices_runLK (Mat u_prev, Mat u_curr, Mat& out_img, bool isNorm) {

	// return status values of calcOpticalFlowPyrLK
	vector<uchar> status;
	vector<float> err;


	// output locations of vertices
	vector<Pixel2> v_next_vec;

	// run LK for all vertices
	calcOpticalFlowPyrLK(u_prev, u_curr, v_vec, 
						 v_next_vec, status, err, 
						 Size(50,50),3, 
						 TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.1), 
						 10, 1e-4 );


	// eliminate any large movement
	for ( int i = 0; i < (int)v_next_vec.size(); i++) {
		if ( abs(v_vec[i].x - v_next_vec[i].x) > 20
			|| abs(v_vec[i].y - v_next_vec[i].y) > 20 ) {
			
			v_next_vec[i] = v_vec[i];
		}

	
		if (isNorm) {
			float dx = v_next_vec[i].x - v_vec[i].x;
			float dy = v_next_vec[i].y - v_vec[i].y;
			
			// theta for the delta displacement between 2 frames
			float theta = atan2 (dy, dx);

			float dt = max(0.1, 50.0/total_frame);
		
			v_next_vec[i].x = v_vec[i].x + cos(theta) * dt;
			v_next_vec[i].y = v_vec[i].y + sin(theta) * dt;
		} 



		if ( abs(v_next_vec[i].x - root_vec[i].x) > max_len
			|| abs(v_next_vec[i].y - root_vec[i].y) > max_len ) {
			
			v_next_vec[i] = v_vec[i];
		}

		float xp = v_next_vec[i].x - root_vec[i].x;
		float yp = v_next_vec[i].y - root_vec[i].y;
		relative_vec[i] = Pixel2(xp,yp);
		theta_vec[i] = atan2 (yp, xp) * 180 / M_PI;

	}

	
	
	// copy the result for the next frame
	v_vec = v_next_vec;

	float max_angle = filter_frequency ();
	float max_x = -cos(max_angle) * 10;
	float max_y = -sin(max_angle) * 10;

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

	/*
	// draw grid
	for ( int i = 0; i < (int)v_vec.size(); i++ ) {
		if (isVisible[i]) {
			// BGR
			Scalar base(50, 50, 50);
			Scalar color1(100, 100 - howLikely[i] * 100, 100 - howLikely[i] * 100);
			Scalar color2(100 - howLikely[i] * 100, 100 - howLikely[i] * 100, 100);

			circle(out_img,Point(root_vec[i].x,root_vec[i].y),3,base,-1,8,0);
			circle(out_img,Point(v_vec[i].x,v_vec[i].y),4,color2,-1,8,0);
			line(out_img,Point(root_vec[i].x,root_vec[i].y),Point(v_vec[i].x,v_vec[i].y),color2,2,8,0);
		}
		
	}*/


	for ( int i = 0; i < (int)v_vec.size(); i++ ) {
		if (isVisible[i]) {
			// BGR
			Scalar base(50, 50, 50);
			Scalar color1(100, 100 - howLikely[i] * 100, 100 - howLikely[i] * 100);
			Scalar color2(0, 255 - howLikely[i] * 255, 255 - howLikely[i] * 100);

			circle(out_img,Point(root_vec[i].x,root_vec[i].y),3,color2,-1,8,0);
			arrowedLine(out_img, 
						Point(root_vec[i].x,root_vec[i].y), 
						Point(v_vec[i].x,v_vec[i].y), 
						color2, 4, 8, 0, 0.7);
		}
	}


	arrowedLine(out_img, Point(20,20), 
						Point(20+max_x*1.2,20+max_y*1.2), 
						Scalar(255,255,255), 2, 8, 0, 0.5);
}



void fn_grid_buoy::vertices_runFB (Mat& flow, Mat& out_img) {

	// output locations of vertices
	vector<Pixel2> v_next_vec;

	int count = 0;
	for (Pixel2& v : v_vec) {

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
		
		v_next_vec.push_back (Pixel2 (v.x + f_x * 10, v.y + f_y * 10));
		++count;
	}



	// eliminate any large movement
	for ( int i = 0; i < (int)v_next_vec.size(); i++) {
		if ( abs(v_vec[i].x - v_next_vec[i].x) > 20
			|| abs(v_vec[i].y - v_next_vec[i].y) > 20 ) {
			
			v_next_vec[i] = v_vec[i];
		}
		
		if ( abs(v_next_vec[i].x - root_vec[i].x) > max_len
			|| abs(v_next_vec[i].y - root_vec[i].y) > max_len ) {
			
			v_next_vec[i] = v_vec[i];
		}

		if (v_next_vec[i].x < 0 || width < v_next_vec[i].x ||
			v_next_vec[i].y < 0 || height < v_next_vec[i].y) {
			
			v_next_vec[i] = v_vec[i];
		}
	}
	
	// copy the result for the next frame
	v_vec = v_next_vec;

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
	for ( int i = 0; i < (int)v_vec.size(); i++ ) {

		circle(out_img,Point(root_vec[i].x,root_vec[i].y),4,CV_RGB(0,0,100),-1,8,0);
		circle(out_img,Point(v_vec[i].x,v_vec[i].y),4,CV_RGB(100,0,0),-1,8,0);
		line(out_img,Point(root_vec[i].x,root_vec[i].y),Point(v_vec[i].x,v_vec[i].y),CV_RGB(100,0,0),2,8,0);
	
	}
}