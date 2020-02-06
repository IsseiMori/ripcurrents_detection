#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include <fstream> 
#include <time.h>

#include <opencv2/opencv.hpp>

#include "fn_LK_all_pixel.hpp"

using namespace std;

fn_LK_all_pixel::fn_LK_all_pixel (string _file_name, 
					int _height)
					: method(_file_name, _height) {
}

void fn_LK_all_pixel::LK_to_color (vector<Pixel2>& vertices, vector<Pixel2>& vertices_next, Mat& out_img) {

	float max_displacement = 0;
	vector<float> mag;
	
	for (int row = 0; row < height; ++row ) {
		Pixelc* ptr = out_img.ptr<Pixelc>(row, 0);
		for (int col = 0; col < width; ++col ) {

			int i = width * row + col;

			float dx = vertices_next[i].x - vertices[i].x;
			float dy = vertices_next[i].y - vertices[i].y;

			float theta = atan2(dy, dx)*180/M_PI;	// find angle
			theta += theta < 0 ? 360 : 0;	// enforce strict positive angle

			ptr->x = theta / 2;
			ptr->y = 255;
			ptr->z = 255;

			float magnitude = sqrt(dx * dx + dy * dy);
			mag.push_back (magnitude);

			// store the previous max to maxmin next frame
			if ( magnitude > max_displacement)  {
				max_displacement = magnitude;
				// cout << "n " << vertices_next[i].x << " v " << vertices[i].x << endl;
			} 
			
			++ptr;
		}
	}
	
	/*
	auto iter = mag.begin();
	for (int row = 0; row < height; ++row ) {
		Pixelc* ptr = out_img.ptr<Pixelc>(row, 0);
		for (int col = 0; col < width; ++col ) {
			ptr->z = *iter / max_displacement * 255;
			++iter;
			++ptr;
		}
	}
	*/
	

	// show as hsv format
	cvtColor(out_img, out_img, COLOR_HSV2BGR);

}

void fn_LK_all_pixel::run () {
	cout << "Running color map" << endl;

	VideoWriter* video_output = ini_video_output (file_name + "_LK_all");
	VideoWriter* video_output_overlay = ini_video_output (file_name + "_LK_all_" + "overlay");

	ini_frame();

	vector<Pixel2> vertices;
	for (int col = 1; col <= height; ++col ) {
		for (int row = 1; row <= width; ++row ) {
			vertices.push_back (Pixel2(row, col));
		}
	}

	ini_draw_colorwheel ();

	for (int framecount = 1; true; ++framecount) {
		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		Mat out_img;
		Mat out_img_overlay;
		resized_frame.copyTo(out_img);
		resized_frame.copyTo(out_img_overlay);

		// return status values of calcOpticalFlowPyrLK
		vector<uchar> status;
		vector<float> err;
		vector<Pixel2> vertices_next;

		// run LK for all vertices
		calcOpticalFlowPyrLK(prev_frame, curr_frame, vertices, 
						 vertices_next, status, err, 
						 Size(5,5),3, 
						 TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.1), 
						 10, 1e-4 );


		Scalar color(framecount*(255.0/total_frame));


		LK_to_color (vertices, vertices_next, out_img);

		drawFrameCount(out_img, framecount);

		addWeighted( out_img, 0.5, out_img_overlay, 1.0, 0.0, out_img_overlay);

		draw_colorwheel (out_img);
		draw_colorwheel (out_img_overlay);
		
		imshow ("pathline overlay", out_img);
		imshow ("grid_buoy overlay", out_img_overlay);
		video_output->write (out_img);
		video_output_overlay->write (out_img_overlay);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output->release();
	video_output_overlay->release();
	destroyAllWindows();

}

void fn_LK_all_pixel::justrun () {

	ini_frame();

	vector<Pixel2> vertices;
	for (int col = 1; col <= height; ++col ) {
		for (int row = 1; row <= width / 13.5; ++row ) {
			vertices.push_back (Pixel2(row, col));
		}
	}

	cout << height << " " << width << " " << vertices.size() << endl;

	clock_t start = clock();

	for (int framecount = 1; true; ++framecount) {
		cout << "Frame " << framecount << endl;

		if (read_frame()) break;

		// return status values of calcOpticalFlowPyrLK
		vector<uchar> status;
		vector<float> err;
		vector<Pixel2> vertices_next;

		// run LK for all vertices
		calcOpticalFlowPyrLK(prev_frame, curr_frame, vertices, 
						 vertices_next, status, err, 
						 Size(5,5),3, 
						 TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.1), 
						 10, 1e-4 );


		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	clock_t end = clock();
    const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
    printf("optflow time %lf[ms]\n", time);

	// clean up
	destroyAllWindows();
}
