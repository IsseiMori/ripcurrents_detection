#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include <fstream> 
#include <time.h>

#include <opencv2/opencv.hpp>

#include "fn_dir_color.hpp"

using namespace std;

fn_dir_color::fn_dir_color (string _file_name, 
							int _height)
							: method(_file_name, _height) {
}


void fn_dir_color::justrun () {
	cout << "Running color map" << endl;

	ini_frame();

	clock_t start = clock();

	for (int framecount = 1; true; ++framecount) {
		cout << "Frame " << framecount << endl;
		if (read_frame()) break;

		calc_FB ();

		if ( waitKey(1) == 27) break;

	}

	clock_t end = clock();
    const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
    printf("optflow time %lf[ms]\n", time);

	// clean up
	destroyAllWindows();

}

void fn_dir_color::run (int buffer_size) {
	cout << "Running color map" << endl;

	VideoWriter* video_output_color = ini_video_output (file_name + "_dir_color_" + to_string(buffer_size) + "_color");
	VideoWriter* video_output_overlay = ini_video_output (file_name + "_dir_color_" + to_string(buffer_size) + "_overlay");

	ini_frame();
	ini_buffer(buffer_size);
	ini_draw_colorwheel ();

	for (int framecount = 1; true; ++framecount) {

		if (read_frame()) break;

		calc_FB ();

		Mat out_img;
		Mat out_img_overlay;
		resized_frame.copyTo(out_img);
		resized_frame.copyTo(out_img_overlay);

		eliminate_std (5);

		update_buffer (buffer_size);

		vector_to_color (average_flow, out_img);
		drawFrameCount(out_img, framecount);

		addWeighted( out_img, 1.0, out_img_overlay, 1.0, 0.0, out_img_overlay);

		draw_colorwheel (out_img);
		draw_colorwheel (out_img_overlay);
		
		
		imshow ("grid_buoy color map", out_img);
		imshow ("grid_buoy overlay", out_img_overlay);
		video_output_color->write (out_img);
		video_output_overlay->write (out_img_overlay);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output_color->release();
	video_output_overlay->release();
	destroyAllWindows();

}

void fn_dir_color::run_norm (int buffer_size) {
	cout << "Running color map" << endl;

	VideoWriter* video_output_color = ini_video_output (file_name + "_norm_color_" + to_string(buffer_size) + "_color");
	VideoWriter* video_output_overlay = ini_video_output (file_name + "_norm_color_" + to_string(buffer_size) + "_overlay");

	ini_frame();
	ini_buffer(buffer_size);
	ini_draw_colorwheel ();

	for (int framecount = 1; true; ++framecount) {

		if (read_frame()) break;

		calc_FB ();

		Mat out_img;
		Mat out_img_overlay;
		resized_frame.copyTo(out_img);
		resized_frame.copyTo(out_img_overlay);

		eliminate_std(5);
		normalize_flow();

		update_buffer (buffer_size);

		vector_to_color (average_flow, out_img);
		drawFrameCount(out_img, framecount);


		addWeighted( out_img, 1.0, out_img_overlay, 1.0, 0.0, out_img_overlay);

		draw_colorwheel (out_img);
		draw_colorwheel (out_img_overlay);
		
		
		imshow ("grid_buoy color map", out_img);
		imshow ("grid_buoy overlay", out_img_overlay);
		video_output_color->write (out_img);
		video_output_overlay->write (out_img_overlay);

		curr_frame.copyTo (prev_frame);
		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output_color->release();
	video_output_overlay->release();
	destroyAllWindows();

}

void fn_dir_color::run_dir (int buffer_size) {
	cout << "Running direction only color map" << endl;

	VideoWriter* video_output_color = ini_video_output (file_name + "_dir_only_color_" + to_string(buffer_size) + "_color");
	VideoWriter* video_output_overlay = ini_video_output (file_name + "_dir_only_color_" + to_string(buffer_size) + "_overlay");

	ini_frame();
	ini_buffer(buffer_size);
	ini_draw_colorwheel ();

	for (int framecount = 1; true; ++framecount) {

		if (read_frame()) break;

		calc_FB ();

		Mat out_img;
		Mat out_img_overlay;
		resized_frame.copyTo(out_img);
		resized_frame.copyTo(out_img_overlay);

		update_buffer (buffer_size);

		vector_to_dir_color (average_flow, out_img);
		drawFrameCount(out_img, framecount);

		addWeighted( out_img, 0.5, out_img_overlay, 0.5, 0.0, out_img_overlay);	

		draw_colorwheel (out_img);
		draw_colorwheel (out_img_overlay);	
		
		imshow ("grid_buoy color map", out_img);
		imshow ("grid_buoy overlay", out_img_overlay);
		video_output_color->write (out_img);
		video_output_overlay->write (out_img_overlay);

		curr_frame.copyTo (prev_frame);

		if ( waitKey(1) == 27) break;

	}

	// clean up
	video_output_color->release();
	video_output_overlay->release();
	destroyAllWindows();
}