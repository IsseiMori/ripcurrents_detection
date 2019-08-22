#include <string>
#include <math.h>
#include <vector>
#include <time.h>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "fn_timeline.hpp"
#include "fn_grid_buoy.hpp"
#include "fn_histgram.hpp"
#include "fn_timex.hpp"
#include "fn_dir_color.hpp"

using namespace std;

int main(int argc, char **argv)
{

	string file_name = argv[1];

	int option = 0;
	if (argc >= 3)
	{
		option =  stoi(argv[2]);
	}

	switch (option) {
		// ripcurrents video.mp4 0 v_num(optional)
		// v_num : number of vertices (default 10)
		// Output file name: infile_timelines_stx_sty_edx_edy_vnum
		case 0: {
			cout << "Click two end points of the timeline, then press any key to start" << endl;
			fn_timeline timeline = fn_timeline (file_name, 480);
			if (argc < 4 ) {
				timeline.run();
			}
			else {
				timeline.run(stoi (argv[3]));
			}
			break;
		}
		// ripcurrents video.mp4 1 v_count(optional) h_count(optional)
		// v_count h_count : number of buoys in vertical and horizontal (default 10)
		// Output file name : infile_grid_buoy_LK_vcount_hcount
		case 1: {
			cout << "Usage: ripcurrents video.mp4 1 v_count(optional) h_count(optional)" << endl;
			if (argc < 4) {
				fn_grid_buoy g_buoy = fn_grid_buoy (file_name, 480);
				g_buoy.runLK();
			}
			else if (argc >= 5){
				fn_grid_buoy g_buoy = fn_grid_buoy (file_name, 480, stoi (argv[3]), stoi (argv[4]));
				g_buoy.runLK();
			}
			break;
		}
		// ripcurrents video.mp4 2 v_count(optional) h_count(optional)
		// v_count h_count : number of buoys in vertical and horizontal (default 10)
		// Output file name : infile_grid_buoy_FB_vcount_hcount
		case 2: {
			cout << "Usage: ripcurrents video.mp4 2 v_count(optional) h_count(optional)" << endl;
			if (argc < 4) {
				fn_grid_buoy g_buoy = fn_grid_buoy (file_name, 480);
				g_buoy.runFB();
			}
			else if (argc >= 5){
				fn_grid_buoy g_buoy = fn_grid_buoy (file_name, 480, stoi (argv[3]), stoi (argv[4]));
				g_buoy.runFB();
			}
			break;
		}
		// ripcurrents video.mp4 3
		// Output file name : infile_histogram
		case 3: {
			cout << "Usage: ripcurrents video.mp4 3" << endl;
			fn_histgram hist = fn_histgram (file_name, 480);
			hist.run();
			break;
		}
		// ripcurrents video.mp4 4 buffer_size(optional)
		// buffer_size : number of frames to average, 0 for all frames (default all frames)
		// Output file name : infile_timex_buffersize
		case 4: {
			fn_timex timex = fn_timex (file_name, 480);
			if (argc < 4 ) {
				timex.run ();
			}
			else {
				timex.run (stoi (argv[3]));
			}
			break;
		}
		// ripcurrents video.mp4 5 buffer_size(optional)
		// buffer_size : number of frames to average (default 1)
		// Output file name : infile_dir_color_buffersize
		case 5: {
			fn_dir_color dir_color = fn_dir_color (file_name, 480);
			if (argc < 4 ) {
				dir_color.run ();
			}
			else {
				dir_color.run (stoi (argv[3]));
			}
			break;
		}
		// ripcurrents video.mp4 6 buffer_size(optional)
		// buffer_size : number of frames to average (default 1)
		// Output file name : infile_dir_only__buffersize
		case 6: {
			fn_dir_color dir_color = fn_dir_color (file_name, 480);
			if (argc < 4 ) {
				dir_color.run_dir ();
			}
			else {
				dir_color.run_dir (stoi (argv[3]));
			}
			break;
		}
		default: {
			break;
		}
	}

	return 0;
}
