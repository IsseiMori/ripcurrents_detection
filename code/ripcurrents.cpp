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
	cv::VideoCapture video;
	video = cv::VideoCapture(argv[1]);
	if (!video.isOpened())
	{
		cout << argv[1] << "File not found" << endl;
		exit(1);
	}

	int option = 0;
	if (argc >= 3)
	{
		option =  stoi(argv[2]);
	}

	switch (option) {
		// ripcurrents video.mp4 0 v_num
		// v_num : number of vertices
		case 0: {
			int v_num = stoi (argv[3]);
			cout << "Click two end points of the timeline, then press any key to start" << endl;
			fn_timeline timeline = fn_timeline (video, 480, v_num);
			timeline.run();
			break;
		}
		case 1: {
			fn_grid_buoy g_buoy = fn_grid_buoy (video, 480, 10, 10);
			g_buoy.runLK();
			break;
		}
		case 2: {
			fn_grid_buoy g_buoy = fn_grid_buoy (video, 480, 20, 20);
			g_buoy.runFB();
			break;
		}
		case 3: {
			fn_histgram hist = fn_histgram (video, 480);
			hist.run();
			break;
		}
		case 4: {
			fn_timex timex = fn_timex (video, 480);
			timex.run ();
			break;
		}
		case 5: {
			fn_dir_color dir_color = fn_dir_color (video, 480);
			dir_color.run (100);
			break;
		}
		default: {
			break;
		}
	}

	return 0;
}
