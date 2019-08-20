#include <string>
#include <math.h>
#include <vector>
#include <time.h>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "fn_timeline.hpp"
#include "fn_grid_buoy.hpp"
#include "fn_histgram.hpp"

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
		case 0: {
			fn_timeline timeline = fn_timeline (video, 480);
			timeline.add_timeline (Pixel2(timeline.width * 0.1, timeline.height * 0.6), 
								Pixel2(timeline.width * 0.9, timeline.height * 0.6), 10);
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
		default: {
			break;
		}
	}

	return 0;
}
