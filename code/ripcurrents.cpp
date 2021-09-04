#include <string>
#include <math.h>
#include <vector>
#include <time.h>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "fn_timeline.hpp"
#include "fn_grid_buoy.hpp"
#include "fn_grid_arrow.hpp"
#include "fn_histgram.hpp"
#include "fn_timex.hpp"
#include "fn_dir_color.hpp"
#include "fn_shear.hpp"
#include "fn_pathline.hpp"
#include "fn_LK_all_pixel.hpp"
#include "fn_streakline.hpp"
#include "fn_philip.hpp"
#include "fn_virtual_dyes.hpp"
#include "fn_virtual_dyes_line.hpp"
#include <time.h>

using namespace std;

int main(int argc, char **argv)
{

	string file_name = argv[1];

	int option = 0;
	if (argc >= 3)
	{
		option = stoi(argv[2]);
	}

	clock_t start = clock();

	switch (option)
	{
	// ripcurrents video.mp4 0 v_num(optional)
	// v_num : number of vertices (default 10)
	// born_period : duration of generating new timeline (default 0 = no generation)
	// lifespan : duration of a timeline to die (default 0 = never die)
	// Output file name: infile_timelines_stx_sty_edx_edy_vnum
	case 0:
	{
		cout << "Click two end points of the timeline, then press any key to start" << endl;
		if (argc < 6)
		{
			fn_timeline timeline = fn_timeline(file_name, 480, 20, 0, 0);
			timeline.run(false);
		}
		else if (argc == 6)
		{
			fn_timeline timeline = fn_timeline(file_name, 480, stoi(argv[3]), stoi(argv[4]), stoi(argv[5]));
			timeline.run(false);
		}
		else if (argc > 6)
		{
			fn_timeline timeline = fn_timeline(file_name, 480, stoi(argv[3]), stoi(argv[4]), stoi(argv[5]));
			timeline.run(true);
		}
		break;
	}
	// ripcurrents video.mp4 1 v_count(optional) h_count(optional)
	// v_count h_count : number of buoys in vertical and horizontal (default 10)
	// Output file name : infile_grid_buoy_LK_vcount_hcount
	case 1:
	{
		cout << "Usage: ripcurrents video.mp4 1 v_count(optional) h_count(optional)" << endl;
		if (argc < 4)
		{
			fn_grid_buoy g_buoy = fn_grid_buoy(file_name, 480, 20, 20);
			g_buoy.runLK();
		}
		else if (argc == 5)
		{
			fn_grid_buoy g_buoy = fn_grid_buoy(file_name, 480, stoi(argv[3]), stoi(argv[4]));
			g_buoy.runLK(false);
		}
		else if (argc > 5)
		{
			fn_grid_buoy g_buoy = fn_grid_buoy(file_name, 480, stoi(argv[3]), stoi(argv[4]));
			g_buoy.runLK(true);
		}
		break;
	}
	// ripcurrents video.mp4 2 v_count(optional) h_count(optional)
	// v_count h_count : number of buoys in vertical and horizontal (default 10)
	// Output file name : infile_grid_buoy_FB_vcount_hcount
	case 2:
	{
		cout << "Usage: ripcurrents video.mp4 2 v_count(optional) h_count(optional)" << endl;
		if (argc < 4)
		{
			fn_grid_buoy g_buoy = fn_grid_buoy(file_name, 480, 20, 20);
			g_buoy.runFB();
			// g_buoy.runLK(true);
		}
		else if (argc >= 5)
		{
			fn_grid_buoy g_buoy = fn_grid_buoy(file_name, 480, stoi(argv[3]), stoi(argv[4]));
			// g_buoy.runLK(true);
			g_buoy.runFB();
		}
		break;
	}
	// ripcurrents video.mp4 3
	// Output file name : infile_histogram
	case 3:
	{
		cout << "Usage: ripcurrents video.mp4 3" << endl;
		fn_histgram hist = fn_histgram(file_name, 480);
		hist.run();
		break;
	}
	// ripcurrents video.mp4 4 buffer_size(optional)
	// buffer_size : number of frames to average, 0 for all frames (default all frames)
	// Output file name : infile_timex_buffersize
	case 4:
	{
		fn_timex timex = fn_timex(file_name, 480);
		if (argc < 4)
		{
			timex.run();
		}
		else
		{
			timex.run(stoi(argv[3]));
		}
		break;
	}
	// ripcurrents video.mp4 5 buffer_size(optional)
	// buffer_size : number of frames to average (default 1)
	// Output file name : infile_dir_color_buffersize_color
	// Output file name : infile_dir_color_buffersize_overlay
	case 5:
	{
		fn_dir_color dir_color = fn_dir_color(file_name, 480);
		if (argc < 4)
		{
			dir_color.run();
		}
		else
		{
			dir_color.run(stoi(argv[3]));
		}
		break;
	}
	// ripcurrents video.mp4 6 buffer_size(optional)
	// buffer_size : number of frames to average (default 1)
	// Output file name : infile_dir_only__buffersize_color
	// Output file name : infile_dir_only__buffersize_overlay
	case 6:
	{
		fn_dir_color dir_color = fn_dir_color(file_name, 480);
		if (argc < 4)
		{
			dir_color.run_dir();
		}
		else
		{
			dir_color.run_dir(stoi(argv[3]));
		}
		break;
	}
	// ripcurrents video.mp4 7 buffer_size(optional)
	// buffer_size : number of frames to average (default 1)
	// Output file name : infile_norm_color_buffersize_color
	// Output file name : infile_norm_color_buffersize_overlay
	case 7:
	{
		fn_dir_color dir_color = fn_dir_color(file_name, 480);
		if (argc < 4)
		{
			dir_color.run_norm();
		}
		else
		{
			dir_color.run_norm(stoi(argv[3]));
		}
		break;
	}
	// ripcurrents video.mp4 8 buffer_size(optional)
	// buffer_size : number of frames to average (default 1)
	// offset :
	// normalize :
	case 8:
	{
		cout << strcmp(argv[5], "0") << endl;
		fn_shear shear = fn_shear(file_name, 480);
		shear.run(stoi(argv[3]), stoi(argv[4]), strcmp(argv[5], "0") != 0);
		break;
	}
	// ripcurrents video.mp4 9
	// Output file name : infile_pathline
	// Output file name : infile_pathline
	case 9:
	{
		fn_pathline pathline = fn_pathline(file_name, 480);
		if (argc < 4)
		{
			pathline.runLK(10);
		}
		else if (argc == 4)
		{
			pathline.runLK(stoi(argv[3]), false);
		}
		else
		{
			pathline.runLK(stoi(argv[3]), true);
		}
		break;
	}
	case 10:
	{
		//fn_LK_all_pixel lk = fn_LK_all_pixel (file_name, 480);
		//lk.justrun ();

		fn_dir_color dir = fn_dir_color(file_name, 480);
		dir.justrun();
		break;
	}
	case 11:
	{
		// ripcurrents video.mp4 11 v_num born_dist max_num
		// v_num : number of streamlines
		// born_dist : min distance to birth the next vertex
		// max_num : max number of vertices in a streamline. Exceeding this will be removed
		fn_streakline streakline = fn_streakline(file_name, 480, stoi(argv[3]), stof(argv[4]), stoi(argv[5]));
		streakline.run(true);
		break;
	}

	// ripcurrents video.mp4 12 v_count(optional) h_count(optional)
	// v_count h_count : number of buoys in vertical and horizontal (default 10)
	// Output file name : infile_grid_buoy_LK_vcount_hcount
	case 12:
	{
		cout << "Usage: ripcurrents video.mp4 1 v_count(optional) h_count(optional)" << endl;
		fn_grid_arrow g_arrow = fn_grid_arrow(file_name, 480, stoi(argv[3]), stoi(argv[4]), stoi(argv[5]));
		g_arrow.runLK(true);
		break;
	}
	// Philip
	//   9
	// 0   18
	//   27
	case 13:
	{
		fn_philip philip = fn_philip(file_name, 480);
		philip.run(90, stoi(argv[3]), stoi(argv[4]));
		break;
	}
	// Virtual Dyes
	// int _birth_Rate
	// int _max_num
	// float _draw_r
	// float opacity
	// float dt
	case 14:
	{
		fn_virtual_dyes vd = fn_virtual_dyes(file_name, 480,
											 stoi(argv[3]),
											 stof(argv[4]),
											 stoi(argv[5]),
											 stof(argv[6]),
											 stof(argv[7]));
		vd.run(true);
		break;
	}
	// Virtual Dyes Line
	// int vnum
	// int birth_rate
	// int max_num
	// float opacity
	// float dt
	case 15:
	{
		fn_virtual_dyes_line vdl = fn_virtual_dyes_line(file_name, 480,
														stoi(argv[3]),
														stof(argv[4]),
														stoi(argv[5]),
														stof(argv[6]),
														stof(argv[7]));
		vdl.run(true);
		break;
	}
	case 16:
	{
		float data[2][2] = {{0, 1}, {-2, -3}};
		Mat m = (Mat_<float>(2, 2) << 0, -2, -2, -3);
		cout << m << endl;
		Mat e, v;
		eigen(m, e, v);
		cout << e << endl;
		cout << e.at<float>(0) << endl;
		cout << e.at<float>(1) << endl;
		cout << v << endl;

		float emax = max(abs(e.at<float>(0)), abs(e.at<float>(1)));
		float emin = min(abs(e.at<float>(0)), abs(e.at<float>(1)));
		float eratio = emax / emin;

		cout << emax << endl;
		cout << emin << endl;
		cout << eratio << endl;

		break;
	}
	// ripcurrents video.mp4 7 buffer_size(optional)
	// buffer_size : number of frames to average (default 1)
	// Output file name : infile_norm_color_buffersize_color
	// Output file name : infile_norm_color_buffersize_overlay
	case 17:
	{
		fn_dir_color dir_color = fn_dir_color(file_name, 480);
		if (argc >= 5)
		{
			dir_color.run_norm_filter(stoi(argv[3]), stoi(argv[4]));
		}
		break;
	}
	default:
	{
		break;
	}
	}

	clock_t end = clock();
	const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
	printf("time %lf[ms]\n", time);

	return 0;
}
