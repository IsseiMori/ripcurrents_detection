#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "method.hpp"

using namespace std;

typedef cv::Point_<float> Pixel2;


class fn_virtual_dyes_line: public method {
	private:

		int vnum;
		vector<Pixel2> roots;
		int birth_rate;
		int max_num;

		class streakline {
			private:
				Pixel2 root;
				vector<Pixel2> vertices;
			public: 
				streakline (Pixel2 _root);

				// run LK method on each vertex and draw lines
				void runLK(Mat& u_prev, Mat& u_curr, Mat& out_img, int max_n, bool isNorm);
				void add(int max_n);

		};

		vector<streakline> streaklines;

	public:
		fn_virtual_dyes_line (string file_name,
        					  int _height, 
        					  int _vnum,
        					  int _birthrate,
        					  int _max_num);
		void run(bool isNorm);
};