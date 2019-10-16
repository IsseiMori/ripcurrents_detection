#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "method.hpp"

using namespace std;

typedef cv::Point_<float> Pixel2;


class fn_streakline: public method {
	private:

		int vnum;
		vector<Pixel2> roots;
		float born_distance;
		int max_num;

		class streakline {
			private:
				Pixel2 root;
				vector<Pixel2> vertices;
			public: 
				streakline (Pixel2 _root);

				// run LK method on each vertex and draw lines
				void runLK(Mat& u_prev, Mat& u_curr, Mat& out_img, float born_d, int max_n, bool isNorm);

		};

		vector<streakline> streaklines;

	public:
		fn_streakline (string file_name,
					 int _height, 
					 int _vnum,
					 float _born_distance,
					 int _max_num);
		void run(bool isNorm);
};