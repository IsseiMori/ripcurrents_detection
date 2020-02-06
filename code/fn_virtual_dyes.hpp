#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "method.hpp"

using namespace std;

typedef cv::Point_<float> Pixel2;


class fn_virtual_dyes: public method {
	public:

		vector<Pixel2> roots;
		int birth_rate;
        int max_num;
        float draw_r;
		float opacity;
		float dt;

		class Dyes {
            public: 
                Pixel2 root;
				vector<Pixel2> vertices;

                Dyes();

                void add(int max_n);

				// run LK method on each vertex and draw lines
				void runLK(Mat& u_prev, Mat& u_curr, Mat& out_img, float r, bool isNorm, float opacity, float dt);

		};

		Dyes dyes;

		fn_virtual_dyes (string file_name,
					 int _height, 
					 int _birth_rate,
                     int _max_num,
                     float _draw_r,
					 float _opacity,
					 float _dt);
		void run(bool isNorm);
};