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
		float opacity;
		float dt;

		// For grid arrows to detect the incoming direction
		vector<Pixel2> root_vec;
		vector<Pixel2> v_vec;
		vector<Pixel2> relative_vec;
		vector<float> theta_vec;
		vector<bool> isVisible;
		vector<float> howLikely; // 0 (no rip) to 1 (rip)
		vector<vector<Pixel2>> vec_buffer;
		vector<Pixel2> average_vec;
		int w_count;
		int h_count;
		float max_len;
		int buffer_size;

		class streakline {
			private:
				Pixel2 root;
				vector<Pixel2> vertices;
			public: 
				streakline (Pixel2 _root);

				// run LK method on each vertex and draw lines
				void runLK(Mat& u_prev, Mat& u_curr, Mat& out_img, int max_n, bool isNorm, float opacity, float dt);
				void add(int max_n);
				void vertices_filter(int max_id);

		};

		vector<streakline> streaklines;

	public:
		fn_virtual_dyes_line (string file_name,
        					  int _height, 
        					  int _vnum,
        					  int _birthrate,
        					  int _max_num,
							  float _opacity,
							  float _dt);
		void run(bool isNorm);
		void vertices_runLK (Mat u_prev, Mat u_curr, Mat& out_img, bool isNorm, float framecount);
		int filter_frequency6 ();
};