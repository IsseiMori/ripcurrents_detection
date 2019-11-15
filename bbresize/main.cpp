#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

vector<string> split(const string& str, char delim = ','){
    istringstream iss(str);
    string tmp;
    vector<string> res;
    while (getline(iss, tmp, delim)) res.push_back(tmp);
    return res;
}

int main(int argc, char **argv) 
{
    string video_name = argv[1];
    string text_name = argv[2];

    ifstream file(text_name);

    ofstream outfile;
    outfile.open(text_name+"_resized.txt");

    VideoCapture video = VideoCapture(video_name);

    double orig_h = video.get(cv::CAP_PROP_FRAME_HEIGHT);
    double orig_w = video.get(cv::CAP_PROP_FRAME_WIDTH);

    double now_h = 480;
    double now_w = floor(orig_w * now_h / orig_h);

    double resize_h = orig_h / now_h; 
    double resize_w = orig_w / now_w;


    string line;

    while (getline(file, line)) {

        vector<string> words = split(line);

        int num_bb = 0;
        if (words.size() == 5 || words.size() == 9) num_bb = 1;
       
        
        outfile << words[0];
        auto iter = words.begin();
        for (int i = 0; i < num_bb; ++i) {
            int ltx = stoi(*(++iter));
            int lty = stoi(*(++iter));
            int rbx = stoi(*(++iter));
            int rby = stoi(*(++iter));

            outfile << "," << floor(ltx * resize_w)
                    << "," << floor(lty * resize_h)
                    << "," << floor(rbx * resize_w)
                    << "," << floor(rby * resize_h);
        }

        outfile << endl;
    }


    return 0;

}