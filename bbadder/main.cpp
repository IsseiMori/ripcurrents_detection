#include <string>
#include <iostream>
#include <fstream>
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

    VideoCapture video = VideoCapture(video_name);

    double fps = video.get(CAP_PROP_FPS);
    int height = video.get(CAP_PROP_FRAME_HEIGHT);
    int width = video.get(CAP_PROP_FRAME_WIDTH);

    VideoWriter* video_out = new VideoWriter(text_name + "_bb.mp4",
                                             0x7634706d,
                                             fps, Size(width, height), true);

    ifstream file(text_name);

    Mat frame;

    for (;;) {
        video.read(frame);
        if (frame.empty()) break;

        string line;
        getline(file, line);

        vector<string> words = split(line);
        cout << words[0] << endl;

        int num_bb = 0;
        if (words.size() == 5) num_bb = 1;
        else if (words.size() == 9) num_bb = 2;
       
        
        auto iter = words.begin();
        for (int i = 0; i < num_bb; ++i) {
            int ltx = stoi(*(++iter));
            int lty = stoi(*(++iter));
            int rbx = stoi(*(++iter));
            int rby = stoi(*(++iter));

            rectangle(frame, Point(ltx, lty), Point(rbx, rby),
                      Scalar(0,255,0), 2);
        }

        // imshow("a", frame);
        video_out->write(frame);
        if ( waitKey(1) == 27) break;

    }


    video_out->release();
    destroyAllWindows();

    return 0;

}