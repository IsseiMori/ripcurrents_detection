#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

using namespace std;

vector<string> split(const string& str, char delim = ','){
    istringstream iss(str);
    string tmp;
    vector<string> res;
    while (getline(iss, tmp, delim)) res.push_back(tmp);
    return res;
}

int main(int argc, char **argv) 
{
    string text_name = argv[1];
    ifstream file(text_name);

    ofstream outfile;
    outfile.open(text_name+"_1.txt");


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

            outfile << "," << ltx 
                    << "," << lty 
                    << "," << rbx 
                    << "," << rby;
        }

        outfile << endl;
    }

    return 0;

}