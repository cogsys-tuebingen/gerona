#ifndef SPEECHOUTPUT_H
#define SPEECHOUTPUT_H

#include <iostream>
//using namespace std;

class SpeechOutput
{
public:
    SpeechOutput();
    void sayText(std::string text);
    void playMp3(std::string text);
    void loadMp3Map();
    void insertMp3Map(std::string key, std::string value);
    void loadInitFile(std::string filename);
};

#endif // SPEECHOUTPUT_H
