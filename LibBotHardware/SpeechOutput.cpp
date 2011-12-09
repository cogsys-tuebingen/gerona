#include "SpeechOutput.h"
#include "ConfigFileReader.h"

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <cstdlib>
#include <fstream>
#include <sstream>


using namespace std;

map<string, string> mp3Map;
string speechProgram;
string audioProgram;
string mp3Dir;

SpeechOutput::SpeechOutput()
{
    this->loadMp3Map();
    //speechProgram = "flite -t "; // "espeak -v en-us -s 130 -g 12 -p 30 "
    //mp3Dir = "/home/robot/Desktop/speech/mp3s/";

    string configFileReaderPath = "/home/robot/Desktop/speech/mp3s/SpeechConfig";
    ConfigFileReader cFR = ConfigFileReader(configFileReaderPath);

    speechProgram = cFR.GetString("speechProgram_", "flite -t");
    mp3Dir = cFR.GetString("mp3Dir", "$HOME/Desktop/speech/mp3s/");
    audioProgram = cFR.GetString("audioProgram_", "cvlc --play-and-exit");
}



void SpeechOutput::sayText(string text){
    string out = speechProgram;;

    out.append(" \"");
    out.append(text);
    out.append("\" &");

    system(out.c_str());
}

void SpeechOutput::playMp3(string text){
    string out = audioProgram;

    out.append(" ");
    out.append(mp3Dir);
    out.append(" \"");
    out.append(mp3Map[text]);
    out.append("\" &");

    system(out.c_str());
}

void SpeechOutput::loadMp3Map(){

    insertMp3Map( "wetter", "wetta.wav" );
    insertMp3Map( "benderkill", "Kill All Humans.flv" );
    insertMp3Map( "hastalavista", "WaTerminator.flv" );
    insertMp3Map( "illbeback", "_I_ll be back_ 2 (Jim Sweet) animation cartoon rough flip.flv" );
    insertMp3Map( "talktothehand", "Talk to the hand.flv" );
    insertMp3Map( "hastalavista2", "Terminator 2_ Judgement Day - Montage Clip.flv" );

}

void SpeechOutput::insertMp3Map(string key, string value){
    mp3Map.insert( map<string, string>::value_type( key, value ) );
}


void SpeechOutput::loadInitFile(string filename)
{
    map<string, string> configMap;

    ifstream file(filename.c_str());
    string lineWS;

    while (getline(file, lineWS))
    {
        string line;

        // Parse line
        if (line.size() > 0 && line[0] != '#')
        {
            int pos = line.find('=');

            if (pos > 0)
            {
                string key = line.substr(0, pos);
                string value;
                if (pos + 1 < line.size())
                {
                    value = line.substr(pos + 1, line.size());
                }
                configMap[key] = value;
            }
        }
    }
}
