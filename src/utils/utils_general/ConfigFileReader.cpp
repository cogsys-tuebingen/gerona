#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include "Strings.h"
#include "ConfigFileReader.h"

ConfigFileReader::ConfigFileReader()
{
}

ConfigFileReader::ConfigFileReader(std::string filename)
{
    Load(filename);
}

void ConfigFileReader::Load(std::string filename)
{
    mMap.clear();

    std::ifstream file(filename.c_str());

    if (file)
    {
        std::string lineWS;

        while (std::getline(file, lineWS))
        {

            std::string line;

            //if quotas exist, do not ignore whitespaces
            if( lineWS[0] != '#' && lineWS.find('"')!=std::string::npos && lineWS.find_first_of('"') < lineWS.find_last_of('"'))
            {

                int pos = lineWS.find_first_of('='), begin = lineWS.find_first_of('"'), end = lineWS.find_last_of('"');
                if (pos>=1) {
                    string k=lineWS.substr(0,pos);
                    Strings::Trim(k);
                    line.append(k);
                    line.append("=");
                    line.append(lineWS.substr(begin + 1, end - begin - 1));
                }
            }
            else
            {
                // Ignore whitespaces
                for (unsigned c = 0; c != lineWS.size(); ++c)
                {
                    if (!std::isspace(lineWS[c]))
                    {
                        line.push_back(lineWS[c]);
                    }
                }
            }
            // Parse line
            if (line.size() > 0 && line[0] != '#')
            {
                int pos = line.find('=');

                if (pos > 0)
                {
                    std::string key = line.substr(0, pos);
                    std::string value;
                    if (pos + 1 < line.size())
                    {
                        value = line.substr(pos + 1);
                    }                  
                    mMap[key] = value;
                }
            }
        }
    }
    else
    {
        std::cerr << "ConfigFileReader error: Cannot find file " << filename << std::endl;
    }
}

void ConfigFileReader::Save(std::string filename)
{
    std::ofstream file(filename.c_str());

    for (std::map<std::string, std::string>::const_iterator it = mMap.begin(); it != mMap.end(); ++it)
    {
        file << it->first << " = " << it->second << std::endl;
    }
}


void ConfigFileReader::SaveMatlab(const std::string &filename,
                                  const std::string &base_key_spec)
{
  std::ofstream outStream(filename.c_str());
  int prevLineNr = -1;
  int lineNr;
  std::string subKey;
  subKey.resize(255);

  // write file header first
  outStream<< "% ";
  for (std::map<std::string, std::string>::const_iterator it = mMap.begin();
       it != mMap.end(); ++it) {
    const string& key=it->first;
    if (subKey.length()<key.length()) {
      subKey.resize(key.length());
    }
    int n=sscanf(key.c_str(),base_key_spec.c_str(),&lineNr,subKey.c_str());
    if (n!=2) {
      cout << "invalid key for matlab-save. key="<<key<< endl;
      continue;
    }
    if (lineNr!=prevLineNr ) {
      if (prevLineNr>=0) {
        outStream<< std::endl;
        break;
      }
      prevLineNr = lineNr;
    }
    outStream<< subKey << " ";
  }
   // write the lines
  for (std::map<std::string, std::string>::const_iterator it = mMap.begin();
       it != mMap.end(); ++it) {
    const string& key=it->first;
    if (subKey.length()<key.length()) {
      subKey.resize(key.length());
    }
    int n=sscanf(key.c_str(),base_key_spec.c_str(),&lineNr,subKey.c_str());
    if (n!=2) {
      cout << "invalid key for matlab-save. key="<<key<< endl;
      continue;
    }
    if (lineNr!=prevLineNr ) {
      if (prevLineNr>=0) {
        outStream<< std::endl;
      }
      prevLineNr = lineNr;
    }
    outStream<< subKey << " ";
  }


}


void ConfigFileReader::Clear()
{
    mMap.clear();
}

bool ConfigFileReader::GetBool(std::string key, bool def) const
{
    std::map<std::string, std::string>::const_iterator it=mMap.find(key);
    if (it == mMap.end())
    {
        return def;
    }

    std::string value = it->second;
    return bool(std::atoi(value.c_str()));
}

int ConfigFileReader::GetInt(std::string key, int def) const
{
    std::map<std::string, std::string>::const_iterator it=mMap.find(key);
    if (it == mMap.end())
    {
        return def;
    }

    std::string value = it->second;
    return std::atoi(value.c_str());
}


void ConfigFileReader::GetIntList(std::string key, IList &ret_values, IList &def) const
{
    std::map<std::string, std::string>::const_iterator it=mMap.find(key);

    if (it == mMap.end())
    {       
       ret_values=def;
       return;
    }
    std::string value = it->second;
    Strings::ToIList(value," ",ret_values);
}



double ConfigFileReader::GetDouble(std::string key, double def) const
{
    std::map<std::string, std::string>::const_iterator it=mMap.find(key);
    if (it == mMap.end())
    {
        return def;
    }

    std::string value = it->second;
    return double(std::atof(value.c_str()));
}

std::string ConfigFileReader::GetString(std::string key, std::string def) const
{
    std::map<std::string, std::string>::const_iterator it=mMap.find(key);
    if (it == mMap.end())
    {
        return def;
    }

    return it->second;

}

std::map<std::string, std::string> ConfigFileReader::GetMap() const
{
    return mMap;
}

void ConfigFileReader::SetBool(std::string key, bool value)
{
    std::stringstream stream;
    stream << value;
    mMap[key] = stream.str();
}

void ConfigFileReader::SetInt(std::string key, int value)
{
    std::stringstream stream;
    stream << value;
    mMap[key] = stream.str();
}

void ConfigFileReader::SetIntList(std::string key, const IList &values)
{
    mMap[key]=Strings::ListToString(values);
}


void ConfigFileReader::SetDouble(std::string key, double value)
{
    std::stringstream stream;
    stream << value;
    mMap[key] = stream.str();
}

void ConfigFileReader::SetString(std::string key, std::string value)
{
    mMap[key] = value;
}


bool ConfigFileReader::KeyExists(const std::string &key, bool fullMatch)
{
  if (fullMatch) {
    // check simply if key is in map
    std::map<std::string, std::string>::const_iterator it=mMap.find(key);
    if (it==mMap.end()) {
      return false;
    } else {
      return true;
    }
  } else {
    int keyLen = key.length();
    for (std::map<std::string, std::string>::const_iterator it=mMap.begin();
         it!=mMap.end();++it) {
      int cmp=it->first.substr(0,keyLen).compare(key);
      if (cmp==0) {
        return true;
      } else if (cmp>0) {
        return false;
      }
    }
  }
  return false;
}
