
#include "config_modelbasedplanner.h"

template<typename Out>
    void splitT(const std::string &s, char delim, Out result) {
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delim)) {
            *(result++) = item;
        }
    }

    std::vector<std::string> split(const std::string &s, char delim) {
        std::vector<std::string> elems;
        splitT(s, delim, std::back_inserter(elems));
        return elems;
    }
    std::string ModelBasedPlannerConfig::getFolderName(const std::string &s)
    {
        std::string res = "";

        std::vector<std::string> tokens = split(s, '/');

        for (unsigned int tl = 0; tl < tokens.size()-1;tl++)
        {
            res += tokens[tl] + "/";
        }


        return res;
    }
