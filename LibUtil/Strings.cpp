
/**

    @author Karsten Bohlmann


*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Strings.h"

#include <sstream>

// white space is also the not so often used 
// Vertical tab     VT  11  \v
// Carriage return  CR  13  \r
// Formfeed         FF  12  \f
const int BUFFER_LEN=32768;
const char * Strings::WHITE_SPACES_ALL = " \t\n\v\r\f";
const char * Strings::WHITE_SPACES_DELIMITERS = " \t";


void Strings::SplitFileName (const string& fname, string& path, string& file)
{
    // look for path delimiter in filename
    int pos = fname.find_last_of("/\\");
    if (pos != string::npos)
    {
        // extract path
        path = fname.substr(0, pos + 1);
        file = fname.substr(pos+1,fname.length()-pos-1);
    }
    else
    {       
        path.erase();
        file.assign(fname);
    }
}


void Strings::ConcatFileName (string& out_fname, const string& path,
    const string& fname)
{
    if (fname.empty()) {
        out_fname=path;
    } else if (path.empty()) {
        out_fname=fname;

    } else {
        char ff = fname.at(0);
        if ((ff=='/')||(ff=='\\')) {
            out_fname=fname;
            return;
        }
        char lp = path.at(path.length()-1);
        out_fname.assign(path);
        if (!((lp=='/')||(lp=='\\'))) {
            out_fname.append("/");
        }
        out_fname.append(fname);
    }
}


void Strings::ConcatPath (string& out_path, const string& path,
    const string& sub_dir)
{
    if (path.empty()) {
        out_path = sub_dir;
    } else {
        //Retrieve the last character in the basepath
        char lp = path.at(path.length()-1);

        //Assign the basepath
        out_path.assign(path);

        //Append a path separator if necessary
        if (!((lp=='/')||(lp=='\\'))) {
            out_path.append("/");
        }

        //Append the subdirectory name
        out_path.append(sub_dir);
    }
}


int Strings::Find (const StringVector& line, const char *value) {
    int pos=0;
    bool found=false;
    while (pos<line.size() && !found) {
        if (line[pos]==value) 
            found =true;
        else
            ++pos;
    }
    if (!found)
        pos=-1;
    return pos;
}


string& Strings::Trim (string& line)
{
    int pos1 = line.find_first_not_of(WHITE_SPACES_ALL);
    if (pos1==string::npos) {
        line.assign("");
    } else {
        int pos2 = line.find_last_not_of(WHITE_SPACES_ALL);
        line = line.substr(pos1,pos2-pos1+1);
    }
    return line;
}


string&  Strings::Trim (string& line, const char *trash)
{
    int pos1 = line.find_first_not_of(trash);
    if (pos1!=string::npos) {
        line.erase(0,pos1);
    } else {
        line.erase();
        return line;
    }
    int pos2 = line.find_last_not_of(trash);
    if ((pos2!=line.length()-1)&&(pos2!=string::npos)) {
        line.erase(pos2+1);
    }
    return line;
}



string& Strings::TrimRight(string &line)
{

    int ppos;
    //trim white spaces at right side of strline 
    if( ((ppos = line.find_last_not_of(WHITE_SPACES_ALL)) != string::npos) && (ppos+1 != line.length()))
    {
        line.erase(ppos+1);
    }

    return line;
}

//string left trim
string& Strings::TrimLeft(string &line)
{
    int ppos;
    //trim white spaces at left side of strline 
    if( (ppos = line.find_first_not_of(WHITE_SPACES_ALL)) != 0 )
    {
        line.erase(0,ppos);
    }

    return line;
}


string Strings::ListToString (const IList& int_list)
{
    int len=int_list.size();
    // INT_MIN=-2147483648 -> 11digits +1space per number + 1 for \0
    char *result = new char[len*12+1];
    char *resptr = result;
    char buffer[15];
    for (IList::const_iterator iter=int_list.begin();iter!=int_list.end();
        ++iter) {
        sprintf(buffer,"%d ",*iter);
        strcpy(resptr,buffer);
        resptr+=strlen(buffer);
    }
    string resultStr;
    resultStr.assign(result);
    return resultStr;
}

string& Strings::Append(string& str, int val)
{
    string sval;
    Assign(sval,val);
    str.append(sval);
    return str;
}


string& Strings::Assign(string& str, const char* fortran_str,
    int len)
{
    char *cstr=new char[len+1];
    memcpy(cstr,fortran_str,len);
    cstr[len]=0;
    str.assign(cstr);
    delete[] cstr;
    return str;
}


string& Strings::PadRight(string& str, Uint width, char pad)
{
    if (width<=str.length()) {
        return str;
    }
    str.resize(width,pad);
    return str;
}


string& Strings::PadLeft(string& str, Uint width, char pad)
{
    if (width<=str.length()) {
        return str;
    }
    char *buffer=new char[width+1];

    int offset = width - str.length();
    memset(buffer, pad, offset);
    buffer[width] = '\0';

    strcpy(buffer + offset, str.c_str());
    str.assign(buffer);

    delete [] buffer;
    return str;
}


int Strings::ToInt (const string& str, int& result)
{
    int n=sscanf(str.c_str(),"%d",&result);
    if (n!=1) {
        // conversion failed
        return ENOTFOUND;
    } else {
        // conversion successful
        return EOK;
    }
}


int Strings::ToS64 (const string& str, S64& result)
{
    /**
#ifdef WIN32
    int n = sscanf(str.c_str(),"%I64d", &result);
#else
    int n = sscanf(str.c_str(),"%ld", &result);
#endif
    if (n != 1) {
        // conversion failed
        return ENOTFOUND;
    } else {
        // conversion successful
        return EOK;
    }
    */
}


int Strings::ToDouble (const string& str, double& result)
{
    int pos;
    string c;
    const char *buf;
    if( (pos=str.find('d')) != string::npos  || (pos=str.find('D')) != string::npos ) 
    {
        c.assign(str);
        c[pos] = 'E';
        buf = c.c_str();
    } else {
        buf = str.c_str();
    }

    int n=sscanf(buf,"%lf",&result);
    if (n!=1) {
        result = 0.0;
        // conversion failed
        return ENOTFOUND;
    } else {
        // conversion successful
        return EOK;
    }
}


int Strings::ToDoubleVec (const string& str, DVector& result,bool clear_result)
{
    char *token;
    if (clear_result) {
        result.clear();
    }
    int n=0;
#ifdef WIN32
    char *source_str = _strdup(str.c_str());
#else
    char *source_str = strdup(str.c_str());
#endif
    if (!source_str) {
        //("Out of memory");
        return ESYSTEM;
    }
    token = strtok(source_str, " \t" );
    while (token) {
        // While there are tokens in "string" 
        double value;
        int m=sscanf(token,"%lf",&value);
        if (m==1) {
            result.push_back(value);
        } else {
            return EINVALID;
        }
        ++n;
        // Get next token 
        token = strtok( NULL, " \t" );
    }
    free (source_str);
    return n;   
}


int Strings::ToIList (const string& str, IList& result)
{
    return ToIList(str, WHITE_SPACES_DELIMITERS, result);
}


int Strings::ToIList (const string& str, const char *seps, IList& result)
{
    char *token;

    int n=0;
#ifdef WIN32
    char *source_str = _strdup(str.c_str());
#else
    char *source_str = strdup(str.c_str());
#endif
    if (!source_str) {
        //("Out of memory");
        return ESYSTEM;
    }
    token = strtok(source_str, seps );
    while (token) {
        // While there are tokens in "string" 
        int value;
        int m=sscanf(token,"%d",&value);
        if (m==1) {
            result.push_back(value);
        } else {
            return EINVALID;
        }
        ++n;
        // Get next token 
        token = strtok( NULL, seps );
    }
    free (source_str);
    return n;   
}


int Strings::Split(const string& line, const char *seps,
    const char *trash, StringList& res_tokens)
{
    char *token;

    int ntoken = 0;
    char * str = new char[line.length()+1];
    strcpy(str, line.c_str());

    token = strtok( str, seps );

    // While there are tokens in "string"
    while( token != 0 )
    {
        string tok(token);
        Trim(tok,trash);
        res_tokens.push_back(tok);
        // Get next token: 
        token = strtok( NULL, seps );
    }   
    delete[] str;
    return ntoken;
}


int Strings::Split(const string& line, const char *seps,
    StringVector& res_tokens)
{
    char *token;

    char * str = new char[line.length()+1];
    strcpy(str, line.c_str());
    res_tokens.clear();
    token = strtok( str, seps );

    // While there are tokens in "string"
    while( token != 0 )
    {
        res_tokens.push_back(token);
        // Get next token: 
        token = strtok( NULL, seps );
    }   
    delete[] str;
    return res_tokens.size();
}


int Strings::SplitPair(const string& line, const char* seps, pair<string,string>& result)
{
    int retval = EOK;

    //Find the first occurance of a separator character
    size_t index = line.find_first_of(seps);

    //Check if separator was found
    if ( index != string::npos ) {

        //Split the passed line and throw away the separator character
        result.first = line.substr(0, index);
        result.second = line.substr(index+1, line.length()-index+1);
    } else {
        //Assign the line to the first value and leave the second empty
        result.first = line;
        result.second = "";
        retval = ENOTFOUND;
    }

    return retval;
}


bool Strings::IsLetter (char c)
{
    return (((c>='A')&&(c<='Z'))||
        ((c>='a')&&(c<='z')))?true:false;
}


bool Strings::IsDigit (char c)
{
    return ((c>='0')&&(c<='9'))?true:false;
}


int Strings::Hash (const string& str)
{
    int hash = 2054435761;
    for (int i=0;i<str.length();++i) {
        hash ^=str[i];
        hash <<= 1;
    }
    return hash;
}


string Strings::ToUpper(const string& s)
{
    int i;
    string res;

    res.resize(s.length());

    for (i = 0; i < s.length(); ++i) {
        res[i] = toupper(s[i]);
    }
    return res;
}


string Strings::ToLower(const string& s)
{
    int i;
    string res;

    res.resize(s.length());

    for (i = 0; i < s.length(); ++i) {
        res[i] = tolower(s[i]);
    }
    return res;
}


string Strings::BinToString (U8 val)
{
    string res(" 0000 0000");
    for (int n=1;n<=9;++n) {
        if (n!=5) {
            if (val & 0x80) {
                res[n]='1';
            }
            val = val << 1;
        }   
    }   
    return res;
}


string Strings::BinToString (int val)
{
    string res("                                        ");
    for (int b=0;b<4;++b) {
        string part=BinToString((U8)(val & 0xff));
        res.replace((3-b)*10, 10, part);
        val = val >> 8;
    }
    return res;
}


string Strings::AppendSuffix(const string& basename, const string& suffix)
{
    string result(basename);
    int s_len=suffix.length();
    int b_len=basename.length();
    if ((s_len>b_len)||(basename.substr(b_len-s_len,s_len).compare(suffix))) {
        result.append(suffix);
    }
    return result;
}


bool Strings::StartsWith(const string& str, const string& prefix)
{
    return !str.substr(0,prefix.length()).compare(prefix);
}
