#ifndef STRINGS_H
#define STRINGS_H
#include <sstream>
#include "Global.h"
/**
    @author Karsten Bohlmann
    general utilities for strings

*/

class Strings
{
public:

    /**
        splits the given string into tokens and trims the tokens
        @param line string to be split
        @param seps separator chars
        @param trash chars which are left/right removed from token
        @param res_tokens resulting ist of tokens
        @return number of tokens
    */
    static int Split(const string& line, const char *seps, const char *trash,
        StringList& res_tokens);

    /**
        splits the given string into tokens
        @return number of tokens
    */
    static int Split(const string& line, const char *seps,
        StringVector& res_tokens);

    /**
        Splits the passed line into two values at the first found seperator.
        If a separator is found as first charactor, the first entry of the resulting
        pair will be the empty string and the second will contain the line without the
        leading character. If the first found separator is the last character the
        result will be the opposite.
        If no seperate is found the first value will hold the complete line and
        the second will be empty. In this case additionally the returned value will
        indicate an error.
        @param[in] line the string value to split into a pair of strings.
        @param[in] seps sepearators to look for.
        @param[out] result the splitted pair.
        @return STATUS_OK is successfully splitted, otherwise an appropriate errorcode
    */
    static int SplitPair(const string& line, const char* seps, pair<string,string>& result);

    /**
        Splits the given string <fname> in <path> and <filename>.
        If the passed path is empty the filename is return as passed to the method.
        If the filename is empty the path is returned accordingly.
        @param[in,out] fname the create filename
        @param[in] path the base path
        @param[in] file the element appended to the base path.
    */
    static void	SplitFileName (const string& fname, string& path, 
        string& file);

    /**
        Concats path and fname to absolute path.

        @param[in,out] fname the create filename
        @param[in] path the base path
        @param[in] file the element appended to the base path.
    */
    static void	ConcatFileName (string& out_fname, const string& path,
        const string& fname);


    /**
        Concatenates the passed paths to a single directory path.
        The method works with an absolute basepath as well as with a relative.
        @param[out] out_path the concatenated path name.
        @param[in] path the base path.
        @param[in] sub_dir the directory name that should be appended to the base path.
        @return STATUS_OK if method succeeds otherwise an appropriate errorcode.
    */
    static void ConcatPath (string& out_path, const string& path, const string& sub_dir);


    /**
        Concatenates the passed paths to a single absolute path.
        The method works with an absolute basepath as well as with a relative.
        @param[out] out_path the concatenated absolut path name.
        @param[in] path the base path
        @param[in] sub_dirs vector containing the subdirectory names that should be appended.
        @return STATUS_OK if method succeeds otherwise an appropriate errorcode.
    */
    static void ConcatPath (string& out_path, const string& path, const vector<string>& sub_dirs);


    /**
        Find String in StringVector and return position. 
        -1 if not found
    */ 
    static int      Find(const StringVector& line, const char *value);

    /**
        removes trailing whitespace from line
    */ 
    static string&  TrimRight (string &line);

    /**
        removes  preceding whitespace from line
    */ 
    static string&  TrimLeft (string &line);

    /**
        removes left and right white space (' ','\t','\n')
        from line
    */
    static string&  Trim (string &line);

    /**
        removes from left and right all characters listed in <code>trash</code>
        from line       
    */
    static string&  Trim (string &line, const char *trash);

    static string& PadLeft(string& str, Uint width, char pad = ' ');

    /**
        pads the given string with characters (default blank)
        on the right side
        up to given width, do nothing if string.length>=width
        @return the padded string
    */
    static string&  PadRight(string& str, Uint width, char pad=' ');

    static string&  Append(string& str, int val);
    /**
        converts a fortran string (not null terminated) to a c++ string

    */
    static string&  Assign (string& str, const char *fortran_str, int len);


    template <class T>
            static inline std::string Assign (std::string& str, const T& t)
    {
        std::ostringstream ss;
        ss << t;
        str.assign(ss.str());
        return str;
    }

    template <class T>
    static inline std::string ToString (const T& t)
    {
        std::ostringstream ss;
        ss << t;
        return ss.str();
    }


    /**
        converts given list of integers to string with numbers separated
        by space
    */
    static string   ListToString (const IList& int_list);

    /**
        converts string to integer
        (the std fun atoi() does the same, but without error detection)
        @return STATUS_OK or ERR_NOT_FOUND (if conversion failed)
    */
    static int      ToInt (const string& str, int& result);


    /**
        Converts a string into a 64bit integer value.
        Intepretes until the first invalid character is found.
        Examples:<br>
        165.90 is converted into 165 (. is invalid character)<br>
        1 129 is converted into 1 (space is invalid character)<br>
        123abc is converted into 123 (a is invalid character)<br>
          String with preceeding invalid characters or empty string cause an error.
        Preceeding spaces are ignored.
        @param[in] str the string to convert.
        @param[out] result the result of the conversion.
        @return STATUS_OK is successfully converted otherwise an appropriate errorcode.
    */
    static int ToS64 (const string& str, S64& result);

    /**
        converts string to double
        @return STATUS_OK or ERR_NOT_FOUND (if conversion failed)
    */
    static int      ToDouble (const string& str, double& result);

    /**
        converts string which is a space separated list of numbers to double vector
        Example: "12.8 15e45  27 302.33 50002.0"
        @return number of recognized doubles
    */
    static int      ToDoubleVec (const string& str, DVector& result, bool clear_result=true);
    
    /**
        converts string which is a space separated list of numbers to integerlist
        Example: "12 15  27 302 50002"
        @return number of recognized integers
    */
    static int      ToIList (const string& str, IList& result);

    /**
        converts string which is a list of numbers separated by given char to integerlist
        Example: "12, 15,  27, 302, 50002" or "192.162.87.32" or "15,78.2;99"
        @param str source string
        @param seps accepted separator chars in string, like ",; " or "."
        @return number of recognized integers
    */
    static int      ToIList (const string& str, const char *seps, IList& result);

    /**
        returns a hash value for string str
        @return hash value
    */
    static int      Hash (const string& str);

    /**
        @return true if c is in [0-9]
    */
    static bool     IsDigit (char c);

    /**
        @return true if c is in [a-z,A-Z]
    */
    static bool     IsLetter (char c);

    /**
        converts argument to uppercase string
        @return uppercased parameter s
    */
    static string   ToUpper(const string& s);

    /**
        converts argument to lowercase string
        @return lowercased parameter s
    */
    static string   ToLower(const string& s);

    /**
        converts a byte to a string in binary representation
        MSBit left
    */
    static string BinToString (U8 n);

    /**
        converts an integer to a string in binary representation
        MSBit left
    */
    static string BinToString (int n);


    /**
        Checks if basename already ends with suffix and if not appends the 
        suffix
        @return string, which ends with suffix
    */
    static string AppendSuffix(const string& basename, const string& suffix);

    /**
      Tests if the string starts with given prefix
      @return true if yes, false otherwise
      */
    static bool StartsWith(const string& str, const string& prefix);

    static const char* WHITE_SPACES_ALL;
    static const char* WHITE_SPACES_DELIMITERS;

};







#endif
