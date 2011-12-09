//
// $Id: command_line.hh,v 1.2 2004-05-24 10:03:01 cholm Exp $ 
//  
//  optionmm::command_line
//  Copyright (C) 2002 Christian Holm Christensen <cholm@nbi.dk> 
//
//  This library is free software; you can redistribute it and/or 
//  modify it under the terms of the GNU Lesser General Public License 
//  as published by the Free Software Foundation; either version 2.1 
//  of the License, or (at your option) any later version. 
//
//  This library is distributed in the hope that it will be useful, 
//  but WITHOUT ANY WARRANTY; without even the implied warranty of 
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
//  Lesser General Public License for more details. 
// 
//  You should have received a copy of the GNU Lesser General Public 
//  License along with this library; if not, write to the Free 
//  Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 
//  02111-1307 USA 
//
#ifndef OPTIONMM_command_line
#define OPTIONMM_command_line
#ifndef OPTIONMM_optionmm
#include <optionmm/option.hh>
#endif
#ifndef __VECTOR__
#include <vector>
#endif
#ifndef __STRING__
#include <string>
#endif
#ifndef __IOSTREAM__
#include <iostream>
#endif

/** @file   command_line.hh
    @author Christian Holm
    @date   Sat Dec 28 19:02:59 2002
    @brief  Command line parser */

namespace optionmm
{
  /** @defgroup CommandLine Command line classes 
      These classes are imported from my project 
      <a href="http://cern.ch/cholm/misc/#optionmm">Option--</a> as a
      convinience for the test programs, and user applications. 
   */
  /** @class default_error_handler command_line.hh <optionmm/command_line.hh>
      @brief The default error handler for the command line. 
      @ingroup CommandLine 

      Client programs can define similar structures or classes, and
      pass them as a template parameter to basic_command_line.
  */
  struct default_error_handler 
  {
    /** Handle an unknown short option.
	@param c The short option
	@param progname name of the program
	@return true if basic_command_line<>::process should fail,
	false if it should go on. */
    bool on_unknown(char c, const std::string& progname) 
    {
      std::cerr << "Option `-" << c << "' unknown, try `" 
		<< progname << " --help'" << std::endl;
      return true;
    }
    /** Handle an unknown long option.
	@param str The long option
	@param progname name of the program
	@return true if basic_command_line<>::process should fail,
	false if it should go on. */
    bool on_unknown(const std::string& str, const std::string& progname)
    {
      std::cerr << "Option `" << str << "' unknown, try `" 
		<< progname << " --help'" << std::endl;
      return true;
    }
    /** Handle a bad handling of an argument to an option.  Short
	version. 
	@param o The option
	@param is_short Whether the short form was given or not.
	@param progname The program invocation name.
	@return true if basic_command_line<>::process should fail,
	false if it should go on. */
    template <typename Option>
    bool on_missing_argument(Option& o, 
			     bool is_short, 
			     const std::string& progname) 
    {
      std::cerr << "Option ";
      if (is_short) std::cerr << "-" << o.short_name();
      else          std::cerr << "--" << o.long_name();
      std::cerr << " need an argument, try " << progname << "  --help" 
		 << std::endl; 
      return true;
    }
    /** Handle a bad handling of an argument to an option.  Short
	version. 
	@param o The option
	@param is_short Whether the short form was given or not.
	@param progname The program invocation name.
	@return true if basic_command_line<>::process should fail,
	false if it should go on. */
    template <typename Option>
    bool on_bad_argument(Option& o, 
			 bool is_short, 
			 const std::string& progname) 
    {
      std::cerr << "Bad argument to option ";
      if (is_short) std::cerr << "-" << o.short_name();
      else          std::cerr << "--" << o.long_name();
      std::cerr << ", try " << progname << "  --help" << std::endl; 
      return true;
    }
  };

  //==================================================================
  /** @class basic_command_line command_line.hh <optionmm/command_line.hh>
      @brief Command line parser.
      @ingroup CommandLine 
      @param ErrorHandler A policy class to handle bad command line
      options and arguments.  See also default_error_handler.

   */
  template <typename ErrorHandler=default_error_handler>
  class basic_command_line : public ErrorHandler
  {
  public:
    /// The container of options. 
    typedef std::vector<option_base*> option_list;
  private:
    /// List of options 
    option_list _options;
    /// The program name (sans directory part)
    std::string _program_name;
    /// The number of arguments cached from command line 
    int& _argc;
    /// The command line arguments. 
    char** _argv;
    /// The title of the application 
    std::string _title;
    /// The version number of the application
    std::string _version; 
    /// The copyright of the application 
    std::string _copyright;
    /// The copyright of the application 
    std::string _usage;
    /// The help option. 
    basic_option<bool,false,false> _help_option;
    /// The version information option. 
    basic_option<bool,false,false> _version_option; 
    /** Handle the i'th short options 
	@return true on success, false oterwise. */
    bool handle_short(int& i);
    /** Handle the i'th long option
	@return true on success, false oterwise. */
    bool handle_long(int& i);
    /// Clean up command line 
    void cleanup();
  public:
    /** Constructor. 
	@param argc reference to the number of command line options
	from @c main
	@param argv the command line options from main 
	@param title the title of the application.
	@param version the version of the application 
	@param copy The copyright of the application.
	@param usage What to print as syntax (may be empty) */
    basic_command_line(const std::string title, 
		       const std::string& version, 
		       const std::string& copy, 
		       const std::string& usage, 
		       int&               argc, 
		       char**             argv);
    /** Add an option to the command line 
	@param option The option object to add to the manager. */
    template <typename T, bool a, bool m>
    void add(basic_option<T,a,m>& option) { _options.push_back(&option); }
    /** Print a help message 
	If the manager saw the @c --help option, then print the
	help message to stream @a o, and return true. 
	One can use the return value to jump out of the main program
	if the @c --help option was given, like 
	@code 
	  int main(int argc, char** argv) { 
	    using namespace optionmm; 
	    option_manager om("foo", "1.0", "bar", "", argc, argv);
	    ...
	    if (!om.process()) return 1;
	    if (om.help()) return 0;
	    ...
          }
        @endcode 
	@return true if the help option was given. */
    bool help(std::ostream& o=std::cout);
    /** Print version information. 
	If the manager saw the @c --version option, then print the
	version information to stream @a o, and return true. 
	@param o The stream to write information to. 
	@return true if version option was given. */
    bool version(std::ostream& o=std::cout);
    /** Process the command line 
	@return true on success, false oterwise. */
    bool process();
    /** Get the application name */
    const std::string& program_name() const { return _program_name; }
    /** Get the application title */
    const std::string& title() const { return _title; }
    /** Get the application version */
    const std::string& version() const { return _title; }
    /** Get the application copyright */
    const std::string& copyright() const { return _copyright; }
  };

  //____________________________________________________________________
  template <typename ErrorHandler>
  inline
  basic_command_line<ErrorHandler>::
  basic_command_line(const std::string  title,
		     const std::string& version, 
		     const std::string& copy,
		     const std::string& usage,
		     int&               argc, 
		     char**             argv)
    : _program_name(argv[0]), 
      _argc(argc), 
      _argv(argv),
      _title(title), 
      _version(version), 
      _copyright(copy), 
      _usage(usage),
      _help_option('h',"help","Show this help",false), 
      _version_option('\0',"version", "Show version information",false)
  {
    std::string::size_type slash = _program_name.find_last_of('/');
    _program_name.erase(0, slash+1);
    add(_help_option);
    add(_version_option);
  }

  //____________________________________________________________________
  template <typename ErrorHandler>
  inline
  bool
  basic_command_line<ErrorHandler>::help(std::ostream& out) 
  {
    if (!_help_option.value()) return false;

    _version_option.push_arg(_help_option.position());
    version();
    out << std::endl << "Usage: " << _program_name;
    if (_usage.empty()) out << " [OPTIONS]";
    else                out << " " << _usage;
    out << std::endl << std::endl; 

    /// Figure out the longest long_name
    std::string::size_type ll = 0;
    for (option_list::iterator o = _options.begin(); o != _options.end(); ++o) 
      if ((*o)->long_name().length() >= ll) 
	ll = (*o)->long_name().length();
    for (option_list::iterator p = _options.begin(); 
	 p != _options.end(); ++p) {
      out << "    ";
      (*p)->print(ll, out);
      out << std::endl;
    }
    out << std::endl;
    return true;
  }

  //____________________________________________________________________
  template <typename ErrorHandler>
  inline
  bool
  basic_command_line<ErrorHandler>::version(std::ostream& o) 
  { 
    if (!_version_option.value()) return false;
    o << _title << " version " << _version << std::endl
      << _copyright << std::endl;
    return true;
  }

  //____________________________________________________________________
  template <typename ErrorHandler>
  inline
  bool
  basic_command_line<ErrorHandler>::process() 
  {
    for (int i = 1; i < _argc; i++) {
      if (_argv[i] && _argv[i][0] == '-') {
	// Got an option
	bool ret;
	if (_argv[i][1] != '-') ret = handle_short(i);
	else                    ret = handle_long(i);
	if (!ret) return false;
      }
    }
    cleanup();
    return true;
  } 

  //____________________________________________________________________
  template <typename ErrorHandler>
  inline
  void
  basic_command_line<ErrorHandler>::cleanup() 
  {
    int n = 1;
    for (int i = 1; i < _argc; i++) {
      int j = i;
      while (!_argv[j] && j < _argc-1) { j++; }
      if (i != j) {
	_argv[i] = _argv[j]; 
	_argv[j] = 0;
      }
      if (_argv[i]) n++;
    }
    _argc = n;
  }


  //____________________________________________________________________
  template <typename ErrorHandler>
  inline
  bool
  basic_command_line<ErrorHandler>::handle_short(int& i) 
  {
    int  j     = 1;
    int  ret   = 0;
    bool gotit = false;
    while (_argv[i] && _argv[i][j] && _argv[i][j] != '-') {
      option_list::iterator o;
      for (o = _options.begin(); o < _options.end(); o++) {
	char* arg = &(_argv[i][j]);
	if ((ret = (*o)->handle(arg, _argv[i+1], i)) 
	    == option_base::can_handle) {
	  int k = j;
	  // Eat away this argument. 
	  while (_argv[i][k] != '\0') { 
	    // I really wanted to have `_argv[i][++k]' on the right hand
	    // side of this assignment, but GCC 3.0 optimises that
	    // incrementation away when -O1 or higher is specified, so
	    // we pin it out a bit more. 
	    _argv[i][k] = _argv[i][k+1];
	    k++;
	  }
	  j--;
	  if (!_argv[i+1]) gotit = true;
	  break;
	}
	switch (ret) {
	case option_base::bad_argument: 
	  if (ErrorHandler::on_bad_argument(**o, true, _program_name))
	    return false;
	  break;
	case option_base::missing_argument: 
	  if (ErrorHandler::on_missing_argument(**o, true, _program_name))
	    return false;
	  break;
	}
      }
      if (gotit) break;
      if (o == _options.end())
	if (ErrorHandler::on_unknown(_argv[i][j], _program_name))
	  return false;
      j++;
    } 
    if (_argv[i][0] == '-' && _argv[i][1] == '\0') _argv[i] = 0;
    if (gotit) i++;
    return true;
  }

  //____________________________________________________________________
  template <typename ErrorHandler>
  inline
  bool
  basic_command_line<ErrorHandler>::handle_long(int& i) 
  {
    int ret = 0;
    std::string n(_argv[i]);
    option_list::iterator o;
    for (o = _options.begin(); o < _options.end(); o++) {
      if ((ret = (*o)->handle(n, i)) == option_base::can_handle) {
	_argv[i] = 0;
	break;
      }
      switch (ret) {
      case option_base::bad_argument: 
	if (ErrorHandler::on_bad_argument(**o, false, _program_name))
	  return false;
	break;
      case option_base::missing_argument: 
	if (ErrorHandler::on_missing_argument(**o, false, _program_name))
	  return false;
	break;
      }
    } 
    if (o == _options.end())
      if (ErrorHandler::on_unknown(n, _program_name))
	return false;
    return true;
  }

  /** @typedef typedef basic_option<int,true,true> int_option
      @brief Explicit specialisation of basic_option for int
      @ingroup Option
  */
  typedef basic_option<int,true,true> int_option;
  /** @typedef typedef basic_option<float,true,true> float_option
      @brief Explicit specialisation of basic_option for float
      @ingroup Option
  */
  typedef basic_option<float,true,true> float_option;
  /** @typedef typedef basic_option<bool,false,false> bool_option
      @brief Explicit specialisation of basic_option for bool
      @ingroup Option
  */
  typedef basic_option<bool,false,false> bool_option;
  /** @typedef typedef basic_option<std::string,true,true> string_option
      @brief Explicit specialisation of basic_option for std::string
      @ingroup Option
  */
  typedef basic_option<std::string,true,true> string_option;
  /** @typedef typedef basic_command_line<> command_line
      @brief Explicit specialisation of basic_command_line for
      default_error_handler 
      @ingroup CommandLine
  */
  typedef basic_command_line<> command_line;
}

#endif
//____________________________________________________________________
//
// EOF
//
