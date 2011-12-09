////////////////////////////////////////////////////////////////////////////////
///
///  \brief A C++ interface to gnuplot.
///
///
///  The interface uses pipes and so won't run on a system that doesn't have
///  POSIX pipe support Tested on Windows (MinGW and Visual C++) and Linux (GCC)
///
/// Version history:
/// 0. C interface
///    by N. Devillard (27/01/03)
/// 1. C++ interface: direct translation from the C interface
///    by Rajarshi Guha (07/03/03)
/// 2. corrections for Win32 compatibility
///    by V. Chyzhdzenka (20/05/03)
/// 3. some member functions added, corrections for Win32 and Linux
///    compatibility
///    by M. Burgis (10/03/08)
/// 4. moved non-template, non-inline function definitions into seperate file,
///    added plot_arrows method
///    by S. Scherer (10/05/10)
///
/// Requirements:
/// * gnuplot has to be installed (http://www.gnuplot.info/download.html)
/// * for Windows: set Path-Variable for Gnuplot path
///         (e.g. C:/program files/gnuplot/bin)
///         or set Gnuplot path with:
///         Gnuplot::set_GNUPlotPath(const std::string &path);
///
////////////////////////////////////////////////////////////////////////////////


#ifndef _GNUPLOT_PIPES_H_
#define _GNUPLOT_PIPES_H_


#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>              // for std::ostringstream
#include <stdexcept>
#include <cstdio>
#include <cstdlib>              // for getenv()
#include <list>                 // for std::list


#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
//defined for 32 and 64-bit environments
#include <io.h>                // for _access(), _mktemp()
#define GP_MAX_TMP_FILES  27   // 27 temporary files it's Microsoft restriction
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
//all UNIX-like OSs (Linux, *BSD, MacOSX, Solaris, ...)
#include <unistd.h>            // for access(), mkstemp()
#define GP_MAX_TMP_FILES  64
#else
#error unsupported or unknown operating system
#endif

//declare classes in global namespace


class GnuplotException : public std::runtime_error
{
public:
	GnuplotException(const std::string &msg) : std::runtime_error(msg){}
};



class Gnuplot
{
private:

	//----------------------------------------------------------------------------------
	// member data
	///\brief pointer to the stream that can be used to write to the pipe
	FILE                    *gnucmd;
	///\brief validation of gnuplot session
	bool                     valid;
	///\brief true = 2d, false = 3d
	bool                     two_dim;
	///\brief number of plots in session
	int                      nplots;
	///\brief functions and data are displayed in a defined styles
	std::string              pstyle;
	///\brief interpolate and approximate data in defined styles (e.g. spline)
	std::string              smooth;
	///\brief list of created tmpfiles
	std::vector<std::string> tmpfile_list;

	//----------------------------------------------------------------------------------
	// static data
	///\brief number of all tmpfiles (number of tmpfiles restricted)
	static int               tmpfile_num;
	///\brief name of executed GNUPlot file
	static std::string       m_sGNUPlotFileName;
	///\brief gnuplot path
	static std::string       m_sGNUPlotPath;
	///\brief standart terminal, used by showonscreen
	static std::string       terminal_std;

	//----------------------------------------------------------------------------------
	// member functions (auxiliary functions)
	// ---------------------------------------------------
	///\brief get_program_path(); and popen();
	///
	/// \param --> void
	///
	/// \return <-- void
	// ---------------------------------------------------
	void           init();
	// ---------------------------------------------------
	///\brief creates tmpfile and returns its name
	///
	/// \param tmp --> points to the tempfile
	///
	/// \return <-- the name of the tempfile
	// ---------------------------------------------------
	std::string    create_tmpfile(std::ofstream &tmp);

	//----------------------------------------------------------------------------------
	///\brief gnuplot path found?
	///
	/// \param ---
	///
	/// \return <-- found the gnuplot path (yes == true, no == false)
	// ---------------------------------------------------------------------------------
	static bool    get_program_path();

	// ---------------------------------------------------------------------------------
	///\brief checks if file is available
	///
	/// \param filename --> the filename
	/// \param mode 	--> the mode [optional,default value = 0]
	///
	/// \return file exists (yes == true, no == false)
	// ---------------------------------------------------------------------------------
	bool file_available(const std::string &filename);

	// ---------------------------------------------------------------------------------
	///\brief checks if file exists
	///
	/// \param filename --> the filename
	/// \param mode 	--> the mode [optional,default value = 0]
	///
	/// \return file exists (yes == true, no == false)
	// ---------------------------------------------------------------------------------
	static bool    file_exists(const std::string &filename, int mode=0);

public:

	// ----------------------------------------------------------------------------
	/// \brief optional function: set Gnuplot path manual
	/// attention:  for windows: path with slash '/' not backslash '\'
	///
	/// \param path --> the gnuplot path
	///
	/// \return true on success, false otherwise
	// ----------------------------------------------------------------------------
	static bool set_GNUPlotPath(const std::string &path);


	// ----------------------------------------------------------------------------
	/// optional: set standart terminal, used by showonscreen
	///   defaults: Windows - win, Linux - x11, Mac - aqua
	///
	/// \param type --> the terminal type
	///
	/// \return ---
	// ----------------------------------------------------------------------------
	static void set_terminal_std(const std::string &type);

	//-----------------------------------------------------------------------------
	// constructors
	// ----------------------------------------------------------------------------


	///\brief set a style during construction
	Gnuplot(const std::string &style = "points");

	/// plot a single std::vector at one go
	Gnuplot(const std::vector<double> &x,
			const std::string &title = "",
			const std::string &style = "points",
			const std::string &labelx = "x",
			const std::string &labely = "y");

	/// plot pairs std::vector at one go
	Gnuplot(const std::vector<double> &x,
			const std::vector<double> &y,
			const std::string &title = "",
			const std::string &style = "points",
			const std::string &labelx = "x",
			const std::string &labely = "y");

	/// plot triples std::vector at one go
	Gnuplot(const std::vector<double> &x,
			const std::vector<double> &y,
			const std::vector<double> &z,
			const std::string &title = "",
			const std::string &style = "points",
			const std::string &labelx = "x",
			const std::string &labely = "y",
			const std::string &labelz = "z");

	/// destructor: needed to delete temporary files
	~Gnuplot();


	//----------------------------------------------------------------------------------

	/// send a command to gnuplot
	Gnuplot& cmd(const std::string &cmdstr);
	// -------------------------------------------------------------------------
	///\brief Sends a command to an active gnuplot session, identical to cmd()
	/// send a command to gnuplot using the <<  operator
	///
	/// \param cmdstr --> the command string
	///
	/// \return <-- a reference to the gnuplot object
	// -------------------------------------------------------------------------
	inline Gnuplot& operator<<(const std::string &cmdstr){
		cmd(cmdstr);
		return(*this);
	}



	//--------------------------------------------------------------------------
	// show on screen or write to file

	/// sets terminal type to terminal_std
	Gnuplot& showonscreen(); // window output is set by default (win/x11/aqua)

	/// Saves a gnuplot to a file named filename.  Defaults to saving pdf
	Gnuplot& savetofigure(const std::string filename,
			const std::string terminal="ps");

	//--------------------------------------------------------------------------
	// set and unset

	/// set line style (some of these styles require additional information):
	///  lines, points, linespoints, impulses, dots, steps, fsteps, histeps,
	///  boxes, histograms, filledcurves
	Gnuplot& set_style(const std::string &stylestr = "points");

	/// interpolation and approximation of data, arguments:
	///  csplines, bezier, acsplines (for data values > 0), sbezier, unique, frequency
	/// (works only with plot_x, plot_xy, plotfile_x, plotfile_xy
	/// (if smooth is set, set_style has no effekt on data plotting)
	Gnuplot& set_smooth(const std::string &stylestr = "csplines");

	// ----------------------------------------------------------------------
	/// \brief unset smooth
	/// attention: smooth is not set by default
	///
	/// \param ---
	///
	/// \return <-- a reference to a gnuplot object
	// ----------------------------------------------------------------------
	inline Gnuplot& unset_smooth(){ smooth = ""; return *this;};


	/// scales the size of the points used in plots
	Gnuplot& set_pointsize(const double pointsize = 1.0);

	/// turns grid on/off
	inline Gnuplot& set_grid()	{cmd("set grid");return *this;};
	/// grid is not set by default
	inline Gnuplot& unset_grid(){cmd("unset grid");return *this;};

	// -----------------------------------------------
	/// set the mulitplot mode
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// -----------------------------------------------
	inline Gnuplot& set_multiplot(){cmd("set multiplot") ;return *this;};

	// -----------------------------------------------
	/// unsets the mulitplot mode
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// -----------------------------------------------
	inline Gnuplot& unset_multiplot(){cmd("unset multiplot");return *this;};



	/// set sampling rate of functions, or for interpolating data
	Gnuplot& set_samples(const int samples = 100);
	/// set isoline density (grid) for plotting functions as surfaces (for 3d plots)
	Gnuplot& set_isosamples(const int isolines = 10);

	// --------------------------------------------------------------------------
	/// enables/disables hidden line removal for surface plotting (for 3d plot)
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// --------------------------------------------------------------------------
	Gnuplot& set_hidden3d(){cmd("set hidden3d");return *this;};

	// ---------------------------------------------------------------------------
	/// hidden3d is not set by default
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// ---------------------------------------------------------------------------
	inline Gnuplot& unset_hidden3d(){cmd("unset hidden3d"); return *this;};

	/// enables/disables contour drawing for surfaces (for 3d plot)
	///  base, surface, both
	Gnuplot& set_contour(const std::string &position = "base");
	// --------------------------------------------------------------------------
	/// contour is not set by default, it disables contour drawing for surfaces
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// ------------------------------------------------------------------
	inline Gnuplot& unset_contour(){cmd("unset contour");return *this;};

	// ------------------------------------------------------------
	/// enables/disables the display of surfaces (for 3d plot)
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// ------------------------------------------------------------------
	inline Gnuplot& set_surface(){cmd("set surface");return *this;};

	// ----------------------------------------------------------
	/// surface is set by default,
	/// it disables the display of surfaces (for 3d plot)
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// ------------------------------------------------------------------
	inline Gnuplot& unset_surface(){cmd("unset surface"); return *this;}


	/// switches legend on/off
	/// position: inside/outside, left/center/right, top/center/bottom, nobox/box
	Gnuplot& set_legend(const std::string &position = "default");

	// ------------------------------------------------------------------
	/// \brief  Switches legend off
	/// attention:legend is set by default
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// ------------------------------------------------------------------
	inline Gnuplot& unset_legend(){cmd("unset key"); return *this;}

	// -----------------------------------------------------------------------
	/// \brief sets and clears the title of a gnuplot session
	///
	/// \param title --> the title of the plot [optional, default == ""]
	///
	/// \return <-- reference to the gnuplot object
	// -----------------------------------------------------------------------
	inline Gnuplot& set_title(const std::string &title = "")
	{
		std::string cmdstr;
		cmdstr = "set title \"";
		cmdstr+=title;
		cmdstr+="\"";
		*this<<cmdstr;
		return *this;
	}

	//----------------------------------------------------------------------------------
	///\brief Clears the title of a gnuplot session
	/// The title is not set by default.
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// ---------------------------------------------------------------------------------
	inline Gnuplot& unset_title(){this->set_title();return *this;}


	/// set x axis label
	Gnuplot& set_ylabel(const std::string &label = "x");
	/// set y axis label
	Gnuplot& set_xlabel(const std::string &label = "y");
	/// set z axis label
	Gnuplot& set_zlabel(const std::string &label = "z");

	/// set axis - ranges
	Gnuplot& set_xrange(const double iFrom,
			const double iTo);
	/// set y-axis - ranges
	Gnuplot& set_yrange(const double iFrom,
			const double iTo);
	/// set z-axis - ranges
	Gnuplot& set_zrange(const double iFrom,
			const double iTo);
	/// autoscale axis (set by default) of xaxis
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// -----------------------------------------------
	inline Gnuplot& set_xautoscale(){cmd("set xrange restore");cmd("set autoscale x");return *this;};

	// -----------------------------------------------
	/// autoscale axis (set by default) of yaxis
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// -----------------------------------------------
	inline Gnuplot& set_yautoscale(){cmd("set yrange restore");cmd("set autoscale y");return *this;};

	// -----------------------------------------------
	/// autoscale axis (set by default) of zaxis
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// -----------------------------------------------
	inline Gnuplot& set_zautoscale(){cmd("set zrange restore");cmd("set autoscale z");return *this;};


	/// turns on/off log scaling for the specified xaxis (logscale is not set by default)
	Gnuplot& set_xlogscale(const double base = 10);
	/// turns on/off log scaling for the specified yaxis (logscale is not set by default)
	Gnuplot& set_ylogscale(const double base = 10);
	/// turns on/off log scaling for the specified zaxis (logscale is not set by default)
	Gnuplot& set_zlogscale(const double base = 10);

	// -----------------------------------------------
	/// turns off log scaling for the x axis
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// -----------------------------------------------
	inline Gnuplot& unset_xlogscale(){cmd("unset logscale x"); return *this;};

	// -----------------------------------------------
	/// turns off log scaling for the y axis
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// -----------------------------------------------
	inline Gnuplot& unset_ylogscale(){cmd("unset logscale y"); return *this;};

	// -----------------------------------------------
	/// turns off log scaling for the z axis
	///
	/// \param ---
	///
	/// \return <-- reference to the gnuplot object
	// -----------------------------------------------
	inline Gnuplot& unset_zlogscale(){cmd("unset logscale z"); return *this;};


	/// set palette range (autoscale by default)
	Gnuplot& set_cbrange(const double iFrom, const double iTo);


	//--------------------------------------------------------------------------
	// plot

	/// plot a single std::vector: x
	///   from file
	Gnuplot& plotfile_x(const std::string &filename,
			const unsigned int column = 1,
			const std::string &title = "");
	///   from std::vector
	template<typename X>
	Gnuplot& plot_x(const X& x, const std::string &title = "");


	/// plot x,y pairs: x y
	///   from file
	Gnuplot& plotfile_xy(const std::string &filename,
			const unsigned int column_x = 1,
			const unsigned int column_y = 2,
			const std::string &title = "");
	///   from data
	template<typename X, typename Y>
	Gnuplot& plot_xy(const X& x, const Y& y, const std::string &title = "");


	/// plot x,y pairs with dy errorbars: x y dy
	///   from file
	Gnuplot& plotfile_xy_err(const std::string &filename,
			const unsigned int column_x  = 1,
			const unsigned int column_y  = 2,
			const unsigned int column_dy = 3,
			const std::string &title = "");
	///   from data
	template<typename X, typename Y, typename E>
	Gnuplot& plot_xy_err(const X &x, const Y &y, const E &dy,
			const std::string &title = "");


	/// plot x,y,z triples: x y z
	///   from file
	Gnuplot& plotfile_xyz(const std::string &filename,
			const unsigned int column_x = 1,
			const unsigned int column_y = 2,
			const unsigned int column_z = 3,
			const std::string &title = "");
	///   from std::vector
	template<typename X, typename Y, typename Z>
	Gnuplot& plot_xyz(const X &x,
			const Y &y,
			const Z &z,
			const std::string &title = "");

	/// plot arrows that point from (xf,yf) to (xto,yto)
	Gnuplot& plotfile_arrows(const std::string &filename,
			const unsigned int column_xf,
			const unsigned int column_yf,
			const unsigned int column_xto,
			const unsigned int column_yto,
			const std::string &title = "");

	/// plot arrows that point from (xf,yf) to (xto,yto)
	template<typename X, typename Y>
	Gnuplot& plot_arrows(const X& xf, const Y& yf,
			const X& xto, const Y& yto,
			const std::string &title = "");


	/// plot an equation of the form: y = ax + b, you supply a and b
	Gnuplot& plot_slope(const double a,
			const double b,
			const std::string &title = "");

	// plot an ellipse
	Gnuplot& plot_ellipse(const double centerX, const double centerY,
                          const double alpha,
	                      const double a, const double b,
	                      const int lineType = -1,
	                      const std::string &title = "");


	/// plot an equation supplied as a std::string y=f(x), write only the
	/// function f(x) not y the independent variable has to be x
	/// binary operators: ** exponentiation, * multiply, / divide, + add, -
	///     substract, % modulo
	/// unary operators: - minus, ! factorial
	/// elementary functions: rand(x), abs(x), sgn(x), ceil(x), floor(x),
	///   int(x), imag(x), real(x), arg(x), sqrt(x), exp(x), log(x), log10(x),
	///   sin(x), cos(x), tan(x), asin(x), acos(x), atan(x), atan2(y,x),
	///   sinh(x), cosh(x), tanh(x), asinh(x), acosh(x), atanh(x)
	/// special functions: erf(x), erfc(x), inverf(x), gamma(x), igamma(a,x),
	///   lgamma(x), ibeta(p,q,x), besj0(x), besj1(x), besy0(x), besy1(x),
	///   lambertw(x)
	/// statistical fuctions: norm(x), invnorm(x)
	Gnuplot& plot_equation(const std::string &equation,
			const std::string &title = "");

	/// plot an equation supplied as a std::string z=f(x,y), write only the
	/// function f(x,y) not z the independent variables have to be x and y
	Gnuplot& plot_equation3d(const std::string &equation,
			const std::string &title = "");


	/// plot image
	Gnuplot& plot_image(const unsigned char *ucPicBuf,
			const unsigned int iWidth,
			const unsigned int iHeight,
			const std::string &title = "");


	//--------------------------------------------------------------------------
	///\brief replot repeats the last plot or splot command.
	///  this can be useful for viewing a plot with different set options,
	///  or when generating the same plot for several devices (showonscreen,
	//   savetofigure)
	///
	/// \param ---
	///
	/// \return ---
	//--------------------------------------------------------------------------
	inline Gnuplot& replot(void){if (nplots > 0) cmd("replot");return *this;};

	/// resets a gnuplot session (next plot will erase previous ones)
	Gnuplot& reset_plot();

	/// resets a gnuplot session and sets all variables to default
	Gnuplot& reset_all();

	/// deletes temporary files
	void remove_tmpfiles();

	// -------------------------------------------------------------------
	/// \brief Is the gnuplot session valid ??
	///
	///
	/// \param ---
	///
	/// \return true if valid, false if not
	// -------------------------------------------------------------------
	inline bool is_valid(){return(valid);};

};


//------------------------------------------------------------------------------
//
/// Plots a 2d graph from a list of doubles: x
//
template<typename X>
Gnuplot& Gnuplot::plot_x(const X& x, const std::string &title)
{
	if (x.size() == 0)
	{
		throw GnuplotException("std::vector too small");
		return *this;
	}

	std::ofstream tmp;
	std::string name = create_tmpfile(tmp);
	if (name == "")
		return *this;

	//
	// write the data to file
	//
	for (unsigned int i = 0; i < x.size(); i++)
		tmp << x[i] << std::endl;

	tmp.flush();
	tmp.close();


	plotfile_x(name, 1, title);

	return *this;
}


//------------------------------------------------------------------------------
//
/// Plots a 2d graph from a list of doubles: x y
//
template<typename X, typename Y>
Gnuplot& Gnuplot::plot_xy(const X& x, const Y& y, const std::string &title)
{
	if (x.size() == 0 || y.size() == 0)
	{
		throw GnuplotException("std::vectors too small");
		return *this;
	}

	if (x.size() != y.size())
	{
		throw GnuplotException("Length of the std::vectors differs");
		return *this;
	}


	std::ofstream tmp;
	std::string name = create_tmpfile(tmp);
	if (name == "")
		return *this;

	//
	// write the data to file
	//
	for (unsigned int i = 0; i < x.size(); i++)
		tmp << x[i] << " " << y[i] << std::endl;

	tmp.flush();
	tmp.close();


	plotfile_xy(name, 1, 2, title);

	return *this;
}

//------------------------------------------------------------------------------
//
/// Plots a 2d graph from a list of doubles: x y
//
template<typename X, typename Y>
Gnuplot& Gnuplot::plot_arrows(const X& xf, const Y& yf,
		const X& xto, const Y& yto,
		const std::string &title)
{
	if (xf.size() == 0 || yf.size() == 0 || xto.size() == 0 || yto.size() == 0 )
	{
		throw GnuplotException("std::vectors too small");
		return *this;
	}

	if (xf.size() != yf.size() || xf.size() != xto.size()
			|| xf.size() != yto.size())
	{
		throw GnuplotException("Length of the std::vectors differs");
		return *this;
	}


	std::ofstream tmp;
	std::string name = create_tmpfile(tmp);
	if (name == "")
		return *this;

	//
	// write the data to file
	//
	for (unsigned int i = 0; i < xf.size(); i++)
		tmp << xf[i] << " " << yf[i] << " " << xto[i]-xf[i] << " " << yto[i]-yf[i] << std::endl;

	tmp.flush();
	tmp.close();


	plotfile_arrows(name, 1, 2, 3, 4, title);

	return *this;
}

///-----------------------------------------------------------------------------
///
/// plot x,y pairs with dy errorbars
///
template<typename X, typename Y, typename E>
Gnuplot& Gnuplot::plot_xy_err(const X &x,
		const Y &y,
		const E &dy,
		const std::string &title)
{
	if (x.size() == 0 || y.size() == 0 || dy.size() == 0)
	{
		throw GnuplotException("std::vectors too small");
		return *this;
	}

	if (x.size() != y.size() || y.size() != dy.size())
	{
		throw GnuplotException("Length of the std::vectors differs");
		return *this;
	}


	std::ofstream tmp;
	std::string name = create_tmpfile(tmp);
	if (name == "")
		return *this;

	//
	// write the data to file
	//
	for (unsigned int i = 0; i < x.size(); i++)
		tmp << x[i] << " " << y[i] << " " << dy[i] << std::endl;

	tmp.flush();
	tmp.close();


	// Do the actual plot
	plotfile_xy_err(name, 1, 2, 3, title);

	return *this;
}


//------------------------------------------------------------------------------
//
// Plots a 3d graph from a list of doubles: x y z
//
template<typename X, typename Y, typename Z>
Gnuplot& Gnuplot::plot_xyz(const X &x,
		const Y &y,
		const Z &z,
		const std::string &title)
{
	if (x.size() == 0 || y.size() == 0 || z.size() == 0)
	{
		throw GnuplotException("std::vectors too small");
		return *this;
	}

	if (x.size() != y.size() || x.size() != z.size())
	{
		throw GnuplotException("Length of the std::vectors differs");
		return *this;
	}


	std::ofstream tmp;
	std::string name = create_tmpfile(tmp);
	if (name == "")
		return *this;

	//
	// write the data to file
	//
	for (unsigned int i = 0; i < x.size(); i++)
		tmp << x[i] << " " << y[i] << " " << z[i] <<std::endl;

	tmp.flush();
	tmp.close();


	plotfile_xyz(name, 1, 2, 3, title);

	return *this;
}




//------------------------------------------------------------------------------
//
// A string tokenizer taken from http://www.sunsite.ualberta.ca/Documentation/
// /Gnu/libstdc++-2.90.8/html/21_strings/stringtok_std_h.txt
//
template <typename Container>
void stringtok (Container &container,
		std::string const &in,
		const char * const delimiters = " \t\n")
{
	const std::string::size_type len = in.length();
	std::string::size_type i = 0;

	while ( i < len )
	{
		// eat leading whitespace
		i = in.find_first_not_of (delimiters, i);

		if (i == std::string::npos)
			return;   // nothing left but white space

		// find the end of the token
		std::string::size_type j = in.find_first_of (delimiters, i);

		// push token
		if (j == std::string::npos)
		{
			container.push_back (in.substr(i));
			return;
		}
		else
			container.push_back (in.substr(i, j-i));

		// set up for next loop
		i = j + 1;
	}

	return;
}

#endif
