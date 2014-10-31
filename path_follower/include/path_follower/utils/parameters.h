#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <string>
#include <sstream>
#include <vector>
#include <ros/param.h>

/**
 * @brief Wrapper for ROS parameter access that encourages the user to add a description for each parameter and provides
 *        methods to export a list of all parameters for documentation.
 *
 * Usage:
 *
 * Inherit this class and add a public member of type 'P' for each of your parameters and add a constructor to define
 * them.
 *
 *     struct MyParameters : public Parameters
 *     {
 *         P<int> foo;
 *         P<float> bar;
 *
 *         MyParameters():
 *             foo(this, "~foo", 13, "Meaningless number"),
 *             bar(this, "~bar", 3.14, "Pi"),
 *         {}
 *     }
 *
 * To access the parameters, simply instantiate the class and access the parameters as follows:
 *     MyParameters opt;
 *     int foo = opt.foo();
 *
 * The Parameters class provides several methods to get a nice list of all parameters with default value and description:
 *  * print(): directly prints the list to the terminal (can for example be used when implementing a --help argument)
 *  * printToFile(filename): prints the list to the specified file (default: /tmp/parameters.md). The file can directly
 *          be used as a markdown table. Note: this method appends to existing files, without deleting existing content.
 * The class also keeps track of all living Parameters instances and thus provides the methods printAllInstances() and
 * printToFileAllInstances(), that work exactly like the methods described above, but combines the parameters of all
 * living instances. This is for example useful, if there are several Parameters instances spread over submodules of the
 * program.
 *
 * @author Felix Widmaier <felix.widmaier@web.de>
 * @version 1.2
 */
class Parameters
{
    friend class P;

public:
    template<typename T>
    class P
    {
    public:
        /**
         * @brief A Parameter
         * @param opt          Pointer to the parent Parameters class. Set this always to 'this'.
         * @param param_name   Name of this parameter (add prefix '~' for private parameters).
         * @param default_val  Default value.
         * @param desc         Description of the parameter.
         */
        P(Parameters *opt, const std::string& param_name, const T& default_val, const std::string& desc)
        {
            ros::param::param<T>(param_name, value_, default_val);
            opt->registerParam(param_name, default_val, desc);
        }

        //! Returns the parameters value.
        T operator() () const
        {
            return value_;
        }

        //! Change the value.
        void set(T v)
        {
            value_ = v;
        }

    private:
        T value_;
    };

    //! Print a simple parameter list with default values and descriptions to the terminal.
    void print();

    //! Print a list of all parameters to the specified file.
    void printToFile(const std::string &filename="/tmp/parameters.md");

    //! Like print() but combining the parameters of all living Parameters instances.
    static void printAllInstances();
    //! Like printToFile() but combining the parameters of all living Parameters instances.
    static void printToFileAllInstances(const std::string &filename="/tmp/parameters.md");

protected:
    Parameters(); // abstract class
    ~Parameters();

private:
    struct ParamInfo
    {
        std::string name;
        std::string description;
        std::string default_value;
    };

    std::vector<ParamInfo> params_;

    //! Register a parameter to the parameter list used by the print*-methods
    template<typename T>
    void registerParam(const std::string& param_name, const T& default_val, const std::string& desc = "")
    {
        std::stringstream ss_def_val;
        ss_def_val << default_val;

        ParamInfo pi;
        pi.name = param_name;
        pi.description = desc;
        pi.default_value = ss_def_val.str();

        params_.push_back(pi);
    }

    //! List, that keeps track if all living Parameters instances
    static std::vector<Parameters*> instances_;
    //! Register a new Parameters instance (called by constructor)
    static void registerInstance(Parameters* ptr);
    //! Unregisters a deleted Parameters instance (called by destructor)
    static void unregisterInstance(Parameters* ptr);
};

#endif // PARAMETERS_H
