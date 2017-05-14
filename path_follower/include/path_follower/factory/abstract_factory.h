#ifndef ABSTRACT_FACTORY_H
#define ABSTRACT_FACTORY_H

#include <string>

/**
 * @brief The AbstractFactory class is the base class for all factories.
 */
class AbstractFactory
{
public:
    virtual ~AbstractFactory();

protected:
    /**
     * @brief toLower Convenience function to transform a string to lower case.
     * @param s String in any case
     * @return s transformed to lower case
     */
    static std::string toLower(const std::string& s);
};

#endif // ABSTRACT_FACTORY_H
