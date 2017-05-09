#ifndef ABSTRACT_FACTORY_H
#define ABSTRACT_FACTORY_H

#include <string>

class AbstractFactory
{
public:
    virtual ~AbstractFactory();

protected:
    static std::string toLower(const std::string& s);
};

#endif // ABSTRACT_FACTORY_H
