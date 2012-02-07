/*
 * RamaxxException.h
 *
 *  Created on: 13.09.2010
 *      Author: dube
 */

#include <string>
#include <exception>

#ifndef RAMAXXEXCEPTION_H_
#define RAMAXXEXCEPTION_H_

class RamaxxException : public std::exception{
public:
	RamaxxException(std::string message);
	virtual ~RamaxxException() throw();
	virtual const char* what() const throw();
private:
	std::string mMessage;
};

#endif /* RAMAXXEXCEPTION_H_ */
