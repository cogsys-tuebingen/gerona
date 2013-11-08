/*
 * Stopwatch.h
 *
 *  Created on: Mar 18, 2010
 *      Author: Sebastian Scherer <sebastian.scherer@uni-tuebingen.de>
 */

#ifndef STOPWATCH_H_
#define STOPWATCH_H_

#include <sys/time.h>
#include <ostream>

class Stopwatch {
public:
    Stopwatch();

    void resetStopped();
    void stop();
    void restart();
    void resume();
    void reset();

    int sElapsed() const;
    int msElapsed() const;
    int usElapsed() const;
    double sElapsedDouble() const;
    double elapsed() const;
    int sElapsedStatic();

    friend std::ostream& operator<<(std::ostream& stream, const Stopwatch&);
    
private:
    timeval mStart;
    timeval mStop;
    bool stopped;
};

#endif /* STOPWATCH_H_ */
