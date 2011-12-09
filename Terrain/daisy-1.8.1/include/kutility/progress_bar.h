//////////////////////////////////////////////////////////////////////////
// Software License Agreement (BSD License)                             //
//                                                                      //
// Copyright (c) 2009                                                   //
// Engin Tola                                                           //
// web   : http://cvlab.epfl.ch/~tola                                   //
// email : engin.tola@epfl.ch                                           //
//                                                                      //
// All rights reserved.                                                 //
//                                                                      //
// Redistribution and use in source and binary forms, with or without   //
// modification, are permitted provided that the following conditions   //
// are met:                                                             //
//                                                                      //
//  * Redistributions of source code must retain the above copyright    //
//    notice, this list of conditions and the following disclaimer.     //
//  * Redistributions in binary form must reproduce the above           //
//    copyright notice, this list of conditions and the following       //
//    disclaimer in the documentation and/or other materials provided   //
//    with the distribution.                                            //
//  * Neither the name of the EPFL nor the names of its                 //
//    contributors may be used to endorse or promote products derived   //
//    from this software without specific prior written permission.     //
//                                                                      //
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS  //
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT    //
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS    //
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE       //
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  //
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, //
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;     //
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER     //
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT   //
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN    //
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE      //
// POSSIBILITY OF SUCH DAMAGE.                                          //
//                                                                      //
// See licence.txt file for more details                                //
//////////////////////////////////////////////////////////////////////////

#ifndef KUTILITY_PROGRESS_BAR_H
#define KUTILITY_PROGRESS_BAR_H

#include <string>
#include <ctime>

class progress_bar
{
public:
   explicit inline progress_bar(int start, int end, int divisions);

   void reset();

   void reset(int start, int end, int divisions);

   std::ostream& operator>>(std::ostream& os) const;

   const progress_bar& operator()(int current);

   void set_text(const std::string& text);

   void set_end_text( const std::string& text);

   void set_format(const std::string& formatString);

private:
   int m_start;
   int m_current;
   int m_end;
   int m_divisions;
   mutable int m_progress;
   time_t m_starting_time;

   std::string m_message;
   std::string m_end_message;
   std::string m_done;
   std::string m_processing;
   std::string m_notDone;
   std::string m_limit;
};


inline progress_bar::progress_bar(int start, int end, int divisions)
   : m_start(start),
     m_current(start),
     m_end(end),
     m_divisions(divisions),
     m_progress(0),
     m_message("Progress: "),
     m_end_message(" "),
     m_done("-"),
     m_processing(">"),
     m_notDone(" "),
     m_limit("|")
{
   time(&m_starting_time);
}

inline std::ostream& operator<<(std::ostream& os, const progress_bar& pb)
{
   return pb >> os;
}


#endif
