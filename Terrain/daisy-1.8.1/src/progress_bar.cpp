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

#include "kutility/progress_bar.h"
#include <ostream>

void progress_bar::reset()
{
   m_current = m_start;
   m_progress = 0;
   time(&m_starting_time);
}

void progress_bar::reset(int start, int end, int divisions)
{
   m_start = start;
   m_current = start;
   m_end = end;
   m_divisions = divisions;
   m_progress = 0;
   time(&m_starting_time);
}

std::ostream& progress_bar::operator>>(std::ostream& os) const
{
   if(m_current > (m_progress * (m_end - m_start) / m_divisions) || m_current == m_end)
   {
      ++m_progress;
      os << m_message << m_limit;
      for(int c = 1; c <= m_divisions; ++c)
      {
         if(c < m_progress || m_current == m_end) {
            os << m_done;
         }
         else if(c > m_progress) {
            os << m_notDone;
         }
         else {
            os << m_processing;
         }
      }
      os << m_limit;

      time_t now; time(&now);
      double percent = double(m_current-m_start)/double(m_end-m_start);
      double elapsed = difftime( now, m_starting_time );
      double eta = elapsed / percent;

      os<<" ";
      os.width(5);
      os.fill(' ');
      os.precision(3);
      os.setf( std::ios_base::right );
      os<<eta - elapsed;

      os<<" / ";
      os.width(5);
      os.fill(' ');
      os.precision(3);
      os.setf( std::ios_base::left  );
      os<< eta<<"  ";

      os << m_end_message;

      if(m_current == m_end) {
         os << "\n" << std::flush;
      }
      else {
         os << "  \r" << std::flush;
      }
   }

   return os;
}

const progress_bar& progress_bar::operator()(int current)
{
   m_current = current;
   return *this;
}

void progress_bar::set_text(const std::string& text)
{
   m_message = text;
}
void progress_bar::set_end_text( const std::string& text)
{
   m_end_message = text;
}

void progress_bar::set_format(const std::string& formatString)
{
   if(formatString.length() >= 4)
   {
      m_limit = formatString[0];
      m_done = formatString[1];
      m_processing = formatString[2];
      m_notDone = formatString[3];
   }
}
