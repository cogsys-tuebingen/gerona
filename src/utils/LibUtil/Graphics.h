/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   3/15/2012
   @file   Graphics.h

*/ 

#ifndef GRAPHICS_H
#define GRAPHICS_H

class Graphics
{
public:
/**
  source:
http://www.codeguru.com/forum/showthread.php?t=421328
  */
static void rgbToHsl( unsigned r ,  unsigned g,   unsigned b ,unsigned& h, unsigned& s, unsigned& l);
static void hslToRgb(const unsigned& h, const unsigned& s, const unsigned& l,
              unsigned& r ,  unsigned& g,   unsigned& b );
};

#endif // GRAPHICS_H
