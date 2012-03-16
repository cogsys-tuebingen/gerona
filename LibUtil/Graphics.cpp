/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   3/15/2012
   @file   Graphics.cpp

*/ 

#include "Graphics.h"


// This is a subfunction of HSLtoRGB
void HSLtoRGB_Subfunction(unsigned& c, const double& temp1, const double& temp2, const double& temp3)
{
  if((temp3 * 6) < 1)
    c = (unsigned)((temp2 + (temp1 - temp2)*6*temp3)*100);
  else
  if((temp3 * 2) < 1)
    c = (unsigned)(temp1*100);
  else
  if((temp3 * 3) < 2)
    c = (unsigned)((temp2 + (temp1 - temp2)*(.66666 - temp3)*6)*100);
  else
    c = (unsigned)(temp2*100);
  return;
}

// This function extracts the hue, saturation, and luminance from "color"
// and places these values in h, s, and l respectively.
void Graphics::rgbToHsl( unsigned r ,  unsigned g,   unsigned b ,unsigned& h, unsigned& s, unsigned& l)
{

  double r_percent = ((double)r)/255;
  double g_percent = ((double)g)/255;
  double b_percent = ((double)b)/255;

  double max_color = 0;
  if((r_percent >= g_percent) && (r_percent >= b_percent))
    max_color = r_percent;
  if((g_percent >= r_percent) && (g_percent >= b_percent))
    max_color = g_percent;
  if((b_percent >= r_percent) && (b_percent >= g_percent))
    max_color = b_percent;

  double min_color = 0;
  if((r_percent <= g_percent) && (r_percent <= b_percent))
    min_color = r_percent;
  if((g_percent <= r_percent) && (g_percent <= b_percent))
    min_color = g_percent;
  if((b_percent <= r_percent) && (b_percent <= g_percent))
    min_color = b_percent;

  double L = 0;
  double S = 0;
  double H = 0;

  L = (max_color + min_color)/2;

  if(max_color == min_color)
  {
    S = 0;
    H = 0;
  }
  else
  {
    if(L < .50)
    {
      S = (max_color - min_color)/(max_color + min_color);
    }
    else
    {
      S = (max_color - min_color)/(2 - max_color - min_color);
    }
    if(max_color == r_percent)
    {
      H = (g_percent - b_percent)/(max_color - min_color);
    }
    if(max_color == g_percent)
    {
      H = 2 + (b_percent - r_percent)/(max_color - min_color);
    }
    if(max_color == b_percent)
    {
      H = 4 + (r_percent - g_percent)/(max_color - min_color);
    }
  }
  s = (unsigned)(S*100);
  l = (unsigned)(L*100);
  H = H*60;
  if(H < 0)
    H += 360;
  h = (unsigned)H;
}

// This function converts the "color" object to the equivalent RGB values of
// the hue, saturation, and luminance passed as h, s, and l respectively
void Graphics::hslToRgb(const unsigned& h, const unsigned& s, const unsigned& l,
              unsigned& r ,  unsigned& g,   unsigned& b )
{
  r = 0;
  g = 0;
  b = 0;

  double L = ((double)l)/100;
  double S = ((double)s)/100;
  double H = ((double)h)/360;

  if(s == 0)
  {
    r = l;
    g = l;
    b = l;
  }
  else
  {
    double temp1 = 0;
    if(L < .50)
    {
      temp1 = L*(1 + S);
    }
    else
    {
      temp1 = L + S - (L*S);
    }

    double temp2 = 2*L - temp1;

    double temp3 = 0;
    for(int i = 0 ; i < 3 ; i++)
    {
      switch(i)
      {
      case 0: // red
        {
          temp3 = H + .33333;
          if(temp3 > 1)
            temp3 -= 1;
          HSLtoRGB_Subfunction(r,temp1,temp2,temp3);
          break;
        }
      case 1: // green
        {
          temp3 = H;
          HSLtoRGB_Subfunction(g,temp1,temp2,temp3);
          break;
        }
      case 2: // blue
        {
          temp3 = H - .33333;
          if(temp3 < 0)
            temp3 += 1;
          HSLtoRGB_Subfunction(b,temp1,temp2,temp3);
          break;
        }
      default:
        {

        }
      }
    }
  }
  r = (unsigned)((((double)r)/100)*255);
  g = (unsigned)((((double)g)/100)*255);
  b = (unsigned)((((double)b)/100)*255);

}
