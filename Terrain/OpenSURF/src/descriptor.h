/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2010 Yasir Niaz Khan

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H

#include <string>

class Descriptor
{
  public:
    Descriptor();
    virtual bool CalculateDescriptor(int binSizeX, int binSizeY, char *folder, float param=0)=0;
    virtual bool SaveToFile();
    virtual bool LoadFromFile();
    virtual bool PrepareData(char *extension, bool equal=true);
    
  private:
    std::string m_name;
    
};

#endif // DESCRIPTOR_H
