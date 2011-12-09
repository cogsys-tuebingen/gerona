#ifndef FIELD_H
#define FIELD_H

#include <vector>

template < class CellT >
class Field
{
public:
    Field();
    std::vector<CellT> cells;
private:

};

template< class CellT > Field<CellT>::Field()
{
    cells.resize(65536);
}

#endif // FIELD_H
