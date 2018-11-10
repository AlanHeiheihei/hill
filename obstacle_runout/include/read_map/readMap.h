#ifndef _READMAP_H_
#define _READMAP_H_

#include "read_map/read.h"

typedef struct{
    char *name;
    int rows,cols;
    int *data;
}IMG;

class ReadMap
{
    public:
        IMG* img_open(string filename_tmp);
        IMG* img_allocate();
};

#endif