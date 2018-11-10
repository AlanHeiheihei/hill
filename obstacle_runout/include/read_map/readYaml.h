#ifndef _READYAML_H_
#define _READYAML_H_

#include "read_map/read.h"

typedef struct{
    double pixel;
    double init_x,init_y,init_z;
}YAML;

class ReadYaml
{
    public:
        YAML *yaml_allocate();
        YAML *yaml_open(string filename_tmp);
};

#endif
