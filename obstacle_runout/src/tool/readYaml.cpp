#include "read_map/readYaml.h"

YAML * ReadYaml::yaml_allocate(){
    YAML *newYAML;
    newYAML = (YAML*)malloc(sizeof(YAML));
    if(newYAML == NULL){
        printf("yaml_allocate: allocate failed!\n");
        return(NULL);
    }

    newYAML->pixel = 0.0;
    newYAML->init_x = newYAML->init_y = newYAML->init_z =0.0;
    return newYAML;
}

/*
    YAML * yaml_open(string filename_tmp)
    读取yaml文件
    获得每一个像素大小pix
    获取采图时机器人在相应PGM图中的起始点坐标（init_x,init_y）（二维）
*/
YAML * ReadYaml::yaml_open(string filename_tmp){
    FILE *yaml;
    YAML *newYAML;
    char line[512];
    char filename_char[128];
    double pix,x,y,z;
    strcpy(filename_char,filename_tmp.c_str());
    char *filename = filename_char;
    int flag = 0;

    newYAML = yaml_allocate();
    if((yaml = fopen(filename,"r")) == NULL){
        printf("yaml_open: open yaml file %s failed!\n",filename);
        return NULL;
    }
    while(fgets(line,511,yaml) != NULL){
        if(sscanf(line,"resolution: %lf",&pix) != 0){
            newYAML->pixel = pix;
            flag++;
            printf("%lf\n",pix);
            continue;
        }
        if(sscanf(line,"origin: [%lf, %lf, %lf]",&x,&y,&z) != 0){
            newYAML->init_x = x;
            newYAML->init_y = y;
            newYAML->init_z = z;
            flag++;
            continue;
        }
        if(flag >= 2) break;
    }

    printf("readYAML: %f %f %f %f\n",newYAML->pixel,newYAML->init_x,newYAML->init_y,newYAML->init_z);
    fclose(yaml);
    return newYAML;

}