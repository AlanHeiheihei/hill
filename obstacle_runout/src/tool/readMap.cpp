#include "read_map/readMap.h"

IMG* ReadMap::img_allocate(){
    IMG *newMAP;
    newMAP = (IMG*)malloc(sizeof(IMG));

    if(newMAP == NULL){
        printf("img_allocate: allocate failed!\n");
        return(NULL);
    }

    newMAP->name = NULL;
    newMAP->rows = 0; newMAP->cols = 0;
    newMAP->data = NULL;
    return(newMAP);
}

/*
    IMG* img_open(string filename_tmp)
    读取PGM图
    用的是P5类型的PGM图
    获取每个像素点的灰度值，读取顺序为从左到右，从上到下，存入newMAP->data[]中
*/
IMG* ReadMap::img_open(string filename_tmp){//char *filename
    FILE *pgm;
    IMG *newMAP;
    char line[512];
    char filename_char[128];
    int type,row,col,maxnum;
    strcpy(filename_char,filename_tmp.c_str());
    char *filename = filename_char;

    newMAP = img_allocate(); 
    if((pgm = fopen(filename,"r")) == NULL){
        printf("img_open: open file %s failed!\n",filename);
        return(NULL);
    }
    newMAP->name = filename;

    for(int i=0;i<4;i++){
        fgets(line,511,pgm);
        if(i == 0){
            sscanf(line,"P%d",&type);                    //pgm图片类型，P5或P2
            printf("type: %d\n",type);
            if(type != 5 && type != 2){
                printf("img_open: file type is wrong!\n");
                fclose(pgm);
                return(NULL);
            }
        }
        if(i == 2){
            sscanf(line,"%d %d",&row,&col);              //图片的宽与高
            newMAP->rows = row;
            newMAP->cols = col;
            printf("row: %d,col: %d\n",row,col);
        }
        if(i == 3){
            sscanf(line,"%d",&maxnum);                    //像素最大数据值
            if(maxnum > 255){
                printf("img_open: max gray value is bigger than 255!\n");
                fclose(pgm);
                return(NULL);
            }
            printf("像素最大数据值: %d\n",maxnum);
        }
        
    }

    newMAP->data = (int *)malloc((unsigned)(row*col*sizeof(int)));
    if(newMAP->data == NULL){
        printf("img_open: allocate memory to data failed!\n");
        fclose(pgm);
        return(NULL);
    }
    
    if(type == 5){
        for(int i=0; i<col; i++){                        //从左到右，从上到下
            for(int j=0; j<row; j++){
                int val_tmp = fgetc(pgm);
                newMAP->data[i*row+j] = val_tmp;
            }
        }
    }
    if(type == 2){}

    fclose(pgm);
    return newMAP;    
}