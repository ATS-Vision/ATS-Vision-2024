#include <iostream>

typedef enum SystemModel
{
    CSMODEL,
    IMMMODEL
} SystemModel;

typedef enum SpinningStatus
{
    STILL_SPINNING,
    MOVEMENT_SPINNING
} SpinningStatus;

enum PredictorState
{
    TRACKING,   //追踪
    PREDICTING, //预测
    LOSTING,    //丢失预测中
    LOST        //丢失
};