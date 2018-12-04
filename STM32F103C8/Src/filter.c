#include "filter.h"

float dFilter_1(float invar)
{
    float sumnum=0.0, sumden=0.0;  int i=0;
    static float states[3] = {0.0,0.0,0.0};
    static float znum[4] = {
        0.0,
        0.0,
        0.0,
        2.3294429e-04
    };
    static float zden[3] = {
        -.88191138,
        2.7565219,
        -2.8743776
    };
    sumnum = sumden = 0.0;
    for (i=0;i<3;i++){
        sumden += states[i]*zden[i];
        sumnum += states[i]*znum[i];
        if (i<2) states[i] = states[i+1];
    }
    states[2] = invar-sumden;
    sumnum += states[2]*znum[3];
    return sumnum;
	
		/* 3rd Order Low Pass Butterworth                                             */
		/* Matched Z Transformation                                                   */
		/* Sample Frequency = 10.00 KHz                                               */
		/* Standard Form                                                              */
		/* Arithmetic Precision = 8 Digits                                            */
		/* Pass Band Frequency = 100.0 Hz                                             */
}

float dFilter_2(float invar)
{
    float sumnum=0.0, sumden=0.0;  int i=0;
    static float states[3] = {0.0,0.0,0.0};
    static float znum[4] = {
        0.0,
        0.0,
        0.0,
        2.3294429e-04
    };
    static float zden[3] = {
        -.88191138,
        2.7565219,
        -2.8743776
    };
    sumnum = sumden = 0.0;
    for (i=0;i<3;i++){
        sumden += states[i]*zden[i];
        sumnum += states[i]*znum[i];
        if (i<2) states[i] = states[i+1];
    }
    states[2] = invar-sumden;
    sumnum += states[2]*znum[3];
    return sumnum;
	
		/* 3rd Order Low Pass Butterworth                                             */
		/* Matched Z Transformation                                                   */
		/* Sample Frequency = 10.00 KHz                                               */
		/* Standard Form                                                              */
		/* Arithmetic Precision = 8 Digits                                            */
		/* Pass Band Frequency = 100.0 Hz                                             */
}

float dFilter_3(float invar)
{
    float sumnum=0.0, sumden=0.0;  int i=0;
    static float states[3] = {0.0,0.0,0.0};
    static float znum[4] = {
        0.0,
        0.0,
        0.0,
        2.3294429e-04
    };
    static float zden[3] = {
        -.88191138,
        2.7565219,
        -2.8743776
    };
    sumnum = sumden = 0.0;
    for (i=0;i<3;i++){
        sumden += states[i]*zden[i];
        sumnum += states[i]*znum[i];
        if (i<2) states[i] = states[i+1];
    }
    states[2] = invar-sumden;
    sumnum += states[2]*znum[3];
    return sumnum;
	
		/* 3rd Order Low Pass Butterworth                                             */
		/* Matched Z Transformation                                                   */
		/* Sample Frequency = 10.00 KHz                                               */
		/* Standard Form                                                              */
		/* Arithmetic Precision = 8 Digits                                            */
		/* Pass Band Frequency = 100.0 Hz                                             */
}

float dFilter_4(float invar)
{
    float sumnum=0.0, sumden=0.0;  int i=0;
    static float states[3] = {0.0,0.0,0.0};
    static float znum[4] = {
        0.0,
        0.0,
        0.0,
        2.3294429e-04
    };
    static float zden[3] = {
        -.88191138,
        2.7565219,
        -2.8743776
    };
    sumnum = sumden = 0.0;
    for (i=0;i<3;i++){
        sumden += states[i]*zden[i];
        sumnum += states[i]*znum[i];
        if (i<2) states[i] = states[i+1];
    }
    states[2] = invar-sumden;
    sumnum += states[2]*znum[3];
    return sumnum;
	
		/* 3rd Order Low Pass Butterworth                                             */
		/* Matched Z Transformation                                                   */
		/* Sample Frequency = 10.00 KHz                                               */
		/* Standard Form                                                              */
		/* Arithmetic Precision = 8 Digits                                            */
		/* Pass Band Frequency = 100.0 Hz                                             */
}

