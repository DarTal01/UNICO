/*
 * Copyright 1994-2012 The MathWorks, Inc.
 *
 * File    : classic_main.c
 *
 * Abstract:
 *      A Generic "Real-Time (single tasking or pseudo-multitasking,
 *      statically allocated data)" main that runs under most
 *      operating systems.
 *
 *      This file may be a useful starting point when targeting a new
 *      processor or microcontroller.
 *
 *
 * Compiler specified defines:
 *	RT              - Required.
 *      MODEL=modelname - Required.
 *	NUMST=#         - Required. Number of sample times.
 *	NCSTATES=#      - Required. Number of continuous states.
 *      TID01EQ=1 or 0  - Optional. Only define to 1 if sample time task
 *                        id's 0 and 1 have equal rates.
 *      MULTITASKING    - Optional. (use MT for a synonym).
 *	SAVEFILE        - Optional (non-quoted) name of .mat file to create.
 *			  Default is <MODEL>.mat
 */


/* 

Modified from https://github.com/TUDelft-DataDrivenControl/DISCON_Simulink/tree/master/Simulink_64bit

Copyright(c) 2018 Delft Center for Systems& Control, Data - Driven Control(TU Delft)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this softwareand associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright noticeand this permission notice shall be included in all
copies or substantial portions of the Software.

*/
#define _CRT_SECURE_NO_WARNINGS
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
FILE *pFile; //Added by JW to read the init file

static float dKpBrLUT[5], dKiBrLUT[5], dKpArLUT[5], dKiArLUT[5];
static float dWindSpeed_KpBrLUT_BP[5], dWindSpeed_KiBrLUT_BP[5];
static float dWindSpeed_KpArLUT_BP[5], dWindSpeed_KiArLUT_BP[5];


#include "rtwtypes.h"
# include "rtmodel.h"
#include "rt_sim.h"
#include "rt_logging.h"
#ifdef UseMMIDataLogging
#include "rt_logging_mmi.h"
#endif

#include "ext_work.h"



/*=========*
 * Defines *
 *=========*/

#ifndef TRUE
#define FALSE (0)
#define TRUE  (1)
#endif

#ifndef EXIT_FAILURE
#define EXIT_FAILURE  1
#endif
#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS  0
#endif

#define QUOTE1(name) #name
#define QUOTE(name) QUOTE1(name)    /* need to expand name    */

#ifndef RT
# error "must define RT"
#endif

#ifndef MODEL
# error "must define MODEL"
#endif

#ifndef NUMST
# error "must define number of sample times, NUMST"
#endif

#ifndef NCSTATES
# error "must define NCSTATES"
#endif

#ifndef SAVEFILE
# define MATFILE2(file) #file ".mat"
# define MATFILE1(file) MATFILE2(file)
# define MATFILE MATFILE1(MODEL)
#else
# define MATFILE QUOTE(SAVEFILE)
#endif

#define RUN_FOREVER -1.0

#define EXPAND_CONCAT(name1,name2) name1 ## name2
#define CONCAT(name1,name2) EXPAND_CONCAT(name1,name2)
#define RT_MODEL            CONCAT(MODEL,_rtModel)

#define EXPAND_CONCAT3(name1,name2,name3) name1 ## _ ## name2 ## . ## name3
#define CONCAT3(name1,name2,name3) EXPAND_CONCAT3(name1,name2,name3)
#define SIG_MODEL(suffix,name)     CONCAT3(MODEL,suffix,name)

#define NINT(a) ((a) >= 0.0 ? (int)((a)+0.5) : (int)((a)-0.5))
#define MIN(a,b) ((a)>(b)?(b):(a))

/*====================*
 * External functions *
 *====================*/
#ifdef __cplusplus

extern "C" {

#endif

extern RT_MODEL *MODEL(void);

extern void MdlInitializeSizes(void);
extern void MdlInitializeSampleTimes(void);
extern void MdlStart(void);
extern void MdlOutputs(int_T tid);
extern void MdlUpdate(int_T tid);
extern void MdlTerminate(void);

extern void __declspec(dllexport) __cdecl DISCON(float *avrSwap, int *aviFail, char *accInfile, char *avcOutname, char *avcMsg); 

#ifdef __cplusplus

}
#endif

#if NCSTATES > 0
#ifdef __cplusplus

extern "C" {

#endif
  extern void rt_ODECreateIntegrationData(RTWSolverInfo *si);
  extern void rt_ODEUpdateContinuousStates(RTWSolverInfo *si);
#ifdef __cplusplus

}
#endif

# define rt_CreateIntegrationData(S) \
    rt_ODECreateIntegrationData(rtmGetRTWSolverInfo(S));
# define rt_UpdateContinuousStates(S) \
    rt_ODEUpdateContinuousStates(rtmGetRTWSolverInfo(S));
# else
# define rt_CreateIntegrationData(S)  \
      rtsiSetSolverName(rtmGetRTWSolverInfo(S),"FixedStepDiscrete");
# define rt_UpdateContinuousStates(S) /* Do Nothing */
#endif


/*==================================*
 * Global data local to this module *
 *==================================*/

static struct {
  int_T    stopExecutionFlag;
  int_T    isrOverrun;
  int_T    overrunFlags[NUMST];
  int_T    eventFlags[NUMST];
  const    char_T *errmsg;
} GBLbuf;


#ifdef EXT_MODE
#  define rtExtModeSingleTaskUpload(S)                          \
   {                                                            \
        int stIdx;                                              \
        rtExtModeUploadCheckTrigger(rtmGetNumSampleTimes(S));   \
        for (stIdx=0; stIdx<NUMST; stIdx++) {                   \
            if (rtmIsSampleHit(S, stIdx, 0 /*unused*/)) {       \
                rtExtModeUpload(stIdx,rtmGetTaskTime(S,stIdx)); \
            }                                                   \
        }                                                       \
   }
#else
#  define rtExtModeSingleTaskUpload(S) /* Do nothing */
#endif

/*=================*
 * Local functions *
 *=================*/

/* Function: initiateController ===========================================
 *
 * Abstract:
 *      Initialize the controller of the compiled Matlab Simulink block.
 */
static RT_MODEL *S;
int initiateController(char* errorMsg) {
    (void)errorMsg;
    const char* status;

    /****************************
     * Initialize global memory *
     ****************************/
    (void)memset(&GBLbuf, 0, sizeof(GBLbuf));

    /************************
     * Initialize the model *
     ************************/

    S = MODEL();
    if (rtmGetErrorStatus(S) != NULL) {
        (void)fprintf(stderr,"Error during model registration: %s\n",
                      rtmGetErrorStatus(S));
        exit(EXIT_FAILURE);
    }
    rtmSetTFinal(S,RUN_FOREVER);

    MdlInitializeSizes();
    MdlInitializeSampleTimes();
    
    status = rt_SimInitTimingEngine(rtmGetNumSampleTimes(S),
                                    rtmGetStepSize(S),
                                    rtmGetSampleTimePtr(S),
                                    rtmGetOffsetTimePtr(S),
                                    rtmGetSampleHitPtr(S),
                                    rtmGetSampleTimeTaskIDPtr(S),
                                    rtmGetTStart(S),
                                    &rtmGetSimTimeStep(S),
                                    &rtmGetTimingData(S));

    if (status != NULL) {
        (void)fprintf(stderr,
                "Failed to initialize sample time engine: %s\n", status);
        exit(EXIT_FAILURE);
    }
    rt_CreateIntegrationData(S);

#ifdef UseMMIDataLogging
    rt_FillStateSigInfoFromMMI(rtmGetRTWLogInfo(S),&rtmGetErrorStatus(S));
#endif
    GBLbuf.errmsg = rt_StartDataLogging(rtmGetRTWLogInfo(S),
                                        rtmGetTFinal(S),
                                        rtmGetStepSize(S),
                                        &rtmGetErrorStatus(S));
    if (GBLbuf.errmsg != NULL) {
        (void)fprintf(stderr,"Error starting data logging: %s\n",GBLbuf.errmsg);
        return(EXIT_FAILURE);
    }

    rtExtModeCheckInit(rtmGetNumSampleTimes(S));
    rtExtModeWaitForStartPkt(rtmGetRTWExtModeInfo(S),
                             rtmGetNumSampleTimes(S),
                             (boolean_T *)&rtmGetStopRequested(S));

    (void)printf("\n** Starting UNICO controller **\n");

    MdlStart();
    if (rtmGetErrorStatus(S) != NULL) {
      GBLbuf.stopExecutionFlag = 1;
    }
    
    return 0;
}  /* end initiateController */

#if !defined(MULTITASKING)  /* SINGLETASKING */
int calcOutputController(float rUserVar1, float rUserVar2, float rUserVar3, float rUserVar4, float rUserVar5, float rUserVar6, float rUserVar7, float rUserVar8, float rUserVar9, float rUserVar10,
    float rUserVar11, float rUserVar12, float rUserVar13, float rUserVar14, float rUserVar15, float rUserVar16, float rUserVar17, float rUserVar18, float rUserVar19, float rUserVar20, float rUserVar21,
    float rUserVar22, float rUserVar23, float rUserVar24, float rUserVar25, float rUserVar26, float rUserVar27, float rUserVar28, float rUserVar29, float rUserVar30, float rUserVar31, float rUserVar32, float rUserVar33, float rUserVar34, float rUserVar35, float rUserVar36, float rUserVar37, 
    float rUserVar38, float rUserVar39, float rUserVar40, float rUserVar41, float rUserVar42, float rUserVar43, float rUserVar44, float rUserVar45,
    float rTime, float rInit, float rSample, float rGeneratorSpeed, float rWindSpeed, float rRatedSpeed,
    float rBelowRatedPitch, float rForeAftTower, float rSideTower,
    float rRotorAzimuth, float rOP1RootMoment, float rOP2RootMoment,
    float rOP3RootMoment, float rIP1RootMoment, float rIP2RootMoment,
    float rIP3RootMoment, float rMeasuredPitch, float rMeasuredTorque, float rShaftTorque,
    float rModeGain, float rYawError, float rYawBearingRate, float rElectricalPower, float* rTorqueDemand, float* rBlade1Pitch,
    float* rBlade2Pitch, float* rBlade3Pitch, float* rPitchDemand, char* errorMsg, float* rYawRate,
    float* rLog1, float* rLog2, float* rLog3, float* rLog4, float* rLog5, float* rLog6, float* rLog7, float* rLog8, float* rLog9, float* rLog10,
    float* rLog11, float* rLog12, float* rLog13, float* rLog14, float* rLog15, float* rLog16, float* rLog17, float* rLog18, float* rLog19, float* rLog20,
    float* dKpBrLUT, float* dKiBrLUT, float* dKpArLUT, float* dKiArLUT,
    float* dWindSpeed_KpBrLUT_BP, float* dWindSpeed_KiBrLUT_BP,
    float* dWindSpeed_KpArLUT_BP, float* dWindSpeed_KiArLUT_BP,
    char* path2table) {
    real_T tnext;
    
    SIG_MODEL(U, SimTime) = rTime;
    SIG_MODEL(U, Generator_Speed) = rGeneratorSpeed;
    SIG_MODEL(U, Wind_Speed) = rWindSpeed;
    SIG_MODEL(U, Com_dt) = rSample;
    SIG_MODEL(U,Rated_Speed) = rRatedSpeed;
    SIG_MODEL(U,Below_Rated_Pitch_Angle) = rBelowRatedPitch;
    SIG_MODEL(U,Fore_Aft_Tower_Accel) = rForeAftTower;
    SIG_MODEL(U,Sidewards_Tower_Accel) = rSideTower;
    SIG_MODEL(U,Measured_Pitch) = rMeasuredPitch;
    SIG_MODEL(U,Measured_Torque) = rMeasuredTorque;
	SIG_MODEL(U,Shaft_Torque) = rShaftTorque;
    SIG_MODEL(U,Mode_Gain) = rModeGain;
    SIG_MODEL(U,Rotor_Azimuth_Angle) = rRotorAzimuth;
    SIG_MODEL(U,Blade1_OP_Root_Moment) = rOP1RootMoment;
    SIG_MODEL(U,Blade2_OP_Root_Moment) = rOP2RootMoment;
    SIG_MODEL(U,Blade3_OP_Root_Moment) = rOP3RootMoment;
    SIG_MODEL(U,Blade1_IP_Root_Moment) = rIP1RootMoment;
    SIG_MODEL(U,Blade2_IP_Root_Moment) = rIP2RootMoment;
    SIG_MODEL(U,Blade3_IP_Root_Moment) = rIP3RootMoment;
	
	SIG_MODEL(U, Init) = rInit;
	SIG_MODEL(U, dEcho) = rUserVar1;
	SIG_MODEL(U, dRotorRadius) = rUserVar2;
	SIG_MODEL(U, dRtWSpd) = rUserVar3;
	SIG_MODEL(U, dRtTq) = rUserVar4;
	SIG_MODEL(U, dRtPwr) = rUserVar5;
	SIG_MODEL(U, dRtGenSpd) = rUserVar6;
	SIG_MODEL(U, dGenEff) = rUserVar7;
	SIG_MODEL(U, dMinTq) = rUserVar8;
	SIG_MODEL(U, dMaxTq) = rUserVar9;
	SIG_MODEL(U, dMaxRat) = rUserVar10;
	SIG_MODEL(U, dMinOMSpd) = rUserVar11;
	SIG_MODEL(U, dRgn2Mode) = rUserVar12;
	SIG_MODEL(U, dKSwitch) = rUserVar13;
	SIG_MODEL(U, dRgn2K) = rUserVar14;
	SIG_MODEL(U, dOMSpdSwitch) = rUserVar15;
    SIG_MODEL(U, dMinOMSpdSwitch) = rUserVar16;
	SIG_MODEL(U, dTSRopt) = rUserVar17;
	SIG_MODEL(U, dCpMax) = rUserVar18;
	SIG_MODEL(U, dRgn3Mode) = rUserVar19;
	SIG_MODEL(U, dMRgn3Lin) = rUserVar20;
	SIG_MODEL(U, dVLin) = rUserVar21;
    SIG_MODEL(U, dKpAr) = rUserVar22;
    SIG_MODEL(U, dKiAr) = rUserVar23;
    SIG_MODEL(U, dKpBr) = rUserVar24;
    SIG_MODEL(U, dKiBr) = rUserVar25;
    SIG_MODEL(U, dWindSpeedFSw) = rUserVar26;
    SIG_MODEL(U, dWindSpeedFLPF) = rUserVar27;
    SIG_MODEL(U, dGenSpeedFSw) = rUserVar28;
    SIG_MODEL(U, dGenSpeedFLPF) = rUserVar29;
    SIG_MODEL(U, dRefSpeedFSw) = rUserVar30;
    SIG_MODEL(U, dRefSpeedFLPF) = rUserVar31;
    SIG_MODEL(U, dRhoAir) = rUserVar32;
    SIG_MODEL(U, dFPitch) = rUserVar33;
    SIG_MODEL(U, Architecture_Selector) = rUserVar34;
    SIG_MODEL(U, dZeros_Damping) = rUserVar35;
    SIG_MODEL(U, dPoles_Damping) = rUserVar36;
    SIG_MODEL(U, dBlades_Number) = rUserVar37;
    SIG_MODEL(U, dBlades_Height) = rUserVar38;
    SIG_MODEL(U, dIntTermSwitch) = rUserVar39;
    SIG_MODEL(U, dKpBrMode) = rUserVar40;
    SIG_MODEL(U, dKiBrMode) = rUserVar41;
    SIG_MODEL(U, dKpArMode) = rUserVar42;
    SIG_MODEL(U, dKiArMode) = rUserVar43;
    SIG_MODEL(U, dRotInertia) = rUserVar44;
    SIG_MODEL(U, dAWSwitch) = rUserVar45;
    
    {
        int j;
        for (j = 0; j < 5; j++) {
            SIG_MODEL(U, dKpBrLUT)[j] = dKpBrLUT[j];
            SIG_MODEL(U, dKiBrLUT)[j] = dKiBrLUT[j];
            SIG_MODEL(U, dKpArLUT)[j] = dKpArLUT[j];
            SIG_MODEL(U, dKiArLUT)[j] = dKiArLUT[j];
            SIG_MODEL(U, dWindSpeed_KpBrLUT_BP)[j] = dWindSpeed_KpBrLUT_BP[j];
            SIG_MODEL(U, dWindSpeed_KiBrLUT_BP)[j] = dWindSpeed_KiBrLUT_BP[j];
            SIG_MODEL(U, dWindSpeed_KpArLUT_BP)[j] = dWindSpeed_KpArLUT_BP[j];
            SIG_MODEL(U, dWindSpeed_KiArLUT_BP)[j] = dWindSpeed_KiArLUT_BP[j];
        }
    }

    strcpy(SIG_MODEL(U, dTablePath), path2table);
    // strcpy(SIG_MODEL(U, dTablePath), path2table);
	SIG_MODEL(U,YawError) = rYawError;
	SIG_MODEL(U,YawBearingRate) = rYawBearingRate;
	SIG_MODEL(U,ElectricalPower) = rElectricalPower;
	/***********************************************
     * Check and see if base step time is too fast *
     ***********************************************/
    if (GBLbuf.isrOverrun++) {
        GBLbuf.stopExecutionFlag = 1;
        return EXIT_FAILURE;
    }

    /***********************************************
     * Check and see if error status has been set  *
     ***********************************************/
    if (rtmGetErrorStatus(S) != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return EXIT_FAILURE;
    }
    
    /* enable interrupts here */
    
    
    tnext = rt_SimGetNextSampleHit();
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(S),tnext);

    MdlOutputs(0);

    rtExtModeSingleTaskUpload(S);

    GBLbuf.errmsg = rt_UpdateTXYLogVars(rtmGetRTWLogInfo(S),
                                        rtmGetTPtr(S));
    if (GBLbuf.errmsg != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return EXIT_FAILURE;
    }

    MdlUpdate(0);
    rt_SimUpdateDiscreteTaskSampleHits(rtmGetNumSampleTimes(S),
                                       rtmGetTimingData(S),
                                       rtmGetSampleHitPtr(S),
                                       rtmGetTPtr(S));

    if (rtmGetSampleTime(S,0) == CONTINUOUS_SAMPLE_TIME) {
        rt_UpdateContinuousStates(S);
    }

    GBLbuf.isrOverrun--;

    rtExtModeCheckEndTrigger();

    
    rTorqueDemand[0] = SIG_MODEL(Y,Generator_Torque);
    /*(void)printf("\n** %f **\n", rTorqueDemand[0]);*/

    rBlade1Pitch[0] = SIG_MODEL(Y,Blade1_Pitch_Angle);
    rBlade2Pitch[0] = SIG_MODEL(Y,Blade2_Pitch_Angle);
    rBlade3Pitch[0] = SIG_MODEL(Y,Blade3_Pitch_Angle);
	rPitchDemand[0] = SIG_MODEL(Y, Collective_Pitch_Angle);
	rYawRate[0] = SIG_MODEL(Y,Yaw_Rate);
    rLog1[0] = SIG_MODEL(Y,Log1);
    rLog2[0] = SIG_MODEL(Y,Log2);
    rLog3[0] = SIG_MODEL(Y,Log3);
    rLog4[0] = SIG_MODEL(Y,Log4);
    rLog5[0] = SIG_MODEL(Y,Log5);
    rLog6[0] = SIG_MODEL(Y,Log6);
    rLog7[0] = SIG_MODEL(Y,Log7);
    rLog8[0] = SIG_MODEL(Y,Log8);
    rLog9[0] = SIG_MODEL(Y,Log9);
    rLog10[0] = SIG_MODEL(Y,Log10);
	rLog11[0] = SIG_MODEL(Y,Log11);
	rLog12[0] = SIG_MODEL(Y,Log12);
	rLog13[0] = SIG_MODEL(Y,Log13);
	rLog14[0] = SIG_MODEL(Y,Log14);
	rLog15[0] = SIG_MODEL(Y,Log15);
	rLog16[0] = SIG_MODEL(Y,Log16);
	rLog17[0] = SIG_MODEL(Y,Log17);
	rLog18[0] = SIG_MODEL(Y,Log18);
	rLog19[0] = SIG_MODEL(Y,Log19);
	rLog20[0] = SIG_MODEL(Y,Log20);


    return 0;
}  /* end calcOutputController */

#else /* MULTITASKING */

# if TID01EQ == 1
#  define FIRST_TID 1
# else
#  define FIRST_TID 0
# endif

int calcOutputController(float rUserVar1, float rUserVar2, float rUserVar3, float rUserVar4, float rUserVar5,float rUserVar6, float rUserVar7, float rUserVar8, float rUserVar9, float rUserVar10,
						 float rUserVar11,float rUserVar12,float rUserVar13,float rUserVar14,float rUserVar15,float rUserVar16,float rUserVar17,float rUserVar18,float rUserVar19,float rUserVar20, 
        float rUserVar21, float rUserVar22, float rUserVar23, float rUserVar24, float rUserVar25, float rUserVar26, float rUserVar27, float rUserVar28, float rUserVar29, float rUserVar30, float rUserVar31,
        float rUserVar32, float rUserVar33, float rUserVar34, float rUserVar35, float rUserVar36, float rUserVar37, float rUserVar38, float rUserVar39, float rUserVar40, float rUserVar41, float rUserVar42, float rUserVar43, 
        float rUserVar44, float rUserVar45, float rTime,
	    float rInit, float rSample,float rGeneratorSpeed, float rWindSpeed, float rRatedSpeed,
        float rBelowRatedPitch, float rForeAftTower, float rSideTower,
        float rRotorAzimuth, float rOP1RootMoment, float rOP2RootMoment,
        float rOP3RootMoment,float rIP1RootMoment, float rIP2RootMoment,
        float rIP3RootMoment, float rMeasuredPitch, float rMeasuredTorque, float rShaftTorque,
        float rModeGain,  float rYawError, float rYawBearingRate, float rElectricalPower, float *rTorqueDemand, float *rBlade1Pitch,
        float *rBlade2Pitch, float *rBlade3Pitch, float *rPitchDemand, char *errorMsg, float* rYawRate,
        float* rLog1, float* rLog2,  float* rLog3,  float* rLog4,  float* rLog5,  float* rLog6,  float* rLog7,  float* rLog8,  float* rLog9,  float* rLog10,
        float* rLog11, float* rLog12, float* rLog13, float* rLog14, float* rLog15, float* rLog16, float* rLog17, float* rLog18, float* rLog19, float* rLog20,
        float* dKpBrLUT, float* dKiBrLUT, float* dKpArLUT, float* dKiArLUT,
        float* dWindSpeed_KpBrLUT_BP, float* dWindSpeed_KiBrLUT_BP,
        float* dWindSpeed_KpArLUT_BP, float* dWindSpeed_KiArLUT_BP,
        char* path2table) {
    int_T  i;
    real_T tnext;
    int_T  *sampleHit = rtmGetSampleHitPtr(S);
    
    SIG_MODEL(U, SimTime) = rTime;
    SIG_MODEL(U, Generator_Speed) = rGeneratorSpeed;
    SIG_MODEL(U, Wind_Speed) = rWindSpeed;
    SIG_MODEL(U, Com_dt) = rSample;
    SIG_MODEL(U,Rated_Speed) = rRatedSpeed;
    SIG_MODEL(U,Below_Rated_Pitch_Angle) = rBelowRatedPitch;
    SIG_MODEL(U,Fore_Aft_Tower_Accel) = rForeAftTower;
    SIG_MODEL(U,Sidewards_Tower_Accel) = rSideTower;
    SIG_MODEL(U,Measured_Pitch) = rMeasuredPitch;
    SIG_MODEL(U,Measured_Torque) = rMeasuredTorque;
	SIG_MODEL(U,Shaft_Torque) = rShaftTorque;
    SIG_MODEL(U,Mode_Gain) = rModeGain;
    SIG_MODEL(U,Rotor_Azimuth_Angle) = rRotorAzimuth;
    SIG_MODEL(U,Blade1_OP_Root_Moment) = rOP1RootMoment;
    SIG_MODEL(U,Blade2_OP_Root_Moment) = rOP2RootMoment;
    SIG_MODEL(U,Blade3_OP_Root_Moment) = rOP3RootMoment;
    SIG_MODEL(U,Blade1_IP_Root_Moment) = rIP1RootMoment;
    SIG_MODEL(U,Blade2_IP_Root_Moment) = rIP2RootMoment;
    SIG_MODEL(U,Blade3_IP_Root_Moment) = rIP3RootMoment;
	
	SIG_MODEL(U,Init) = rInit;
    SIG_MODEL(U, dEcho) = rUserVar1;
    SIG_MODEL(U, dRotorRadius) = rUserVar2;
    SIG_MODEL(U, dRtWSpd) = rUserVar3;
    SIG_MODEL(U, dRtTq) = rUserVar4;
    SIG_MODEL(U, dRtPwr) = rUserVar5;
    SIG_MODEL(U, dRtGenSpd) = rUserVar6;
    SIG_MODEL(U, dGenEff) = rUserVar7;
    SIG_MODEL(U, dMinTq) = rUserVar8;
    SIG_MODEL(U, dMaxTq) = rUserVar9;
    SIG_MODEL(U, dMaxRat) = rUserVar10;
    SIG_MODEL(U, dMinOMSpd) = rUserVar11;
    SIG_MODEL(U, dRgn2Mode) = rUserVar12;
    SIG_MODEL(U, dKSwitch) = rUserVar13;
    SIG_MODEL(U, dRgn2K) = rUserVar14;
    SIG_MODEL(U, dOMSpdSwitch) = rUserVar15;
    SIG_MODEL(U, dMinOMSpdSwitch) = rUserVar16;
    SIG_MODEL(U, dTSRopt) = rUserVar17;
    SIG_MODEL(U, dCpMax) = rUserVar18;
    SIG_MODEL(U, dRgn3Mode) = rUserVar19;
    SIG_MODEL(U, dMRgn3Lin) = rUserVar20;
    SIG_MODEL(U, dVLin) = rUserVar21;
    SIG_MODEL(U, dKpAr) = rUserVar22;
    SIG_MODEL(U, dKiAr) = rUserVar23;
    SIG_MODEL(U, dKpBr) = rUserVar24;
    SIG_MODEL(U, dKiBr) = rUserVar25;
    SIG_MODEL(U, dWindSpeedFSw) = rUserVar26;
    SIG_MODEL(U, dWindSpeedFLPF) = rUserVar27;
    SIG_MODEL(U, dGenSpeedFSw) = rUserVar28;
    SIG_MODEL(U, dGenSpeedFLPF) = rUserVar29;
    SIG_MODEL(U, dRefSpeedFSw) = rUserVar30;
    SIG_MODEL(U, dRefSpeedFLPF) = rUserVar31;
    SIG_MODEL(U, dRhoAir) = rUserVar32;
    SIG_MODEL(U, dFPitch) = rUserVar33;
    SIG_MODEL(U, Architecture_Selector) = rUserVar34;
    SIG_MODEL(U, dZeros_Damping) = rUserVar35;
    SIG_MODEL(U, dPoles_Damping) = rUserVar36;
    SIG_MODEL(U, dBlades_Number) = rUserVar37;
    SIG_MODEL(U, dBlades_Height) = rUserVar38;
    SIG_MODEL(U, dIntTermSwitch) = rUserVar39;
    SIG_MODEL(U, dKpBrMode) = rUserVar40;
    SIG_MODEL(U, dKiBrMode) = rUserVar41;
    SIG_MODEL(U, dKpArMode) = rUserVar42;
    SIG_MODEL(U, dKiArMode) = rUserVar43;
    SIG_MODEL(U, dRotInertia) = rUserVar44;
    SIG_MODEL(U, dAWSwitch) = rUserVar45;

    for (i = 0; i < 5; i++) {
        SIG_MODEL(U, dKpBrLUT)[i] = dKpBrLUT[i];
        SIG_MODEL(U, dKiBrLUT)[i] = dKiBrLUT[i];
        SIG_MODEL(U, dKpArLUT)[i] = dKpArLUT[i];
        SIG_MODEL(U, dKiArLUT)[i] = dKiArLUT[i];
        SIG_MODEL(U, dWindSpeed_KpBrLUT_BP)[i] = dWindSpeed_KpBrLUT_BP[i];
        SIG_MODEL(U, dWindSpeed_KiBrLUT_BP)[i] = dWindSpeed_KiBrLUT_BP[i];
        SIG_MODEL(U, dWindSpeed_KpArLUT_BP)[i] = dWindSpeed_KpArLUT_BP[i];
        SIG_MODEL(U, dWindSpeed_KiArLUT_BP)[i] = dWindSpeed_KiArLUT_BP[i];
    }

    strcpy(SIG_MODEL(U,dTablePath), path2table);
	SIG_MODEL(U,YawError) = rYawError;
	SIG_MODEL(U,YawBearingRate) = rYawBearingRate;
	SIG_MODEL(U,ElectricalPower) = rElectricalPower;
	/***********************************************
     * Check and see if base step time is too fast *
     ***********************************************/
    if (GBLbuf.isrOverrun++) {
        GBLbuf.stopExecutionFlag = 1;
        return EXIT_FAILURE;
    }

    /***********************************************
     * Check and see if error status has been set  *
     ***********************************************/
    if (rtmGetErrorStatus(S) != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return EXIT_FAILURE;
    }
    /* enable interrupts here */

    /***********************************************
     * Update discrete events                      *
     ***********************************************/
    tnext = rt_SimUpdateDiscreteEvents(rtmGetNumSampleTimes(S),
                                       rtmGetTimingData(S),
                                       rtmGetSampleHitPtr(S),
                                       rtmGetPerTaskSampleHitsPtr(S));
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(S),tnext);
    for (i=FIRST_TID+1; i < NUMST; i++) {
        if (sampleHit[i] && GBLbuf.eventFlags[i]++) {
            GBLbuf.isrOverrun--; 
            GBLbuf.overrunFlags[i]++;    /* Are we sampling too fast for */
            GBLbuf.stopExecutionFlag=1;  /*   sample time "i"?           */
            return EXIT_FAILURE;
        }
    }
    /*******************************************
     * Step the model for the base sample time *
     *******************************************/
    MdlOutputs(FIRST_TID);

    rtExtModeUploadCheckTrigger(rtmGetNumSampleTimes(S));
    rtExtModeUpload(FIRST_TID,rtmGetTaskTime(S, FIRST_TID));

    GBLbuf.errmsg = rt_UpdateTXYLogVars(rtmGetRTWLogInfo(S),
                                        rtmGetTPtr(S));
    if (GBLbuf.errmsg != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return EXIT_FAILURE;
    }

    MdlUpdate(FIRST_TID);

    if (rtmGetSampleTime(S,0) == CONTINUOUS_SAMPLE_TIME) {
        rt_UpdateContinuousStates(S);
    }
     else {
        rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S), 
                                     rtmGetTimingData(S), 0);
    }

#if FIRST_TID == 1
    rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S), 
                                 rtmGetTimingData(S),1);
#endif


    /************************************************************************
     * Model step complete for base sample time, now it is okay to          *
     * re-interrupt this ISR.                                               *
     ************************************************************************/

    GBLbuf.isrOverrun--;


    /*********************************************
     * Step the model for any other sample times *
     *********************************************/
    for (i=FIRST_TID+1; i<NUMST; i++) {
        /* If task "i" is running, don't run any lower priority task */
        if (GBLbuf.overrunFlags[i]) return EXIT_FAILURE;

        if (GBLbuf.eventFlags[i]) {
            GBLbuf.overrunFlags[i]++;

            MdlOutputs(i);
 
            rtExtModeUpload(i, rtmGetTaskTime(S,i));

            MdlUpdate(i);

            rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S), 
                                         rtmGetTimingData(S),i);

            /* Indicate task complete for sample time "i" */
            GBLbuf.overrunFlags[i]--;
            GBLbuf.eventFlags[i]--;
        }
    }

    rtExtModeCheckEndTrigger();

    
    rTorqueDemand[0] = SIG_MODEL(Y,Generator_Torque);
    /*(void)printf("\n** %f **\n", rTorqueDemand[0]);*/
    rBlade1Pitch[0] = SIG_MODEL(Y,Blade1_Pitch_Angle);
    rBlade2Pitch[0] = SIG_MODEL(Y,Blade2_Pitch_Angle);
    rBlade3Pitch[0] = SIG_MODEL(Y,Blade3_Pitch_Angle);
	rPitchDemand[0] = SIG_MODEL(Y, Collective_Pitch_Angle);
	rYawRate[0] = SIG_MODEL(Y,Yaw_Rate);
    rLog1[0] = SIG_MODEL(Y,Log1);
    rLog2[0] = SIG_MODEL(Y,Log2);
    rLog3[0] = SIG_MODEL(Y,Log3);
    rLog4[0] = SIG_MODEL(Y,Log4);
    rLog5[0] = SIG_MODEL(Y,Log5);
    rLog6[0] = SIG_MODEL(Y,Log6);
    rLog7[0] = SIG_MODEL(Y,Log7);
    rLog8[0] = SIG_MODEL(Y,Log8);
    rLog9[0] = SIG_MODEL(Y,Log9);
    rLog10[0] = SIG_MODEL(Y,Log10);
	rLog11[0] = SIG_MODEL(Y,Log11);
	rLog12[0] = SIG_MODEL(Y,Log12);
	rLog13[0] = SIG_MODEL(Y,Log13);
	rLog14[0] = SIG_MODEL(Y,Log14);
	rLog15[0] = SIG_MODEL(Y,Log15);
	rLog16[0] = SIG_MODEL(Y,Log16);
	rLog17[0] = SIG_MODEL(Y,Log17);
	rLog18[0] = SIG_MODEL(Y,Log18);
	rLog19[0] = SIG_MODEL(Y,Log19);
	rLog20[0] = SIG_MODEL(Y,Log20);


    return 0;
}  /* end calcOutputController */

#endif /* MULTITASKING */

/* Function: performCleanup ===============================================
 *
 * Abstract:
 *      Execute model on a generic target such as a workstation.
 */
int performCleanup(char *errorMsg) {
    
	#ifdef UseMMIDataLogging
		rt_CleanUpForStateLogWithMMI(rtmGetRTWLogInfo(S));
	#endif
    rt_StopDataLogging(MATFILE, rtmGetRTWLogInfo(S));
    
    rtExtModeShutdown(rtmGetNumSampleTimes(S));
    
    if (GBLbuf.errmsg) {
        (void)fprintf(stderr,"%s\n",GBLbuf.errmsg);
        exit(EXIT_FAILURE);
    }
    
    if (rtmGetErrorStatus(S) != NULL) {
        (void)fprintf(stderr,"ErrorStatus set: \"%s\"\n", rtmGetErrorStatus(S));
        exit(EXIT_FAILURE);
    }
    
    if (GBLbuf.isrOverrun) {
        (void)fprintf(stderr,
                      "%s: ISR overrun - base sampling rate is too fast\n",
                      QUOTE(MODEL));
        exit(EXIT_FAILURE);
    }
	
	#ifdef MULTITASKING
    else {
        int_T i;
        for (i=1; i<NUMST; i++) {
            if (GBLbuf.overrunFlags[i]) {
                (void)fprintf(stderr,
                        "%s ISR overrun - sampling rate is too fast for "
                        "sample time index %d\n", QUOTE(MODEL), i);
                exit(EXIT_FAILURE);
            }
        }
    }
	#endif
    
    sprintf(errorMsg, "** Stopping the controller **");
    MdlTerminate();
    
    return 0;
}  /* end performCleanup */

static void displayUsage (void)
{
    (void) printf("usage: %s -tf <finaltime> -w -port <TCPport>\n",QUOTE(MODEL));
    (void) printf("arguments:\n");
    (void) printf("  -tf <finaltime> - overrides final time specified in "
                  "Simulink (inf for no limit).\n");
    (void) printf("  -w              - waits for Simulink to start model "
                  "in External Mode.\n");
    (void) printf("  -port <TCPport> - overrides 17725 default port in "
                  "External Mode, valid range 256 to 65535.\n");
}

/* This function is added by JW to read the external inputs from Bladed */
float* SetParams(float* avrSwap)
{
    char mystring[200];
    int iStatus;

    iStatus = NINT(avrSwap[0]);

    if (iStatus == 0)
    {
        pFile = fopen("discon.in", "r");
        if (pFile == NULL) { avrSwap[128] = 1; }
        else {
            fgets(mystring, 200, pFile);                          /* [1]  HEADER */
            fgets(mystring, 200, pFile); avrSwap[130] = atof(mystring); /* [2]  dEcho */
            fgets(mystring, 200, pFile);                          /* [3]  empty */
            fgets(mystring, 200, pFile);                          /* [4]  #GEOMETRY DATA */
            fgets(mystring, 200, pFile); avrSwap[163] = atof(mystring); /* [5]  Architecture_Selector */
            fgets(mystring, 200, pFile); avrSwap[166] = atof(mystring); /* [6]  dBlades_Number */
            fgets(mystring, 200, pFile); avrSwap[131] = atof(mystring); /* [7]  dRotorRadius */
            fgets(mystring, 200, pFile); avrSwap[167] = atof(mystring); /* [8]  dBlades_Height */
            fgets(mystring, 200, pFile); avrSwap[173] = atof(mystring); /* [9]  dRotInertia */
            fgets(mystring, 200, pFile); avrSwap[161] = atof(mystring); /* [10] dRhoAir */
            fgets(mystring, 200, pFile);                          /* [11] empty */
            fgets(mystring, 200, pFile);                          /* [12] #WIND TURBINE SPECS */
            fgets(mystring, 200, pFile); avrSwap[132] = atof(mystring); /* [13] dRtWSpd */
            fgets(mystring, 200, pFile); avrSwap[133] = atof(mystring); /* [14] dRtTq */
            fgets(mystring, 200, pFile); avrSwap[134] = atof(mystring); /* [15] dRtPwr */
            fgets(mystring, 200, pFile); avrSwap[147] = atof(mystring); /* [16] dCpMax */
            fgets(mystring, 200, pFile); avrSwap[146] = atof(mystring); /* [17] dTSRopt */
            fgets(mystring, 200, pFile);                          /* [18] empty */
            fgets(mystring, 200, pFile);                          /* [19] #GENERATOR SPECS */
            fgets(mystring, 200, pFile); avrSwap[135] = atof(mystring); /* [20] dRtGenSpd */
            fgets(mystring, 200, pFile); avrSwap[136] = atof(mystring); /* [21] dGenEff */
            fgets(mystring, 200, pFile); avrSwap[137] = atof(mystring); /* [22] dMinTq */
            fgets(mystring, 200, pFile); avrSwap[138] = atof(mystring); /* [23] dMaxTq */
            fgets(mystring, 200, pFile); avrSwap[139] = atof(mystring); /* [24] dMaxRat */
            fgets(mystring, 200, pFile); avrSwap[140] = atof(mystring); /* [25] dMinOMSpd */
            fgets(mystring, 200, pFile);                          /* [26] empty */
            fgets(mystring, 200, pFile);                          /* [27] #CONTROL MODE */
            fgets(mystring, 200, pFile); avrSwap[141] = atof(mystring); /* [28] dRgn2Mode */
            fgets(mystring, 200, pFile); avrSwap[142] = atof(mystring); /* [29] dKSwitch */
            fgets(mystring, 200, pFile); avrSwap[143] = atof(mystring); /* [30] dRgn2K */
            fgets(mystring, 200, pFile); avrSwap[144] = atof(mystring); /* [31] dOMSpdSwitch */
            fgets(mystring, 200, pFile); avrSwap[145] = atof(mystring); /* [32] dMinOMSpdSwitch */
            fgets(mystring, 200, pFile); avrSwap[148] = atof(mystring); /* [33] dRgn3Mode */
            fgets(mystring, 200, pFile); avrSwap[149] = atof(mystring); /* [34] dMRgn3Lin */
            fgets(mystring, 200, pFile); avrSwap[150] = atof(mystring); /* [35] dVLin */
            fgets(mystring, 200, pFile);                          /* [36] empty */
            fgets(mystring, 200, pFile);                          /* [37] #DATA FILTERS */
            fgets(mystring, 200, pFile); avrSwap[155] = atof(mystring); /* [38] dWindSpeedFSw */
            fgets(mystring, 200, pFile); avrSwap[156] = atof(mystring); /* [39] dWindSpeedFLPF */
            fgets(mystring, 200, pFile); avrSwap[157] = atof(mystring); /* [40] dGenSpeedFSw */
            fgets(mystring, 200, pFile); avrSwap[158] = atof(mystring); /* [41] dGenSpeedFLPF */
            fgets(mystring, 200, pFile); avrSwap[159] = atof(mystring); /* [42] dRefSpeedFSw */
            fgets(mystring, 200, pFile); avrSwap[160] = atof(mystring); /* [43] dRefSpeedFLPF */
            fgets(mystring, 200, pFile); avrSwap[162] = atof(mystring); /* [44] dFPitch */
            fgets(mystring, 200, pFile); avrSwap[164] = atof(mystring); /* [45] dZeros_Damping */
            fgets(mystring, 200, pFile); avrSwap[165] = atof(mystring); /* [46] dPoles_Damping */
            fgets(mystring, 200, pFile); avrSwap[174] = atof(mystring); /* [47] dAWSwitch */
            fgets(mystring, 200, pFile); avrSwap[168] = atof(mystring); /* [48] dIntTermSwitch */
            fgets(mystring, 200, pFile);                          /* [49] empty */
            fgets(mystring, 200, pFile);                          /* [50] #GAIN SCHEDULING */
            fgets(mystring, 200, pFile); avrSwap[151] = atof(mystring); /* [51] dKpAr */
            fgets(mystring, 200, pFile); avrSwap[152] = atof(mystring); /* [52] dKiAr */
            fgets(mystring, 200, pFile); avrSwap[153] = atof(mystring); /* [53] dKpBr */
            fgets(mystring, 200, pFile); avrSwap[154] = atof(mystring); /* [54] dKiBr */
            fgets(mystring, 200, pFile); avrSwap[169] = atof(mystring); /* [55] dKpBrMode */
            fgets(mystring, 200, pFile); avrSwap[170] = atof(mystring); /* [56] dKiBrMode */
            fgets(mystring, 200, pFile); avrSwap[171] = atof(mystring); /* [57] dKpArMode */
            fgets(mystring, 200, pFile); avrSwap[172] = atof(mystring); /* [58] dKiArMode */
            fgets(mystring, 200, pFile); sscanf(mystring, "%f %f %f %f %f", &dKpBrLUT[0], &dKpBrLUT[1], &dKpBrLUT[2], &dKpBrLUT[3], &dKpBrLUT[4]); /* [59] dKpBrLUT */
            fgets(mystring, 200, pFile); sscanf(mystring, "%f %f %f %f %f", &dKiBrLUT[0], &dKiBrLUT[1], &dKiBrLUT[2], &dKiBrLUT[3], &dKiBrLUT[4]); /* [60] dKiBrLUT */
            fgets(mystring, 200, pFile); sscanf(mystring, "%f %f %f %f %f", &dKpArLUT[0], &dKpArLUT[1], &dKpArLUT[2], &dKpArLUT[3], &dKpArLUT[4]); /* [61] dKpArLUT */
            fgets(mystring, 200, pFile); sscanf(mystring, "%f %f %f %f %f", &dKiArLUT[0], &dKiArLUT[1], &dKiArLUT[2], &dKiArLUT[3], &dKiArLUT[4]); /* [62] dKiArLUT */
            fgets(mystring, 200, pFile); sscanf(mystring, "%f %f %f %f %f", &dWindSpeed_KpBrLUT_BP[0], &dWindSpeed_KpBrLUT_BP[1], &dWindSpeed_KpBrLUT_BP[2], &dWindSpeed_KpBrLUT_BP[3], &dWindSpeed_KpBrLUT_BP[4]); /* [63] */
            fgets(mystring, 200, pFile); sscanf(mystring, "%f %f %f %f %f", &dWindSpeed_KiBrLUT_BP[0], &dWindSpeed_KiBrLUT_BP[1], &dWindSpeed_KiBrLUT_BP[2], &dWindSpeed_KiBrLUT_BP[3], &dWindSpeed_KiBrLUT_BP[4]); /* [64] */
            fgets(mystring, 200, pFile); sscanf(mystring, "%f %f %f %f %f", &dWindSpeed_KpArLUT_BP[0], &dWindSpeed_KpArLUT_BP[1], &dWindSpeed_KpArLUT_BP[2], &dWindSpeed_KpArLUT_BP[3], &dWindSpeed_KpArLUT_BP[4]); /* [65] */
            fgets(mystring, 200, pFile); sscanf(mystring, "%f %f %f %f %f", &dWindSpeed_KiArLUT_BP[0], &dWindSpeed_KiArLUT_BP[1], &dWindSpeed_KiArLUT_BP[2], &dWindSpeed_KiArLUT_BP[3], &dWindSpeed_KiArLUT_BP[4]); /* [66] */
            fclose(pFile);
        }
    }
    return(avrSwap);
}


/*===================*
 * Visible functions *
 *===================*/


/* Function: main =============================================================
 *
 * Abstract:
 *      Execute model on a generic target such as a workstation.
 */
void __declspec(dllexport) __cdecl DISCON(float *avrSwap, int *aviFail, char *accInfile, char *avcOutname, char *avcMsg) 
{
	int iStatus, iFirstLog;
    char errorMsg[257], OutName[1025], path2table[200];// inFile[257]; 
	float rTime, rSample, rGeneratorSpeed, rWindSpeed, rRatedSpeed, rBelowRatedPitch, 
			rRotorAzimuth, rOP1RootMoment, rOP2RootMoment, rOP3RootMoment, rIP1RootMoment, rIP2RootMoment, rIP3RootMoment,
			rForeAftTower, rSideTower, rMeasuredPitch, rMeasuredTorque, rShaftTorque,
			rModeGain, rInit, rYawError, rYawBearingRate, rElectricalPower;
    static float  rUserVar1, rUserVar2, rUserVar3, rUserVar4, rUserVar5, rUserVar6, rUserVar7, rUserVar8, rUserVar9, rUserVar10,
        rUserVar11, rUserVar12, rUserVar13, rUserVar14, rUserVar15, rUserVar16, rUserVar17, rUserVar18, rUserVar19, rUserVar20, rUserVar21, rUserVar22, rUserVar23, rUserVar24,
        rUserVar25, rUserVar26, rUserVar27, rUserVar28, rUserVar29, rUserVar30, rUserVar31, rUserVar32, rUserVar33, rUserVar34, rUserVar35, rUserVar36, rUserVar37, rUserVar38,
        rUserVar39, rUserVar40, rUserVar41, rUserVar42, rUserVar43, rUserVar44, rUserVar45;
	static float rTorqueDemand, rPitchDemand, rBlade1Pitch, rBlade2Pitch, 
			rBlade3Pitch, rYawRate, rLog1,rLog2,rLog3,rLog4,rLog5,rLog6,rLog7,rLog8,rLog9,rLog10,rLog11,rLog12,rLog13,rLog14,rLog15,rLog16,rLog17,rLog18,rLog19,rLog20;
    static char mystring[200];

	/* Take local copies of strings */
	//memcpy(inFile, accInfile, NINT(avrSwap[49]));
	//inFile[NINT(avrSwap[49])+1] = '\0';
	//memcpy(outName, avcOutname, NINT(avrSwap[50]));
	//outName[NINT(avrSwap[50])+1] = '\0';
	
	/* Set message to blank */
	memset(errorMsg, ' ', 257);
    memset(path2table, ' ', 200); 

    /*(void)printf("\n** %s **\n", path2table);*/
	
	/* Set constants JW turned this on, see function just above this call*/ 
	SetParams(avrSwap); /*PF disable this call for Labview's sake*/
    
    /* Load variables from Bladed (See Appendix A) */
	iStatus          = NINT(avrSwap[0]);
	rInit			= avrSwap[0];
	rTime            = avrSwap[1];
	rSample          = avrSwap[2];
	rMeasuredPitch   = avrSwap[3];
	rBelowRatedPitch = avrSwap[4];
	rModeGain        = avrSwap[15];
	rRatedSpeed      = avrSwap[18];
	rGeneratorSpeed  = avrSwap[19];
    rWindSpeed       = avrSwap[26];
	rMeasuredTorque  = avrSwap[22]; /* this was number 22 but I believe it should be 108 but that is the LSS Torque */
	rShaftTorque	 = avrSwap[108];
	rOP1RootMoment   = avrSwap[29]; 
	rOP2RootMoment   = avrSwap[30];
	rOP3RootMoment   = avrSwap[31];
	rIP1RootMoment   = avrSwap[68]; 
	rIP2RootMoment   = avrSwap[69];
	rIP3RootMoment   = avrSwap[70];
	rForeAftTower    = avrSwap[52];
	rSideTower       = avrSwap[53];
	rRotorAzimuth    = avrSwap[59];
	rYawError		 = avrSwap[23];
	rYawBearingRate  = avrSwap[180];
	rElectricalPower = avrSwap[14];

    if (iStatus == 0) {
        rUserVar1 = avrSwap[130];   /* dEcho */
        rUserVar2 = avrSwap[131];   /* dRotorRadius */
        rUserVar3 = avrSwap[132];   /* dRtWsp */
        rUserVar4 = avrSwap[133];   /* dRtTq */
        rUserVar5 = avrSwap[134];   /* dRtPwr */
        rUserVar6 = avrSwap[135];   /* dRtGenSpd */
        rUserVar7 = avrSwap[136];   /* dGenEff */
        rUserVar8 = avrSwap[137];   /* dMinTq */
        rUserVar9 = avrSwap[138];   /* dMaxTq */
        rUserVar10 = avrSwap[139];   /* dMaxRat */
        rUserVar11 = avrSwap[140];   /* dMinOMSpd */
        rUserVar12 = avrSwap[141];   /* dRgn2Mode */
        rUserVar13 = avrSwap[142];   /* dKSwitch */
        rUserVar14 = avrSwap[143];   /* dRgn2K */
        rUserVar15 = avrSwap[144];   /* dOMSpdSwitch */
        rUserVar16 = avrSwap[145];   /* dMinOMSpdSwitch */
        rUserVar17 = avrSwap[146];   /* dTSRopt */
        rUserVar18 = avrSwap[147];   /* dCpMax */
        rUserVar19 = avrSwap[148];   /* dRgn3Mode */
        rUserVar20 = avrSwap[149];   /* dMRgn3Lin */
        rUserVar21 = avrSwap[150];   /* dVLin */
        rUserVar22 = avrSwap[151];   /* dKpAr */
        rUserVar23 = avrSwap[152];   /* dKiAr */
        rUserVar24 = avrSwap[153];   /* dKpBr */
        rUserVar25 = avrSwap[154];   /*dKiBr */
        rUserVar26 = avrSwap[155];   /* dWindSpeedFSw */
        rUserVar27 = avrSwap[156];   /*dWindSpeedFLPF */
        rUserVar28 = avrSwap[157];   // dGenSpeedFSw
        rUserVar29 = avrSwap[158];   /* dGenSpeedFLPF */
        rUserVar30 = avrSwap[159];   // dRefSpeedFSw
        rUserVar31 = avrSwap[160];   // RefSpeedFLPF
        rUserVar32 = avrSwap[161];   // dRhoAir
        rUserVar33 = avrSwap[162];   // dFPitch
        rUserVar34 = avrSwap[163];   // Architecture_Selector
        rUserVar35 = avrSwap[164];   //dZeros_Damping
        rUserVar36 = avrSwap[165];   //dPoles_Damping
        rUserVar37 = avrSwap[166];   //dBlades_Number
        rUserVar38 = avrSwap[167];   //dBlades_Height
        rUserVar39 = avrSwap[168];   //dIntTermSwitch
        rUserVar40 = avrSwap[169];   //dKpBrMode
        rUserVar41 = avrSwap[170];   //dKiBrMode
        rUserVar42 = avrSwap[171];   //dKpArMode
        rUserVar43 = avrSwap[172];   //dKiArMode
        rUserVar44 = avrSwap[173];   /* dRotInertia */   
        rUserVar45 = avrSwap[174];   /* dAWSwitch */ 

        pFile = fopen("discon.in", "r");
        if (pFile == NULL) {}
        else
        {
            fgets(mystring, 200, pFile); /* [1]  HEADER */
            fgets(mystring, 200, pFile); /* [2]  dEcho */
            fgets(mystring, 200, pFile); /* [3]  empty */
            fgets(mystring, 200, pFile); /* [4]  #GEOMETRY DATA */
            fgets(mystring, 200, pFile); /* [5]  Architecture_Selector */
            fgets(mystring, 200, pFile); /* [6]  dBlades_Number */
            fgets(mystring, 200, pFile); /* [7]  dRotorRadius */
            fgets(mystring, 200, pFile); /* [8]  dBlades_Height */
            fgets(mystring, 200, pFile); /* [9]  dRotInertia */
            fgets(mystring, 200, pFile); /* [10] dRhoAir */
            fgets(mystring, 200, pFile); /* [11] empty */
            fgets(mystring, 200, pFile); /* [12] #WIND TURBINE SPECS */
            fgets(mystring, 200, pFile); /* [13] dRtWSpd */
            fgets(mystring, 200, pFile); /* [14] dRtTq */
            fgets(mystring, 200, pFile); /* [15] dRtPwr */
            fgets(mystring, 200, pFile); /* [16] dCpMax */
            fgets(mystring, 200, pFile); /* [17] dTSRopt */
            fgets(mystring, 200, pFile); /* [18] empty */
            fgets(mystring, 200, pFile); /* [19] #GENERATOR SPECS */
            fgets(mystring, 200, pFile); /* [20] dRtGenSpd */
            fgets(mystring, 200, pFile); /* [21] dGenEff */
            fgets(mystring, 200, pFile); /* [22] dMinTq */
            fgets(mystring, 200, pFile); /* [23] dMaxTq */
            fgets(mystring, 200, pFile); /* [24] dMaxRat */
            fgets(mystring, 200, pFile); /* [25] dMinOMSpd */
            fgets(mystring, 200, pFile); /* [26] empty */
            fgets(mystring, 200, pFile); /* [27] #CONTROL MODE */
            fgets(mystring, 200, pFile); /* [28] dRgn2Mode */
            fgets(mystring, 200, pFile); /* [29] dKSwitch */
            fgets(mystring, 200, pFile); /* [30] dRgn2K */
            fgets(mystring, 200, pFile); /* [31] dOMSpdSwitch */
            fgets(mystring, 200, pFile); /* [32] dMinOMSpdSwitch */
            fgets(mystring, 200, pFile); /* [33] dRgn3Mode */
            fgets(mystring, 200, pFile); /* [34] dMRgn3Lin */
            fgets(mystring, 200, pFile); /* [35] dVLin */
            fgets(mystring, 200, pFile); /* [36] empty */
            fgets(mystring, 200, pFile); /* [37] #DATA FILTERS */
            fgets(mystring, 200, pFile); /* [38] dWindSpeedFSw */
            fgets(mystring, 200, pFile); /* [39] dWindSpeedFLPF */
            fgets(mystring, 200, pFile); /* [40] dGenSpeedFSw */
            fgets(mystring, 200, pFile); /* [41] dGenSpeedFLPF */
            fgets(mystring, 200, pFile); /* [42] dRefSpeedFSw */
            fgets(mystring, 200, pFile); /* [43] dRefSpeedFLPF */
            fgets(mystring, 200, pFile); /* [44] dFPitch */
            fgets(mystring, 200, pFile); /* [45] dZeros_Damping */
            fgets(mystring, 200, pFile); /* [46] dPoles_Damping */
            fgets(mystring, 200, pFile); /* [47] dAWSwitch */
            fgets(mystring, 200, pFile); /* [48] dIntTermSwitch */
            fgets(mystring, 200, pFile); /* [49] empty */
            fgets(mystring, 200, pFile); /* [50] #GAIN SCHEDULING */
            fgets(mystring, 200, pFile); /* [51] dKpAr */
            fgets(mystring, 200, pFile); /* [52] dKiAr */
            fgets(mystring, 200, pFile); /* [53] dKpBr */
            fgets(mystring, 200, pFile); /* [54] dKiBr */
            fgets(mystring, 200, pFile); /* [55] dKpBrMode */
            fgets(mystring, 200, pFile); /* [56] dKiBrMode */
            fgets(mystring, 200, pFile); /* [57] dKpArMode */
            fgets(mystring, 200, pFile); /* [58] dKiArMode */
            fgets(mystring, 200, pFile); /* [59] dKpBrLUT */
            fgets(mystring, 200, pFile); /* [60] dKiBrLUT */
            fgets(mystring, 200, pFile); /* [61] dKpArLUT */
            fgets(mystring, 200, pFile); /* [62] dKiArLUT */
            fgets(mystring, 200, pFile); /* [63] dWindSpeed_KpBrLUT_BP */
            fgets(mystring, 200, pFile); /* [64] dWindSpeed_KiBrLUT_BP */
            fgets(mystring, 200, pFile); /* [65] dWindSpeed_KpArLUT_BP */
            fgets(mystring, 200, pFile); /* [66] dWindSpeed_KiArLUT_BP */
            fgets(mystring, 200, pFile); /* [67] PATH */
            strcpy(path2table, mystring);
            fclose(pFile);
        }
    }
    else
    {
        strcpy(path2table, mystring);
        //(void)printf("\n** %s **\n", path2table); 

    }
    
	
    /* determine iStatus */
    aviFail[0] = 0;
    if (iStatus == 0) {
        
                /* Initialize Controller */
        aviFail[0] = initiateController(errorMsg);
        
                aviFail[0] = calcOutputController(rUserVar1, rUserVar2, rUserVar3, rUserVar4, rUserVar5,rUserVar6, rUserVar7, rUserVar8, rUserVar9, rUserVar10,
						rUserVar11,rUserVar12,rUserVar13,rUserVar14,rUserVar15,rUserVar16,rUserVar17,rUserVar18,rUserVar19,rUserVar20, rUserVar21, rUserVar22, rUserVar23, rUserVar24,
                        rUserVar25, rUserVar26, rUserVar27, rUserVar28, rUserVar29, rUserVar30, rUserVar31, rUserVar32, rUserVar33, rUserVar34, rUserVar35, rUserVar36, rUserVar37, rUserVar38, rUserVar39, rUserVar40, rUserVar41, rUserVar42, rUserVar43, rUserVar44, rUserVar45,
                        rTime, rInit, rSample, rGeneratorSpeed, rWindSpeed, rRatedSpeed,
                        rBelowRatedPitch, rForeAftTower, rSideTower,
                        rRotorAzimuth, rOP1RootMoment, rOP2RootMoment,
                        rOP3RootMoment, rIP1RootMoment, rIP2RootMoment,
                        rIP3RootMoment, rMeasuredPitch, rMeasuredTorque, rShaftTorque,
                        rModeGain, rYawError, rYawBearingRate, rElectricalPower, &rTorqueDemand, &rBlade1Pitch,
                        &rBlade2Pitch, &rBlade3Pitch, &rPitchDemand, errorMsg, &rYawRate,
                        &rLog1,&rLog2,&rLog3,&rLog4,&rLog5,&rLog6,&rLog7,&rLog8,&rLog9,&rLog10,&rLog11,&rLog12,&rLog13,&rLog14,&rLog15,&rLog16,&rLog17,&rLog18,&rLog19,&rLog20, dKpBrLUT, dKiBrLUT, dKpArLUT, dKiArLUT, dWindSpeed_KpBrLUT_BP,
                        dWindSpeed_KiBrLUT_BP, dWindSpeed_KpArLUT_BP, dWindSpeed_KiArLUT_BP, path2table);
        

        rPitchDemand = rMeasuredPitch;    
        rTorqueDemand = rMeasuredTorque;
		sprintf(errorMsg, "Controller initialization complete");
    }
    else if (iStatus >= 0) {
        /* Main calculation */
        aviFail[0] = calcOutputController(rUserVar1, rUserVar2, rUserVar3, rUserVar4, rUserVar5,rUserVar6, rUserVar7, rUserVar8, rUserVar9, rUserVar10,
						rUserVar11,rUserVar12,rUserVar13,rUserVar14,rUserVar15,rUserVar16,rUserVar17,rUserVar18,rUserVar19,rUserVar20, rUserVar21, rUserVar22, rUserVar23, rUserVar24,
                        rUserVar25, rUserVar26, rUserVar27, rUserVar28, rUserVar29, rUserVar30, rUserVar31, rUserVar32, rUserVar33, rUserVar34, rUserVar35, rUserVar36, rUserVar37, rUserVar38, rUserVar39, rUserVar40, rUserVar41, rUserVar42, rUserVar43, rUserVar44, rUserVar45,
                        rTime, rInit, rSample, rGeneratorSpeed, rWindSpeed, rRatedSpeed,
                        rBelowRatedPitch, rForeAftTower, rSideTower,
                        rRotorAzimuth, rOP1RootMoment, rOP2RootMoment,
                        rOP3RootMoment, rIP1RootMoment, rIP2RootMoment,
                        rIP3RootMoment, rMeasuredPitch, rMeasuredTorque, rShaftTorque,
                        rModeGain, rYawError, rYawBearingRate, rElectricalPower, &rTorqueDemand, &rBlade1Pitch, 
                        &rBlade2Pitch, &rBlade3Pitch, &rPitchDemand, errorMsg, &rYawRate,
                        &rLog1,&rLog2,&rLog3,&rLog4,&rLog5,&rLog6,&rLog7,&rLog8,&rLog9,&rLog10,&rLog11,&rLog12,&rLog13,&rLog14,&rLog15,&rLog16,&rLog17,&rLog18,&rLog19,&rLog20, dKpBrLUT, dKiBrLUT, dKpArLUT, dKiArLUT, dWindSpeed_KpBrLUT_BP, 
                        dWindSpeed_KiBrLUT_BP, dWindSpeed_KpArLUT_BP, dWindSpeed_KiArLUT_BP,  path2table);
    
        /* sprintf("%f", &rTorqueDemand); */
    }
    else if (iStatus == -1) {
        /* Main calculation */
        aviFail[0] = calcOutputController(rUserVar1, rUserVar2, rUserVar3, rUserVar4, rUserVar5,rUserVar6, rUserVar7, rUserVar8, rUserVar9, rUserVar10,
						rUserVar11,rUserVar12,rUserVar13,rUserVar14,rUserVar15,rUserVar16,rUserVar17,rUserVar18,rUserVar19,rUserVar20, rUserVar21, rUserVar22, rUserVar23, rUserVar24, rUserVar25, rUserVar26, rUserVar27, rUserVar28, rUserVar29, rUserVar30, rUserVar31, rUserVar32, rUserVar33, rUserVar34, rUserVar35, rUserVar36, rUserVar37, rUserVar38, rUserVar39, rUserVar40, rUserVar41, rUserVar42, rUserVar43, rUserVar44, rUserVar45,
                        rTime, rInit, rSample, rGeneratorSpeed, rWindSpeed, rRatedSpeed,
                        rBelowRatedPitch, rForeAftTower, rSideTower,
                        rRotorAzimuth, rOP1RootMoment, rOP2RootMoment,
                        rOP3RootMoment, rIP1RootMoment, rIP2RootMoment,
                        rIP3RootMoment, rMeasuredPitch, rMeasuredTorque, rShaftTorque,
                        rModeGain, rYawError, rYawBearingRate, rElectricalPower, &rTorqueDemand, &rBlade1Pitch, 
                        &rBlade2Pitch, &rBlade3Pitch, &rPitchDemand, errorMsg, &rYawRate,
                        &rLog1,&rLog2,&rLog3,&rLog4,&rLog5,&rLog6,&rLog7,&rLog8,&rLog9,&rLog10,&rLog11,&rLog12,&rLog13,&rLog14,&rLog15,&rLog16,&rLog17,&rLog18,&rLog19,&rLog20, dKpBrLUT, dKiBrLUT, dKpArLUT, dKiArLUT, dWindSpeed_KpBrLUT_BP, dWindSpeed_KiBrLUT_BP, dWindSpeed_KpArLUT_BP, dWindSpeed_KiArLUT_BP, 
                        path2table);
        /* sprintf("%f", &rTorqueDemand); */
        /* Perform Cleanup */
        aviFail[0] = performCleanup(errorMsg); 
    }
    else {
        aviFail[0] = -1;
        sprintf(errorMsg, "iStatus is not recognized: %d", iStatus);
    }
    
    /* Store variables to Bladed (See Appendix A) */
    avrSwap[9] = 0; /* Pitch Angle */
	avrSwap[27] = 1; /* Individual Pitch control */
    avrSwap[34] = 1; /* Generator contactor status */
    avrSwap[35] = 0; /* Shaft brake status: 0=off */
    avrSwap[40] = 0; /* Demanded yaw actuator torque */
    avrSwap[41] = rBlade1Pitch;  /* Blade 1 pitch angle demand */
    avrSwap[42] = rBlade2Pitch;  /* Blade 2 pitch angle demand */
    avrSwap[43] = rBlade3Pitch;  /* Blade 3 pitch angle demand */
	avrSwap[44] = rPitchDemand;  /* Pitch angle demand CPC*/
    avrSwap[46] = rTorqueDemand; /* Generator torque demand */
    avrSwap[47] = rYawRate; /* Demanded nacelle yaw rate */
    avrSwap[54] = 0; /* Pitch override */
    avrSwap[55] = 0; /* Torque override */
	avrSwap[71] = 0; /* Generator start-up resistance */
    avrSwap[78] = 1; /* Request for loads: 0=none */
    avrSwap[79] = 0; /* Variable slip current status */
    avrSwap[80] = 0; /* Variable slip current demand */
   /*  (void)printf("\n** %f **\n", avrSwap[46]); */
	// To read the log variables in bladed (JW)
	avrSwap[64] =0; /* Number of variables returned for logging */
	iFirstLog = NINT(avrSwap[62])-1; //added also this iFirstLog as an integer
	strcpy(OutName, "Log1:-;Log2:-;Log3:-;Log4:-;Log5:-;Log6:-;Log7:-;Log8:-;Log9:-;Log10:-;Log11:-;Log12:-;Log13:-;Log14:-;Log15:-;Log16:-;Log17:-;Log18:-;Log19:-;Log20:-;"); //Names and units
    avrSwap[iFirstLog] =rLog1; /*avrSwap[144] = rLog1;*/
	avrSwap[iFirstLog+1] =rLog2; /*avrSwap[144] = rLog2;*/
	avrSwap[iFirstLog+2] =rLog3; /*avrSwap[144] = rLog3;*/
	avrSwap[iFirstLog+3] =rLog4; /*avrSwap[144] = rLog4;*/
	avrSwap[iFirstLog+4] =rLog5; /*avrSwap[144] = rLog5;*/
	avrSwap[iFirstLog+5] =rLog6; /*avrSwap[144] = rLog6;*/
	avrSwap[iFirstLog+6] =rLog7; /*avrSwap[144] = rLog7;*/
	avrSwap[iFirstLog+7] =rLog8; /*avrSwap[144] = rLog8;*/
	avrSwap[iFirstLog+8] =rLog9; /*avrSwap[144] = rLog9;*/
	avrSwap[iFirstLog+9] =rLog10; /*avrSwap[144] = rLog10;*/
	avrSwap[iFirstLog+10] =rLog11; /*avrSwap[144] = rLog11;*/
	avrSwap[iFirstLog+11] =rLog12; /*avrSwap[144] = rLog12;*/
	avrSwap[iFirstLog+12] =rLog13; /*avrSwap[144] = rLog13;*/
	avrSwap[iFirstLog+13] =rLog14; /*avrSwap[144] = rLog14;*/
	avrSwap[iFirstLog+14] =rLog15; /*avrSwap[144] = rLog15;*/
	avrSwap[iFirstLog+15] =rLog16; /*avrSwap[144] = rLog16;*/
	avrSwap[iFirstLog+16] =rLog17; /*avrSwap[144] = rLog17;*/
	avrSwap[iFirstLog+17] =rLog18; /*avrSwap[144] = rLog18;*/
	avrSwap[iFirstLog+18] =rLog19; /*avrSwap[144] = rLog19;*/
	avrSwap[iFirstLog+19] =rLog20; /*avrSwap[144] = rLog20;*/

    //Return strings
	memcpy(avcOutname,OutName, NINT(avrSwap[63]));
	memcpy(avcMsg,errorMsg,MIN(256,NINT(avrSwap[48])));
    //memcpy(test, mystring, strlen(path2table) + 1);
  return;
}
		  /* end DISON */



/* EOF: discon_main.c */
