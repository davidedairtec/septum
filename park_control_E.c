/*
 * File : conti.c
 * Abstract:
 *       An example C-file S-function for measuring two signals and doing 
 *       their product
 *
 */


#define S_FUNCTION_NAME  park_control_E
#define S_FUNCTION_LEVEL 2

#define TPWM Tsampling //Tpwm en seg
#define Tsampling 100e-6 //Tiene que coincidir con el valor del tiempo de muestreo Ts de Simulink



#include "simstruc.h"

#include "math.h"


/*Variables globales*/

//// Entradas ////

float Vabc[3]; // Vector de tensiones Fase-Neutro

float Iabc[3]; // Vector de corrientes de Fase

///// Transformadas de Clarke y Parl //////

float Ialf; // Corriente alfa de la transformada de Clarke
float Ibet; // Corriente beta de la transformada de Clarke
float Valf; // Tensión alfa de la transformada de Clarke
float Vbet; // Tensión beta de la transformada de Clarke
float kalbe; // Constante de la transformada de Clarke

float p_alf_bet, q_alf_bet; // Potencias instantaneas en ejes alfa-beta y d-q
float p_dq, q_dq;

float theta, thetaant; // Angulo de giro para la transformada de Park
float omega, omegaant; // Velocidad de giro de la tranformada de Park
float Vd, Vq, Vqant; // Tensiones de y q de la transformada de Park y Vq(k-1)
float Id, Iq; // Corrientes d y q de la transformada de Park

///// Valores del control PI para calculo de Omega /////

//float Kp=2, Ki=100; //Valores iniciales del control
float Kp=6, Ki=800; // Bajar tiempo de establecimiento a un ciclo Kp=6, Ki=800
//float Kp=8, Ki=700; // Reducir SO inicial Kp = 8, Ki = 700
//float Kp=2, Ki=100;
///// Anti-Transformada /////

float Valf_dq, Vbet_dq; // Tensiones alfa y beta de la anti-transformada de Park
float Va_alfbet, Vb_alfbet, Vc_alfbet; // Tensiones a,b,c de la anti-transformada de Clarke 

float Ialf_dq, Ibet_dq; // Corrientes alfa y beta de la anti-transformada de Park
float Ia_alfbet, Ib_alfbet, Ic_alfbet; // Corrientes a,b,c de la anti-transformada de Clarke

///// Lazo de Control interno /////

float Sd, Sd_ant; // Salidas del PI de Ud
float Sq, Sq_ant; // Salidas del PI de Uq

float e_id, e_id_ant; // Error de la corriente Id
float e_iq, e_iq_ant; // Error de la corriente Iq

//float Kp_int = 40, Ki_int = 4000; // Constantes del control del PI del lazo interno

//float Kp_int = 10, Ki_int = 1000; 
//float Kp_int = 0.001, Ki_int = 0.002; 
float Kp_int = 2.5e-2, Ki_int = 1e-3; 

float Id_des, Iq_des; // Corrientes dq deseadas
float Ud, Uq; // Señales de control finales de tensión

float p_des, q_des; // Potencias activa y reactiva deseadas

float L = 12e-5; // Valor de la bobina de red. 5 mH

float Ualf_dq, Ubet_dq; // Tensiones alfa y beta de la anti-transformada de Park
float Ua_alfbet, Ub_alfbet, Uc_alfbet; // Tensiones a,b,c de la anti-transformada de Clarke 

///// Lazo de Control externo /////

float Vdc, Vdc_des; // Tensiones del condensador, medida y de referencia

float Z, Z_des; // Potencia del condesador

float e_Z, e_Z_ant; // Error de la potencia del condesador

float p_des_ant; // Salidas del PI externo anterior

//float Kp_ext = 0.2, Ki_ext = 5; // Constantes del control del PI del lazo externo
//float Kp_ext = 1, Ki_ext =2; 
float Kp_ext = 0.1, Ki_ext =0.2; 

/*================*
 * Build checking *
 *================*/


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    if (!ssSetNumInputPorts(S, 3)) return;
    //ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
    ssSetInputPortWidth(S, 0, 3);
    ssSetInputPortDirectFeedThrough(S, 0, 1); //Por cada entrada que pongamos, hay que poner esta linea, para indicarle a Matlab que la S-Function va a utilizar ese puerto para hacer cosas
    ssSetInputPortWidth(S, 1, 3);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortWidth(S, 2, 3);
    ssSetInputPortDirectFeedThrough(S, 2, 1);

    if (!ssSetNumOutputPorts(S,8)) return;
    //ssSetOutputPortWidth(S, 0, DYNAMICALLY_SIZED);
    ssSetOutputPortWidth(S, 0, 2);
    ssSetOutputPortWidth(S, 1, 2);
    ssSetOutputPortWidth(S, 2, 2);
    ssSetOutputPortWidth(S, 3, 2);
    ssSetOutputPortWidth(S, 4, 2);
    ssSetOutputPortWidth(S, 5, 3);
    ssSetOutputPortWidth(S, 6, 3);
    ssSetOutputPortWidth(S, 7, 3);

    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    //ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetSampleTime(S, 0, Tsampling);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}

#define MDL_START                      /* Change to #undef to remove function */
#if defined(MDL_START)
/* Function: mdlStart ==========================================================
 * Abstract:
 *
 */
static void mdlStart(SimStruct *S) //Función que solo se ejecuta la primera vez que se llama al bloque, de forma que se pueden poner aqui los valores iniciales
{
    theta = 0;
    thetaant = 0;
    Vqant = 0;
    omega = 0;
    omegaant = 0;
    e_id_ant = 0;
    e_iq_ant = 0;
    Sd_ant = 0;
    Sq_ant = 0;
    p_des_ant = 0;
    e_Z_ant = 0;
}
#endif /*  MDL_START */


/* Function: mdlOutputs =======================================================
 * Abstract:
 *    
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    //int_T             i;
    InputRealPtrsType pVabc = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType pIabc = ssGetInputPortRealSignalPtrs(S,1);
    InputRealPtrsType pDCQ = ssGetInputPortRealSignalPtrs(S,2);

    //Salidas
    real_T            *y1    = ssGetOutputPortRealSignal(S,0);
    real_T            *y2    = ssGetOutputPortRealSignal(S,1);
    real_T            *y3    = ssGetOutputPortRealSignal(S,2);
    real_T            *y4    = ssGetOutputPortRealSignal(S,3);
    real_T            *y5    = ssGetOutputPortRealSignal(S,4);
    real_T            *y6    = ssGetOutputPortRealSignal(S,5);
    real_T            *y7    = ssGetOutputPortRealSignal(S,6);
    real_T            *y8    = ssGetOutputPortRealSignal(S,7);

    
    //Cálculos
    
    // Obtención de los valores de entrada para trabajar con ellos
    Vabc[0] = *pVabc[0];
    Vabc[1] = *pVabc[1]; 
    Vabc[2] = *pVabc[2]; 
    
    Iabc[0] = *pIabc[0];
    Iabc[1] = *pIabc[1];
    Iabc[2] = *pIabc[2];

    Vdc = *pDCQ[0];
    Vdc_des = *pDCQ[1];
    q_des = *pDCQ[2];
    
    // Construcción de la matriz de Clarke//

    kalbe = sqrt(2./3.); // Power Invariant
    //kalbe = 2./3.; //Non-Power Invariant

    //Se calcula la representación de clarke // 

    Valf = kalbe*(Vabc[0]-(Vabc[1]/2.)-(Vabc[2]/2.));
    Vbet = kalbe*(0*Vabc[0]+(Vabc[1]*sqrt(3.)/2.)-(Vabc[2]*sqrt(3.)/2.));
    Ialf = kalbe*(Iabc[0]-(Iabc[1]/2.)-(Iabc[2]/2.));
    Ibet = kalbe*(0*Iabc[0]+(Iabc[1]*sqrt(3.)/2.)-(Iabc[2]*sqrt(3.)/2.));
 
    // Trasformada de Park //

    Vd = Valf*cos(thetaant) + Vbet*sin(thetaant);
    Vq = -Valf*sin(thetaant) + Vbet*cos(thetaant);
    Id = Ialf*cos(thetaant) + Ibet*sin(thetaant);
    Iq = -Ialf*sin(thetaant) + Ibet*cos(thetaant);

    // Calculos de potencias activas y  reactivas instantaneas //

    p_alf_bet = (Valf*Ialf) + (Vbet*Ibet);
    q_alf_bet = (Vbet*Ialf) - (Valf*Ibet);

    p_dq = (Vd*Id) + (Vq*Iq);
    q_dq = (Vq*Id) - (Vd*Iq);

    // Control PI para obtener el angulo de giro //

    omega = omegaant + Kp*(Vq-Vqant) + Ki*Vqant*Tsampling;
    theta = thetaant + omegaant*Tsampling;
    Vqant = Vq;
    omegaant = omega;
    thetaant = theta;

    ////// Lazo de control externo //////

    // Calculo de las potencias
    Z = (Vdc*Vdc)/2.;
    Z_des = (Vdc_des*Vdc_des)/2.;

    // Cálculo del error
    e_Z = Z_des - Z;

    // PI del lazo secundario
    p_des = p_des_ant + Kp_ext*(e_Z-e_Z_ant) + Ki_ext*e_Z_ant*Tsampling;


    ////// Lazo de control interno //////

    // Cálculo de Idq_des
    Id_des = (p_des*Vd + q_des*Vq) / (Vd*Vd + Vq*Vq); // Corriente d deseada
    Iq_des = (p_des*Vq - q_des*Vd) / (Vd*Vd + Vq*Vq); // Corriente q deseada

    // Cálculo de los errores
    e_id = Id - Id_des; // Error de Id
    e_iq = Iq - Iq_des; // Error de Iq

    // Controladores PI
    Sd = Sd_ant + Kp_int*(e_id-e_id_ant) + Ki_int*e_id_ant*Tsampling; // PI para Id
    Sq = Sq_ant + Kp_int*(e_iq-e_iq_ant) + Ki_int*e_iq_ant*Tsampling; // PI para Iq

    // Calculo de Udq
    Ud = Vd + Sd + Iq*omega*L; // Señal de control en tensión d
    Uq = Vq + Sq - Id*omega*L; // Señal de control en tensión q

    // Actualizo valores
    e_id_ant = e_id;
    e_iq_ant = e_iq;
    Sd_ant = Sd;
    Sq_ant = Sq;

    //// Anti-transformada de Park /////

    Ualf_dq = Ud*cos(thetaant) - Uq*sin(thetaant);
    Ubet_dq = Ud*sin(thetaant) + Uq*cos(thetaant);

    Ialf_dq = Id*cos(thetaant) - Iq*sin(thetaant);
    Ibet_dq = Id*sin(thetaant) + Iq*cos(thetaant);

    // Anti-transformada de Clarke //

    Ua_alfbet = (Ualf_dq/kalbe)*(2./3.);
    Ub_alfbet = (Ualf_dq/kalbe)*(-1./3.) + (Ubet_dq/kalbe)*(1./sqrt(3));
    Uc_alfbet = (Ualf_dq/kalbe)*(-1./3.) + (Ubet_dq/kalbe)*(-1./sqrt(3));

    Ia_alfbet = (Ialf_dq/kalbe)*(2./3.);
    Ib_alfbet = (Ialf_dq/kalbe)*(-1./3.) + (Ibet_dq/kalbe)*(1./sqrt(3));
    Ic_alfbet = (Ialf_dq/kalbe)*(-1./3.) + (Ibet_dq/kalbe)*(-1./sqrt(3));


    ///// Asignación de las salidas /////

    // Salida 1. Tensión dq
    y1[0] = Vd;
    y1[1] = Vq;
    // Salida 2. Corriente dq
    y2[0] = Id;
    y2[1] = Iq;
    // Salida 3. Potencia dq
    y3[0] = p_dq;
    y3[1] = q_dq;
    // Salida 4. Angulo y velocidad de sincronismo
    y4[0] = omega;
    y4[1] = theta;
    // Salida 5. Corriente Idq deseada
    y5[0] = Id_des;
    y5[1] = Iq_des;
    // Salida 5. Anti-transformada de tensión alfa-beta -> Red
    y6[0] = Ua_alfbet;
    y6[1] = Ub_alfbet;
    y6[2] = Uc_alfbet;
    // Salida 6. Anti-transformada de corriente alfa-beta -> Red
    y7[0] = Ia_alfbet;
    y7[1] = Ib_alfbet;
    y7[2] = Ic_alfbet;
    // Salida 7. Potencia dq deseadas
    y8[0] = p_des;
    y8[1] = q_des;
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
