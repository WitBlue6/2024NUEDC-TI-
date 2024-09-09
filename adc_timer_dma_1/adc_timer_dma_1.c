/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arm_math.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/driverlib/dl_adc12.h"
#include "ti/driverlib/dl_timera.h"
#include "ti/driverlib/dl_timerg.h"
#include "ti_msp_dl_config.h"
#include "fft.h"
#include "oled.h"
#include "stdio.h"

#define ADC_SAMPLE_SIZE (256)
//liu------------
#define DUTY_THER 2000
#define CAP_THES 150
#define CAP_NUM 3
#define DC_BIAS 2000
#define DC_VOLTAGE 1.61
#define DC_NC 560
#define ROUND_TO_UINT16(x)   ((uint8_t)(x)+0.5)>(x)? ((uint8_t)(x)):((uint8_t)(x)+1) //将浮点数四舍五入为uint16_t

volatile bool t_flag;
volatile uint16_t ADC_val;
uint16_t duty_cnt=0;
uint16_t arr_cnt=0;
float cap_max=0;
bool cap_flag = false;
bool cap_stop = false;
float cap_time;
uint16_t CAP_arr[ADC_SAMPLE_SIZE];
uint16_t cap_cnt=0;
volatile float duty_cycle_ns;
volatile float duty_cycle_ms;
volatile uint64_t tim_counter;
volatile uint16_t sys_cnt;
float wave_get[12];
//------------
//hou---------
uint8_t anxia;
uint8_t num;
char    str[60];
char    str1[30];
char    str2[30];
char    str3[30];
char    str4[30];
uint8_t x_pos;
uint8_t y_pos;
uint32_t val1;
uint8_t OLED_GRAM[128][8];
float amp_y;
float t_x;
int   wave_type;
float amp_max;
float amp_min;  
int freq[5];
int freq_1[5];
float duty;
char wave[4][10];
int squ_wave[128]={0};
int sin_wave[5][128];
int sin_wave_simple[128]={27,30,32,35,37,39,41,43,45,47,49,50,51,52,53,53,53,53,53,52,51,50,49,47,45,43,41,39,37,35,32,30,27,24,22,19,17,15,13,11,9 ,7 ,5 ,4 ,3 ,2 ,1 ,1 ,1 ,1 ,1 ,2 ,3 ,4 ,5 ,7 ,9 ,11,13,15,17,19,22,24,27,30,32,35,37,39,41,43,45,47,49,50,51,52,53,53,53,53,53,52,51,50,49,47,45,43,41,39,37,35,32,30,27,24,22,19,17,15,13,11,9 ,7 ,5 ,4 ,3 ,2 ,1 ,1 ,1 ,1 ,1 ,2 ,3 ,4 ,5 ,7 ,9 ,11,13,15,17,19,22,24
};
int amp_wave[5];
int amp_wave_1[5];

int tri_wave[128];
//----------------------------
//--------------------------------------------------------------------------------------------------
/* 舵机相关 */

#define MAX_NUM_SERVO 2
#define PWM_PERIOD_COUNT     (20000)
#define PWM_MAX_PERIOD_COUNT              (PWM_PERIOD_COUNT - 100)


int servo_angle[2];//存的是几何角度
uint16_t servo_pwm[2];//存的是角度经过PwmServo_Angle_To_Pulse()计算转换后得到的占空比
uint16_t current_pwm_pulse=1500;

/* 三次样条插值 */
double timestep = 0.01;
double tf = 1;
static double theta0_yaw,theta0_pitch; 
double thetai_yaw,thetai_pitch; //起始角度 
double thetaf_yaw,thetaf_pitch; //目标角度
double a0;
double a1;
double a2;
double a3;
/****************/
int xy_index=0;
int8_t flag_wave=0;//0表示正弦波，1表示方波，2表示三角波
double y_laser_sine[16]={14,18,20,18,14,10,8,10,14,20,22,20,14,10,8,10};
double x_laser_sine[16]={76,78,79,81,83,86,86,88,90,92,94,95,97,99,101,102};

double y_laser_square[16]={20,20,20,20,8 ,8 ,8 ,8, 22,22,22,22,8 ,8, 8,  8};
double x_laser_square[16]={76,78,79,81,81,85,86,90,90,92,94,97,97,99,101,102};

double y_laser_tri[16]={4,12,16,20,30,20,16,12,4,12,16,20,30,20,16,12};
double x_laser_tri[16]={76,78,79,81,83,85,86,90,90,90,94,95,97,99,101,102};

uint16_t gADCSamples[ADC_SAMPLE_SIZE];
volatile bool gCheckADC;
extern struct compx s[ADC_SAMPLE_SIZE];
extern float result[ADC_SAMPLE_SIZE];
int max=0;

void findMax(uint16_t re[]);
void findMaxMin(uint16_t arr[], uint16_t size, int *max, int *min);
float* wave_style(float error, float error_v, float *wave_wave);
void pulse_capture(uint16_t *cap);
float float2int(float ffreq);
void KEY_SCAN(uint8_t *key);
void DrawPoint(uint8_t x,uint8_t y);
void wavesqu(int freq,float duty_circle);
void wavesin(int freq[],int amp_wavesin[]);
void wavesin_simple(int freq[]);
void wavetri(int8_t freq);
void oled_update(void);
void set_duty(uint32_t CompareValue, uint8_t channel);
void PwmServo_Handle();
double cubicsp(double _thetai, double _thetaf,double _tf,double t);
void PwmServo_Set_All(int angle_s1,int angle_s2);
void PwmServo_Init(void);
uint16_t PwmServo_Angle_To_Pulse(int angle);
void get_back(float *after);
void get_amp(float *after);

int main(void)
{
    SYSCFG_DL_init();
    //liu------------
    //Init
    tim_counter = 0;
    ADC_val = 0;
    t_flag = false;
    //hou-------------
    /* LED on by default */
    OLED_Init();
    sprintf(wave[0],"sine");
    sprintf(wave[1],"squa");
    sprintf(wave[2],"trig");
    sprintf(wave[3],"cplx");
    //liuzhuo-----------
    PwmServo_Init();
    /* Configure DMA source, destination and size */
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,(uint32_t)0x40556280);

    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &gADCSamples[0]);

    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    /* Setup interrupts on device */
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    gCheckADC = false;


    while (false == gCheckADC) {
       // __WFE();

    }    
    while (1) {
        KEY_SCAN(&num);
        if(gCheckADC==true){
            int j=0;
            for(j=0;j<ADC_SAMPLE_SIZE;j++)
            {   
                s[j].real=gADCSamples[j];
                s[j].imag=0;
            }
            FFT(s);
            GetMag();
            wave_style(0.6, 0.7, wave_get);
            gCheckADC=false;
        }
        else if(gCheckADC==false && num!=0){
            //hou---------------------------------------------
            oled_update();        
           
            amp_y=(float)5*(amp_max-amp_min)/224;
            t_x  =(float)1/64/freq[0];
            if(num == 2){   
                OLED_Clear();
                sprintf(str,"%.3fV/%.4fms",amp_y,1000*t_x);
                //画横坐标
                for(int k=0;k<128;k++){
                    DrawPoint(k,29);
                }
                //画纵坐标
                for(int k=0;k<58;k++){
                    DrawPoint(64,k);
                }
                OLED_ShowString(6,57,(uint8_t *)str,8);
                //判断波形
                if(wave_type == 2){  //三角波
                    wavetri(1);
                    for(int k=0;k<128;k++){
                        DrawPoint(k, (tri_wave[k]));
                    } 
                    duty = 0;
                }
                else if(wave_type == 0){//正弦波
                    //OLED_Clear();
                    int freq_temp;
                    freq_temp = freq[0];
                    freq[0] =1;
                    //wavesin(&freq[0],&amp_wave[0]);
                    freq[0] = freq_temp;
                    for(int k=0;k<128;k++){
                    DrawPoint(k, (sin_wave_simple[k]));
                    } 
                    duty = 0;
                }
                else if(wave_type == 1){//方波
                    //OLED_Clear();
                    wavesqu(1,duty);
                    for(int k=0;k<128;k++){
                    DrawPoint(k, squ_wave[k]);
                    } 
                }
                else if(wave_type == 3){//复杂信号
                }
            }
            /*
            
            */
            //delay_ms(1000);
            else if(num == 3){
                
                sprintf(str1,"wave:%s",wave[wave_type]);
                //if(wave_type == 1)
                    //amp_min += 0.2*amp_max;
                sprintf(str2,"U:%.2fv %.2fv",amp_max,amp_min);
                sprintf(str3,"freq:%dHz",freq[0]);
                sprintf(str4,"duty:%.1f%%",100*duty);
                OLED_ShowString(18,0,(uint8_t *)str1,16);
                OLED_ShowString(0,2,(uint8_t *)str2,16);
                OLED_ShowString(18,4,(uint8_t *)str3,16);
                OLED_ShowString(18,6,(uint8_t *)str4,16);
                //delay_ms(1000);
                
            }
            else if(num == 4) {
                OLED_Clear();
            }
            else if(num == 5){
                OLED_Clear();
                //画横坐标
                for(int k=0;k<128;k++){
                    DrawPoint(k,29);
                }
                //画纵坐标
                for(int k=0;k<58;k++){
                    DrawPoint(64,k);
                }
                //freq_1[4]=ROUND_TO_UINT16(freq_1[4]*1.0/freq_1[0]);
                freq_1[3]=ROUND_TO_UINT16(freq_1[3]*1.0/freq_1[0]);
                freq_1[2]=ROUND_TO_UINT16(freq_1[2]*1.0/freq_1[0]);
                freq_1[1]=ROUND_TO_UINT16(freq_1[1]*1.0/freq_1[0]);
                freq_1[0] = 1;
                //amp_wave_1[4]=amp_wave_1[0]/amp_wave_1[4];
                amp_wave_1[3]=amp_wave_1[0]/amp_wave_1[3];
                amp_wave_1[2]=(amp_wave_1[0]/amp_wave_1[2]);
                amp_wave_1[1]=(amp_wave_1[0]/amp_wave_1[1]);
                amp_wave_1[0]=1;
                wavesin(&freq_1[0],&amp_wave_1[0]);
                for(int k=0;k<128;k++){
                    DrawPoint(k, (sin_wave[0][k]+sin_wave[1][k]+sin_wave[2][k]+sin_wave[3][k])/2+5);
                }                
            }
            else if(num == 6){
                float tmpp = (float)((amp_wave[0]*3.3)/4095);
                get_amp(&tmpp);
                sprintf(str,"1:%.3fv,%dHz",tmpp,freq[0]);
                tmpp = (float)((amp_wave[1]*3.3)/4095);
                get_amp(&tmpp);
                sprintf(str1,"2:%.3fv,%dHz",tmpp,freq[1]);
                tmpp = (float)((amp_wave[2]*3.3)/4095);
                get_amp(&tmpp);
                sprintf(str2,"3:%.3fv,%dHz",tmpp,freq[2]);
                tmpp = (float)((amp_wave[3]*3.3)/4095);
                get_amp(&tmpp);
                sprintf(str3,"4:%.3fv,%dHz",tmpp,freq[3]);
                tmpp = (float)((amp_wave[4]*3.3)/4095);
                get_amp(&tmpp);
                sprintf(str4,"5:%.3fv,%dHz",tmpp,freq[4]);
                OLED_ShowString(0, 0, str, 8);
                OLED_ShowString(0, 1, str1, 8);
                OLED_ShowString(0, 2, str2, 8);
                OLED_ShowString(0, 3, str3, 8);
                OLED_ShowString(0, 4, str4, 8);
            }
            else if(num == 7){
                char str[30];
                OLED_Clear();
                if(cap_max != 0)
                    get_back(&cap_max);
                sprintf(str,"max:%.3fv t:%.2fms",cap_max, cap_time);
                OLED_ShowString(0, 0, str, 8);
                //CAP_arr[256]
                int temppp[128];
                int k;
                for(k=0;k<128;k++){
                    temppp[k] =CAP_arr[k]*56.0/4096;
                    DrawPoint(k+1, 60-temppp[k]);
                }
            }
            else if(num == 8){
                NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);
                DL_TimerG_startCounter(TIMER_1_INST);
                num=9;
            }
        }
    }
        
    
}

void ADC12_0_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            ADC_val = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
            arr_cnt++;
            if(ADC_val >= DUTY_THER)
                duty_cnt++;
            if(arr_cnt == ADC_SAMPLE_SIZE){
                wave_get[11] = 1.0*duty_cnt/ADC_SAMPLE_SIZE;
                arr_cnt=0;
                duty_cnt=0;
            }
            pulse_capture(CAP_arr);
            break;
        case DL_ADC12_IIDX_WINDOW_COMP_HIGH:
            cap_flag = true;
            cap_stop = false;
            break;
        case DL_ADC12_IIDX_WINDOW_COMP_LOW:
            cap_flag = false;
            if(cap_cnt)
                cap_time = cap_cnt*0.05;
            cap_stop = true;
            break;
        case DL_ADC12_IIDX_DMA_DONE:
            DL_TimerA_stopCounter(TIMER_0_INST);//先把采集关了，处理完了再继续采
            //__BKPT();
            gCheckADC=true;            
            DL_TimerA_startCounter(TIMER_0_INST);
            break;
        default:
            break;
    }
}

void TIMER_1_INST_IRQHandler(void){
    switch(DL_TimerG_getPendingInterrupt(TIMER_1_INST)){
        case DL_TIMERG_IIDX_ZERO:
            if(xy_index<16){
                switch((int)wave_get[0]){
                    case 0://正弦波
                        thetaf_yaw = PwmServo_Angle_To_Pulse((int)x_laser_sine[xy_index]);
                        thetaf_pitch = PwmServo_Angle_To_Pulse((int)y_laser_sine[xy_index]);
                        PwmServo_Handle();
                        break;
                    case 1://方波

                        thetaf_yaw = PwmServo_Angle_To_Pulse((int)x_laser_square[xy_index]);
                        thetaf_pitch = PwmServo_Angle_To_Pulse((int)y_laser_square[xy_index]);
                        PwmServo_Handle();
                        break;
                    case 2://三角波
                        thetaf_yaw = PwmServo_Angle_To_Pulse((int)x_laser_tri[xy_index]);
                        thetaf_pitch = PwmServo_Angle_To_Pulse((int)y_laser_tri[xy_index]);
                        PwmServo_Handle();
                        break;
                    default:
                        break;    
                }

                xy_index++;
            }

            break;
        default:
            break;
    }
}

void findMax(uint16_t re[]) {
    // 初始化最大值和最小值
    int p=0;
    int re_max = 0;
    // 遍历数组找到最大值和最小值
    for (p = 1; p < ADC_SAMPLE_SIZE; p++) {
        if (re[p] > re_max) {
            re_max = re[p];
        }
    }
    cap_max = 3.3*re_max/4095;
}

void findMaxMin(uint16_t arr[], uint16_t size, int *max, int *min) {
    // 初始化最大值和最小值
    *max = arr[0];
    *min = arr[0];
    // 遍历数组找到最大值和最小值
    for (int i = 1; i < size; i++) {
        if (arr[i] > *max) {
            *max = arr[i];
        }
        if (arr[i] < *min) {
            *min = arr[i];
        }
    }
}

float* wave_style(float error, float error_v, float *wave_wave){
    //error为频率判断的误差许可, error_v为幅度判断的误差许可
    //0正弦, 1方波, 2三角波
    float peak[ADC_SAMPLE_SIZE/2]={0};
    int peak_id[ADC_SAMPLE_SIZE/2]={0};
    float w_max[5]={0};
    int w_max_id[5]={-1};
    int i, j, t;
    //滤直流
    result[0] = 0;
    //获取fft数据中的极大值
    for(i=1, j=0;i<ADC_SAMPLE_SIZE/2;i++){
        if(result[i]>1+result[i-1]&&result[i]>1+result[i+1]){
            peak[j] = result[i];
            peak_id[j++] = i+1;
        }
    }
    //获取极大值前5个最大值及其索引
    int cur = 0;
    for(t=0;t<5;t++){
        for(i=0;i<j;i++){
            if(peak[i]>w_max[t]){
                w_max[t] = peak[i];
                w_max_id[t] = peak_id[i];
                cur = i;
            }
        }
        //消除目前最大值
        peak[cur] = 0;
        peak_id[cur] = -1;
    }
    //float wave_wave[11];
    if((1.0*w_max_id[1]/w_max_id[0]>=3-error && 1.0*w_max_id[1]/w_max_id[0]<=3+error) || 1.0*w_max[0]/w_max[1]<=9+3*error_v){
        if(1.0*w_max[0]/w_max[1]>=3-error_v && 1.0*w_max[0]/w_max[1]<=3+error_v){
            wave_wave[0] = 1;
        }
        else if(1.0*w_max[0]/w_max[1]>=9-3*error_v && 1.0*w_max[0]/w_max[1]<=9+3*error_v){
            wave_wave[0] = 2;
        }
        // else{
        //     return 3;
        // }
    }
    else{
        wave_wave[0] = 0;
    }
    if(w_max_id[0]==0){
        wave_wave[0] = 0;
    }
    for(i=1;i<=5;i++){
        wave_wave[i] = w_max[i-1];
        wave_wave[i+5] = w_max_id[i-1]*20000.0/ADC_SAMPLE_SIZE;       
    }
    wave_wave[6] = float2int(wave_wave[6]); 
    return wave_wave;
}

void pulse_capture(uint16_t *cap){
    if(cap_stop){
        cap_cnt = 0;
        return;
    }
    if(cap_flag){
        if(cap_cnt == ADC_SAMPLE_SIZE){
            cap_cnt = 0;
            cap_flag = false;
            //cap_stop = true;
            return;
        }
        cap[cap_cnt++] = ADC_val;
        return;        
    }
}

float float2int(float ffreq){
    if(ffreq<160 && ffreq>100)
        return 101.1;
    else if(ffreq>170 && ffreq<250)
        return 152.8;
    else if(ffreq>260 && ffreq <320)
        return 202.7;
    else if(ffreq>320 && ffreq <420)
        return ffreq;
    else if(ffreq>420 && ffreq <500)
        return ffreq;
    else if(ffreq >500 && ffreq<1200)
        return ffreq-50;
    else
        return ffreq;
}

//hou----------------------------
void KEY_SCAN(uint8_t *key)
{   
    //第一行
    DL_GPIO_clearPins(GPIOB, GPIO_SWITCHES_SWITCH_1_PIN);
    DL_GPIO_setPins(GPIOB, GPIO_SWITCHES_SWITCH_2_PIN);
    DL_GPIO_setPins(GPIOA, GPIO_SWITCHES_SWITCH_3_PIN|GPIO_SWITCHES_SWITCH_4_PIN);
    val1 = DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN);
    if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) != 0x001020C0)
    {
        delay_ms(10);
        if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) == 0x00102040 )
        {
            anxia = 1;
            *key  = 1;
            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) == 0x00102080)
        {
            anxia = 1;
            *key  = 2;
            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) == 0x001000C0)
        {
            anxia = 1;
            *key  = 3;
            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)  == 0x000020C0)
        {
            anxia = 1;
            *key  = 4;
            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else{
            anxia = 0;
            DL_GPIO_setPins(GPIOB, GPIO_SWITCHES_SWITCH_1_PIN|GPIO_SWITCHES_SWITCH_2_PIN);
            DL_GPIO_setPins(GPIOA, GPIO_SWITCHES_SWITCH_3_PIN|GPIO_SWITCHES_SWITCH_4_PIN);
        }

    }
    //第二行
    DL_GPIO_setPins(GPIOB, GPIO_SWITCHES_SWITCH_1_PIN);
    DL_GPIO_clearPins(GPIOB, GPIO_SWITCHES_SWITCH_2_PIN);
    DL_GPIO_setPins(GPIOA, GPIO_SWITCHES_SWITCH_3_PIN|GPIO_SWITCHES_SWITCH_4_PIN);
    val1 = DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN);
    //__BKPT();
    if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) != 0x001020C0)
    {
        delay_ms(10);
        if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) == 0x00102040 )
        {
            anxia = 1;
            *key  = 5;

            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) == 0x00102080)
        {
            anxia = 1;
            *key  = 6;
            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) == 0x001000C0)
        {
            anxia = 1;
            *key  = 7;
            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)  == 0x000020C0)
        {
            anxia = 1;
            *key  = 8;
            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else{
            anxia = 0;
            DL_GPIO_setPins(GPIOB, GPIO_SWITCHES_SWITCH_1_PIN|GPIO_SWITCHES_SWITCH_2_PIN);
            DL_GPIO_setPins(GPIOA, GPIO_SWITCHES_SWITCH_3_PIN|GPIO_SWITCHES_SWITCH_4_PIN);
        }

    }
    //第三行
    DL_GPIO_setPins(GPIOB, GPIO_SWITCHES_SWITCH_1_PIN|GPIO_SWITCHES_SWITCH_2_PIN);
    DL_GPIO_clearPins(GPIOA, GPIO_SWITCHES_SWITCH_3_PIN);
    DL_GPIO_setPins(GPIOA, GPIO_SWITCHES_SWITCH_4_PIN);
    if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) != 0x001020C0)
    {
        delay_ms(10);
        if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) == 0x00102040 )
        {
            anxia = 1;
            *key  = 9;

            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) == 0x00102080)
        {
            anxia = 1;
            *key  = 10;
            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) == 0x001000C0)
        {
            anxia = 1;
            *key  = 11;
            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)  == 0x000020C0)
        {
            anxia = 1;
            *key  = 12;
            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else{
            anxia = 0;
            DL_GPIO_setPins(GPIOB, GPIO_SWITCHES_SWITCH_1_PIN|GPIO_SWITCHES_SWITCH_2_PIN);
            DL_GPIO_setPins(GPIOA, GPIO_SWITCHES_SWITCH_3_PIN|GPIO_SWITCHES_SWITCH_4_PIN);
        }

    }
    //第四行
    DL_GPIO_setPins(GPIOB, GPIO_SWITCHES_SWITCH_1_PIN|GPIO_SWITCHES_SWITCH_2_PIN);
    DL_GPIO_setPins(GPIOA, GPIO_SWITCHES_SWITCH_3_PIN);
    DL_GPIO_clearPins(GPIOA, GPIO_SWITCHES_SWITCH_4_PIN);
    if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) != 0x001020C0)
    {
        delay_ms(10);
        if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) == 0x00102040 )
        {
            anxia = 1;
            *key  = 13;

            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) == 0x00102080)
        {
            anxia = 1;
            *key  = 14;
            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN) == 0x001000C0)
        {
            anxia = 1;
            *key  = 15;
            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else if(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)  == 0x000020C0)
        {
            anxia = 1;
            *key  = 16;
            while(DL_GPIO_readPins(GPIOB,GPIO_SWITCHES_SWITCH_5_PIN|GPIO_SWITCHES_SWITCH_6_PIN|GPIO_SWITCHES_SWITCH_7_PIN|GPIO_SWITCHES_SWITCH_8_PIN)!=  0x001020C0);
        }
        else{
            anxia = 0;
            DL_GPIO_setPins(GPIOB, GPIO_SWITCHES_SWITCH_1_PIN|GPIO_SWITCHES_SWITCH_2_PIN);
            DL_GPIO_setPins(GPIOA, GPIO_SWITCHES_SWITCH_3_PIN|GPIO_SWITCHES_SWITCH_4_PIN);
        }

    }
   
}

//画点
void DrawPoint(uint8_t x,uint8_t y){
    uint8_t j,m,n;
    j=y/8;
    m=y%8;
    n=1<<m;
    OLED_GRAM[x][j]|=n;
    
    OLED_Set_Pos(x, j);
    OLED_WR_Byte(OLED_GRAM[x][j],OLED_DATA);
}

//方波信号
void wavesqu(int freq,float duty_circle)
{
    int N=128;
    int x=0;
    int ha=0;
    for(x=0;x<N;x++){
        ha = ha + freq;
        if( 
            ( (x) < (int)64*duty_circle )|| 
            ( (x) < (int)64*duty_circle + 64) && ( (x) >= 64)
        ){//第1段
            squ_wave[x]=1;
        }else{
           squ_wave[x]=29; 
        }
    }
}

void wavesin(int freq[],int amp_wavesin[]){
    int sin_wave_1[128]={27,30,32,35,37,39,41,43,45,47,49,50,51,52,53,53,53,53,53,52,51,50,49,47,45,43,41,39,37,35,32,30,27,24,22,19,17,15,13,11,9 ,7 ,5 ,4 ,3 ,2 ,1 ,1 ,1 ,1 ,1 ,2 ,3 ,4 ,5 ,7 ,9 ,11,13,15,17,19,22,24,27,30,32,35,37,39,41,43,45,47,49,50,51,52,53,53,53,53,53,52,51,50,49,47,45,43,41,39,37,35,32,30,27,24,22,19,17,15,13,11,9 ,7 ,5 ,4 ,3 ,2 ,1 ,1 ,1 ,1 ,1 ,2 ,3 ,4 ,5 ,7 ,9 ,11,13,15,17,19,22,24
};
    int ha = 0;
    for(int m=0;m<5;m++){
    for(int k=0;k<128;k++){
        ha = ha + freq[m];
        sin_wave[m][k] = (sin_wave_1[ha%128])/amp_wavesin[m];
    }
    }
}


void wavetri(int8_t freq){
    int tri_wave_1[128]={1 ,3 ,4 ,6 ,8 ,9 ,11,12,14,16,17,19,21,22,24,25,27,29,30,32,34,35,37,38,40,42,43,45,47,48,50,51,53,51,50,48,47,45,43,42,40,38,37,35,34,32,30,29,27,25,24,22,20,19,17,16,14,12,11,9 ,8 ,6 ,4 ,3 ,1 ,3 ,4 ,6 ,8 ,9 ,11,12,14,16,17,19,20,22,24,25,27,29,30,32,34,35,37,38,40,42,43,45,46,48,50,51,53,51,50,48,46,45,43,42,40,38,37,35,34,32,30,29,27,25,24,22,20,19,17,16,14,12,11,9 ,8 ,6 ,4 ,3 
};
    int ha = 0;
    for(int k=0;k<128;k++){
        ha = ha +freq;
        tri_wave[k] = tri_wave_1[ha%128];
    }
}

void oled_update(void){
    duty = wave_get[11];
    //ADC采样值还原
    int k;
    for(k=0;k<5;k++){
            freq[k]=wave_get[6+k];
            freq_1[k]=freq[k];
            amp_wave[k]=wave_get[k+1];
            amp_wave_1[k]=wave_get[k+1];
            //if(amp_wave[k] == 0) amp_wave[k] = 30; 
    }
    wave_type = wave_get[0];
    int mmax=0, mmin=0;
    DL_TimerA_stopCounter(TIMER_0_INST);
    findMaxMin(gADCSamples, ADC_SAMPLE_SIZE, &mmax, &mmin);
    DL_TimerA_startCounter(TIMER_0_INST);
    amp_max = ((float)mmax/4095)*3.3;
    amp_min = ((float)mmin/4095)*3.3;
    get_back(&amp_max);
    get_back(&amp_min);
    findMax(CAP_arr);
}

uint16_t PwmServo_Angle_To_Pulse(int angle)
{
    angle = 180 - angle;
	uint16_t pulse = (200*angle+18*500)/18;
	return pulse;
	
}

void PwmServo_Init(void)
{

	for (int i = 0; i < MAX_NUM_SERVO; i++)
	{
		servo_angle[i] = 90;
		servo_pwm[i] = PwmServo_Angle_To_Pulse(servo_angle[i]) ;		
	}

	
    thetai_yaw = PwmServo_Angle_To_Pulse(90-14);
    thetai_pitch = PwmServo_Angle_To_Pulse(6);

	set_duty(thetai_yaw, 0);
    set_duty(thetai_pitch, 1); 

}

void PwmServo_Set_All(int angle_s1,int angle_s2)
{
	if(angle_s1<= 180){
		servo_angle[0] = angle_s1;
		servo_pwm[0] = PwmServo_Angle_To_Pulse(angle_s1) ;
	}
	
	if(angle_s2<= 180){
		servo_angle[1] = angle_s2;
		servo_pwm[1] = PwmServo_Angle_To_Pulse(angle_s2) ;
	}	

}


double cubicsp(double _thetai, double _thetaf,double _tf,double t){
      //a0 = 70;a1 = 0;a2 = 30 ;a3 = -10;
      //double thetai=45; double thetaf=135;
      double _theta0;
      a0 = _thetai;
      a1 = 0;
      a2 = 3/(pow(_tf,2))*(_thetaf - _thetai);
      a3 = -2/(pow(_tf,3))*(_thetaf - _thetai);
      _theta0 = a0 + a1*(t) +a2*(pow(t,2)) +a3*(pow(t,3));
      return _theta0; 
}

void PwmServo_Handle()
{	
/* 三次样条插值 */
	
    /*YAW*/
    for (double t = 0; t <= tf; t +=timestep) { // goes from 0 degrees to 180 degrees in 1 second(t~tf)
        theta0_yaw =cubicsp(thetai_yaw,thetaf_yaw,tf,t);
        theta0_pitch =cubicsp(thetai_pitch,thetaf_pitch,tf,t);
        //theta0_pitch=theta0_yaw;
        set_duty(theta0_yaw,0);
        set_duty(theta0_pitch,1);
    }
    //__BKPT();
    thetai_yaw = thetaf_yaw;
    thetai_pitch = thetaf_pitch;
	
	
}

void set_duty(uint32_t CompareValue, uint8_t channel){
    if(channel == 0)
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, CompareValue, DL_TIMER_CC_0_INDEX);
    else
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, CompareValue, DL_TIMER_CC_1_INDEX);
}

void get_back(float *after){
    *after = *after - 1.5;
    *after = (*after - 0.173 )/ 0.335; 
}
void get_amp(float *after){
    if((*after - 0.173 )/ 0.335 < 0){
        *after = *after * 1.8;
        return;
    }
    *after = (*after - 0.173 )/ 0.335;
}