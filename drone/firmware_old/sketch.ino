#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"
#include "Wire.h"
#include "SPI.h"

#define LED_PIN 22
#define BUZ_PIN 23
#define PWM1_PIN 5 // OC3A
#define PWM2_PIN 2 // OC3B
#define PWM3_PIN 3 // OC3C
#define PWM4_PIN 6 // OC4A
#define PWM5_PIN 7 // OC4B
#define PWM6_PIN 8 // OC4C

#define MPU_PKT_SIZE 42 // XXX

#define PWM_50HZ
//#define PWM_400HZ

MPU6050 mpu;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
int mpu_fifo_count;
uint32_t mpu_num_samples=0;
uint32_t mpu_num_resets=0;

int16_t mpu_off[6];
char do_update_mpu_off=0;

int16_t cur_q[4];
int16_t cur_accel[3];
int16_t cur_gyro[3];
int16_t cur_rxz,cur_ryz,cur_rzz,cur_rxx,cur_rxy;

int32_t accel_filter[3];
int16_t smooth_accel[3];

int16_t pitch_speed,roll_speed,yaw_speed;
int16_t prev_pitch_speed,prev_roll_speed,prev_yaw_speed;

int16_t target_rxz_base=0,target_ryz_base=0;
int16_t target_rxz=0,target_ryz=0,target_rxx=1<<14,target_rxy=0;
int16_t target_pitch_speed,target_roll_speed,target_yaw_speed;
int16_t target_x_speed, target_y_speed;

HMC5883L compass;
int16_t comp_x,comp_y,comp_z;
int16_t scale_comp_x,scale_comp_y;

uint16_t pwm_durations[6];

uint16_t throttle=0;
int32_t throttle_filt;
int32_t throttle_rate;

int16_t ctrl_pitch_speed_p=0, ctrl_pitch_speed_d=0, ctrl_pitch_speed_i=0;
int16_t ctrl_roll_speed_p=0, ctrl_roll_speed_d=0, ctrl_roll_speed_i=0;
int16_t ctrl_yaw_speed_p=0, ctrl_yaw_speed_d=0, ctrl_yaw_speed_i=0;
int16_t ctrl_pitch_p=0;
int16_t ctrl_roll_p=0;
int16_t ctrl_yaw_p=0;
int32_t ctrl_pitch_speed_imax=0;
int32_t ctrl_roll_speed_imax=0;
int32_t ctrl_yaw_speed_imax=0;
int16_t ctrl_pitch_speed_max;
int16_t ctrl_roll_speed_max;
int16_t ctrl_yaw_speed_max;
int16_t ctrl_alt_p;
int16_t ctrl_alt_speed_max;
int16_t ctrl_alt_speed_p,ctrl_alt_speed_d,ctrl_alt_speed_i;
int32_t ctrl_alt_speed_imax;
int16_t ctrl_throttle_min,ctrl_throttle_max,ctrl_throttle_hover;
int16_t ctrl_x_speed_p;
int16_t ctrl_y_speed_p;
int16_t ctrl_x_speed_i;
int16_t ctrl_y_speed_i;
int16_t ctrl_x_speed_d;
int16_t ctrl_y_speed_d;
int32_t ctrl_x_speed_imax=0;
int32_t ctrl_y_speed_imax=0;
int16_t ctrl_x_pos_p;
int16_t ctrl_y_pos_p;
int16_t ctrl_x_speed_max;
int16_t ctrl_y_speed_max;

int16_t err_pitch_p;
int16_t err_roll_p;
int16_t err_yaw_p;
int16_t err_pitch_speed_p,err_pitch_speed_d;
int16_t err_roll_speed_p,err_roll_speed_d;
int16_t err_yaw_speed_p,err_yaw_speed_d;
int32_t err_pitch_speed_i;
int32_t err_roll_speed_i;
int32_t err_yaw_speed_i;
int32_t err_alt_p,err_alt_d,err_alt_i;
int16_t err_alt_speed_p,err_alt_speed_d;
int32_t err_alt_speed_i;
int32_t err_x_speed_p;
int32_t err_y_speed_p;
int32_t err_x_speed_i;
int32_t err_y_speed_i;

int32_t adj_roll_speed_p,adj_roll_speed_d,adj_roll_speed_i,adj_roll_speed;
int32_t adj_pitch_speed_p,adj_pitch_speed_d,adj_pitch_speed_i,adj_pitch_speed;
int32_t adj_yaw_speed_p,adj_yaw_speed_d,adj_yaw_speed_i,adj_yaw_speed;
int32_t adj_alt_speed_p,adj_alt_speed_d,adj_alt_speed_i,adj_alt_speed;
int32_t adj_x_speed_p,adj_x_speed_i,adj_x_speed_d;
int32_t adj_y_speed_p,adj_y_speed_i,adj_y_speed_d;

uint32_t last_motion_t=0;
uint32_t last_baro_t=0;
uint32_t last_compass_t=0;
uint32_t last_bat_t=0;
uint32_t last_check_t=0;
char received_cmd=0;
uint32_t last_alt_t;

uint16_t bat_volt, bat_amp;

#define ALT_OFFSET 8900000

#define MS5611_ADDRESS 0x77

#define MS5611_CMD_ADC_READ 0x00
#define MS5611_CMD_RESET 0x1E
#define MS5611_CMD_CONV_D1 0x40
#define MS5611_CMD_CONV_D2 0x50
#define MS5611_CMD_READ_PROM 0xA2

#define MS5611_ULTRA_HIGH_RES 0x08

#define BARO_MODE_INIT 0
#define BARO_MODE_P 1
#define BARO_MODE_T 2
uint8_t baro_mode=0;
uint8_t baro_prev_mode=0;

uint32_t baro_p,baro_t;
uint16_t baro_prom[6];
int32_t baro_pressure;
int32_t baro_prev_pressure;
int32_t baro_speed;
int32_t baro_pressure_filter=0;
int32_t baro_speed_filter=0;
int32_t baro_t_new,baro_t_start,baro_t_step,baro_t_diff;

int32_t K_ALT=64;
int32_t K_SPEED=128;
int32_t BARO_DT=40;

int32_t alt_sens;
int32_t alt_pred,alt_speed_pred;
int32_t alt_filt,alt_speed_filt;
int32_t cur_alt;
int16_t cur_alt_speed;
int16_t prev_alt_speed;
int32_t err_alt;
int32_t err_alt_total;
int32_t target_alt;
int16_t target_alt_speed;
int16_t target_accel_z;

int32_t gps_lat,gps_lon,gps_alt;
int32_t gps_hacc,gps_vacc;
uint8_t gps_num_sat;
int16_t gps_north_speed,gps_east_speed,gps_down_speed;
char gps_enable;

int32_t GPS_DT=200;
int32_t K_BODY_X_SPEED=200;
int32_t K_BODY_X_ACCEL=300;
int32_t K_BODY_Y_SPEED=200;
int32_t K_BODY_Y_ACCEL=300;

int16_t body_x_speed,body_y_speed;
int32_t body_x_speed_filt,body_y_speed_filt;
int32_t body_x_accel_filt,body_y_accel_filt;
int32_t body_x_speed_smooth,body_y_speed_smooth;
int16_t body_x_accel,body_y_accel;
int16_t target_body_x_speed,target_body_y_speed;
int32_t x_pos_filt,x_pos;
int32_t y_pos_filt,y_pos;
int32_t target_lat;
int32_t target_lon;
int32_t orig_lon=1006788200,orig_lat=137533700;

int32_t K_X_POS=50;
int32_t K_Y_POS=50;

int8_t cam_motion_x;
int8_t cam_motion_y;

#define MOT_NW_PORT 0
#define MOT_NE_PORT 1
#define MOT_SE_PORT 4
#define MOT_SW_PORT 3

#define CTRL_MODE_DISABLE 0
#define CTRL_MODE_ACRO 1
#define CTRL_MODE_STAB 2
#define CTRL_MODE_ALT 3
#define CTRL_MODE_GPS 4

char ctrl_mode=CTRL_MODE_DISABLE;

uint16_t baro_read_reg_16(uint8_t reg) {
    uint16_t value;
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.requestFrom(MS5611_ADDRESS, 2);
    while(!Wire.available()) {};
    uint8_t vha = Wire.read();
    uint8_t vla = Wire.read();
    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}

uint32_t baro_read_reg_24(uint8_t reg) {
    uint32_t value;
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.requestFrom(MS5611_ADDRESS, 3);
    while(!Wire.available()) {};
    uint8_t vxa = Wire.read();
    uint8_t vha = Wire.read();
    uint8_t vla = Wire.read();
    Wire.endTransmission();

    value = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;

    return value;
}

void read_motion() {
    int32_t diff;
    int i;
    mpu.getMotion6(&cur_accel[0],&cur_accel[1],&cur_accel[2],&cur_gyro[0],&cur_gyro[1],&cur_gyro[2]);
    for (i=0; i<3; i++) {
        diff=((int32_t)cur_accel[i]<<10)-accel_filter[i];
        accel_filter[i]+=(diff+4)>>3;
        smooth_accel[i]=accel_filter[i]>>10;
    }
}

void init_baro() {
    int i;
    for (i=0; i<6; i++) {
        baro_prom[i]=baro_read_reg_16(MS5611_CMD_READ_PROM+i*2);
    }
}

uint32_t num_baro_samples=0;

void read_baro() {
    if (baro_mode==BARO_MODE_P) {
        baro_p=baro_read_reg_24(MS5611_CMD_ADC_READ);
        calc_pressure();
    } else if (baro_mode==BARO_MODE_T) {
        baro_t=baro_read_reg_24(MS5611_CMD_ADC_READ);
    }
    if ((num_baro_samples&1)==0) {
        Wire.beginTransmission(MS5611_ADDRESS);
        Wire.write(MS5611_CMD_CONV_D2+MS5611_ULTRA_HIGH_RES);
        Wire.endTransmission();
        baro_mode=BARO_MODE_T;
    } else {
        Wire.beginTransmission(MS5611_ADDRESS);
        Wire.write(MS5611_CMD_CONV_D1+MS5611_ULTRA_HIGH_RES);
        Wire.endTransmission();
        baro_mode=BARO_MODE_P;
    }
    num_baro_samples++;
}

int calc_pressure() {
    int32_t dt;
    int64_t off,sens; // XXX: sure need 64 bits?
    int32_t p,diff_alt;
    dt=baro_t-((uint32_t)baro_prom[4]<<8);
    off=((int64_t)baro_prom[1]<<16)+(((int64_t)baro_prom[3]*dt)>>7);
    sens=((int64_t)baro_prom[0]<<15)+(((int64_t)baro_prom[2]*dt)>>8);
    p=((((int64_t)baro_p*sens)>>21)-off)>>15;
    if (p>=1000 && p<=120000) {
        baro_pressure=p;
        alt_sens=(101325UL+1000UL-baro_pressure)<<3; // 1Pa ~= 10cm ~= 8cm

        if (!alt_filt) alt_filt=alt_sens<<10;
        alt_pred=alt_filt+((alt_speed_filt*BARO_DT+512)>>10);
        alt_speed_pred=alt_speed_filt;
        diff_alt=(alt_sens<<10)-alt_pred;
        alt_filt=alt_pred+((diff_alt*K_ALT+128)>>8);
        alt_speed_filt=alt_speed_pred+((diff_alt*K_SPEED+128)>>8);
        cur_alt=(alt_filt+512)>>10;
        prev_alt_speed=cur_alt_speed;
        cur_alt_speed=(alt_speed_filt+512)>>10;
    }
}

void calc_position() {
    // 11111111/10000000 = 1.11 ~= 1 => 1 lat/lon unit ~= 1 cm
    int32_t gps_x_pos,x_pos_pred,diff_x_pos;
    int32_t gps_y_pos,y_pos_pred,diff_y_pos;
    int32_t body_x_speed_pred,body_x_accel_pred,diff_body_x_speed;
    int32_t body_y_speed_pred,body_y_accel_pred,diff_body_y_speed;

    gps_x_pos=gps_lon-orig_lon;
    x_pos_pred=x_pos_filt+((gps_east_speed*GPS_DT+512)>>10);
    diff_x_pos=(gps_x_pos<<10)-x_pos_pred;
    x_pos_filt=x_pos_pred+((diff_x_pos*K_X_POS+128)>>8);
    x_pos=(x_pos_filt+512)>>10;

    gps_y_pos=gps_lat-orig_lat;
    y_pos_pred=y_pos_filt+((gps_north_speed*GPS_DT+512)>>10);
    diff_y_pos=(gps_y_pos<<10)-y_pos_pred;
    y_pos_filt=y_pos_pred+((diff_y_pos*K_Y_POS+128)>>8);
    y_pos=(y_pos_filt+512)>>10;

    body_x_speed=((int32_t)gps_north_speed*(int32_t)scale_comp_x+(int32_t)gps_east_speed*(int32_t)scale_comp_y+8192)>>14;
    body_y_speed=((int32_t)gps_north_speed*(int32_t)scale_comp_y-(int32_t)gps_east_speed*(int32_t)scale_comp_x+8192)>>14;

    if (!body_x_speed_filt) body_x_speed_filt=(int32_t)body_x_speed<<10;
    body_x_speed_pred=body_x_speed_filt+((body_x_accel_filt*GPS_DT+512)>>10);
    body_x_accel_pred=body_x_accel_filt;
    diff_body_x_speed=((int32_t)body_x_speed<<10)-body_x_speed_pred;
    body_x_speed_filt=body_x_speed_pred+((diff_body_x_speed*K_BODY_X_SPEED+128)>>8);
    body_x_accel_filt=body_x_accel_pred+((diff_body_x_speed*K_BODY_X_ACCEL+128)>>8);
    body_x_speed_smooth=(body_x_speed_filt+512)>>10;
    body_x_accel=(body_x_accel_filt+512)>>10;

    if (!body_y_speed_filt) body_y_speed_filt=(int32_t)body_y_speed<<10;
    body_y_speed_pred=body_y_speed_filt+((body_y_accel_filt*GPS_DT+512)>>10);
    body_y_accel_pred=body_y_accel_filt;
    diff_body_y_speed=((int32_t)body_y_speed<<10)-body_y_speed_pred;
    body_y_speed_filt=body_y_speed_pred+((diff_body_y_speed*K_BODY_Y_SPEED+128)>>8);
    body_y_accel_filt=body_y_accel_pred+((diff_body_y_speed*K_BODY_Y_ACCEL+128)>>8);
    body_y_speed_smooth=(body_y_speed_filt+512)>>10;
    body_y_accel=(body_y_accel_filt+512)>>10;
}

unsigned isqrt(unsigned long val) {
    unsigned long temp, g=0, b = 0x8000, bshft = 15;
    do {
        printf(".\n");
        if (val >= (temp = (((g << 1) + b)<<bshft--))) {
           g += b;
           val -= temp;
        }
    } while (b >>= 1);
    return g;
}

void read_compass() {
    int32_t s,f;
    compass.getHeading(&comp_x,&comp_y,&comp_z);
    s=(int32_t)comp_x*(int32_t)comp_x+(int32_t)comp_y*(int32_t)comp_y;
    f=((int32_t)1<<24)/isqrt(s); // XXX: slow division
    scale_comp_x=((int32_t)comp_x*f)>>10;
    scale_comp_y=((int32_t)comp_y*f)>>10;
}

void read_volt() {
    uint32_t s;
    s=analogRead(1); // TODO: use interrupt?
    bat_volt=(s*5200+512)>>10;
}

void read_amp() {
    bat_amp=analogRead(0); // TODO: use interrupt?
}

void set_pwm(uint8_t chan, uint16_t dur) {
    if (dur>2000) return;
    if (chan>5) return;
    if (chan==0) {
#ifdef PWM_50HZ
        OCR3A=((uint16_t)20000-dur)*2;
#elif defined PWM_400HZ
        OCR3A=(2500-dur)*2;
#endif
    } else if (chan==1) {
#ifdef PWM_50HZ
        OCR3B=((uint16_t)20000-dur)*2;
#elif defined PWM_400HZ
        OCR3B=(2500-dur)*2;
#endif
    } else if (chan==2) {
#ifdef PWM_50HZ
        OCR3C=((uint16_t)20000-dur)*2;
#elif defined PWM_400HZ
        OCR3C=(2500-dur)*2;
#endif
    } else if (chan==3) {
#ifdef PWM_50HZ
        OCR4A=((uint16_t)20000-dur)*2;
#elif defined PWM_400HZ
        OCR4A=(2500-dur)*2;
#endif
    } else if (chan==4) {
#ifdef PWM_50HZ
        OCR4B=((uint16_t)20000-dur)*2;
#elif defined PWM_400HZ
        OCR4B=(2500-dur)*2;
#endif
    } else if (chan==5) {
#ifdef PWM_50HZ
        OCR4C=((uint16_t)20000-dur)*2;
#elif defined PWM_400HZ
        OCR4C=(2500-dur)*2;
#endif
    }
    pwm_durations[chan]=dur;
}

void do_control() {
    int32_t m_ww,m_wx,m_wy,m_wz,m_xx,m_xy,m_xz,m_yy,m_yz,m_zz;
    int32_t adj_roll_p;
    int32_t adj_pitch_p;
    int32_t adj_yaw_p;
    int32_t adj_alt_p;
    int32_t speed_nw,speed_ne,speed_sw,speed_se;
    uint16_t mot_nw,mot_ne,mot_sw,mot_se;
    int32_t diff;
    int32_t err_x_pos_p,adj_x_pos_p;
    int32_t err_y_pos_p,adj_y_pos_p;

    m_ww=(int32_t)cur_q[0]*(int32_t)cur_q[0];
    m_wx=(int32_t)cur_q[0]*(int32_t)cur_q[1];
    m_wy=(int32_t)cur_q[0]*(int32_t)cur_q[2];
    m_wz=(int32_t)cur_q[0]*(int32_t)cur_q[3];
    m_xx=(int32_t)cur_q[1]*(int32_t)cur_q[1];
    m_xy=(int32_t)cur_q[1]*(int32_t)cur_q[2];
    m_xz=(int32_t)cur_q[1]*(int32_t)cur_q[3];
    m_yy=(int32_t)cur_q[2]*(int32_t)cur_q[2];
    m_yz=(int32_t)cur_q[2]*(int32_t)cur_q[3];
    m_zz=(int32_t)cur_q[3]*(int32_t)cur_q[3];

    cur_rxz=(m_xz-m_wy)>>13;
    cur_ryz=(m_wx+m_yz)>>13;
    cur_rzz=(m_ww-m_xx-m_yy+m_zz)>>14;
    cur_rxx=(m_ww+m_xx-m_yy-m_zz)>>14;
    cur_rxy=(m_wz+m_xy)>>13;

    err_x_pos_p=(target_lon-orig_lon)-x_pos;
    adj_x_pos_p=(err_x_pos_p*(int32_t)ctrl_x_pos_p+128)>>8;
    target_x_speed=adj_x_pos_p;
    if (target_x_speed>ctrl_x_speed_max) target_x_speed=ctrl_x_speed_max;
    else if (target_x_speed<-ctrl_x_speed_max) target_x_speed=-ctrl_x_speed_max;

    err_y_pos_p=(target_lat-orig_lat)-y_pos;
    adj_y_pos_p=(err_y_pos_p*(int32_t)ctrl_y_pos_p+128)>>8;
    target_y_speed=adj_y_pos_p;
    if (target_y_speed>ctrl_y_speed_max) target_y_speed=ctrl_y_speed_max;
    else if (target_y_speed<-ctrl_y_speed_max) target_y_speed=-ctrl_y_speed_max;

    target_body_x_speed=((int32_t)target_y_speed*(int32_t)scale_comp_x+(int32_t)target_x_speed*(int32_t)scale_comp_y+8192)>>14;
    target_body_y_speed=((int32_t)target_y_speed*(int32_t)scale_comp_y-(int32_t)target_x_speed*(int32_t)scale_comp_x+8192)>>14;

    if (ctrl_mode==CTRL_MODE_GPS) {
        err_x_speed_p=target_body_x_speed-body_x_speed;
        err_y_speed_p=target_body_y_speed-body_y_speed;

        err_x_speed_i+=(int32_t)err_x_speed_p;
        if (err_x_speed_i>ctrl_x_speed_imax) err_x_speed_i=ctrl_x_speed_imax;
        else if (err_x_speed_i<-ctrl_x_speed_imax) err_x_speed_i=-ctrl_x_speed_imax;

        err_y_speed_i+=(int32_t)err_y_speed_p;
        if (err_y_speed_i>ctrl_y_speed_imax) err_y_speed_i=ctrl_y_speed_imax;
        else if (err_y_speed_i<-ctrl_y_speed_imax) err_y_speed_i=-ctrl_y_speed_imax;

        adj_x_speed_p=((int32_t)err_x_speed_p*(int32_t)ctrl_x_speed_p+64)>>7;
        adj_y_speed_p=((int32_t)err_y_speed_p*(int32_t)ctrl_y_speed_p+64)>>7;

        adj_x_speed_i=(err_x_speed_i*(int32_t)ctrl_x_speed_i+512)>>10;
        adj_y_speed_i=(err_y_speed_i*(int32_t)ctrl_y_speed_i+512)>>10;

        adj_x_speed_d=-((int32_t)body_x_accel*(int32_t)ctrl_x_speed_d+64)>>7;
        adj_y_speed_d=-((int32_t)body_y_accel*(int32_t)ctrl_y_speed_d+64)>>7;

        target_rxz=target_rxz_base-adj_x_speed_p-adj_x_speed_i-adj_x_speed_d;
        target_ryz=target_ryz_base-adj_y_speed_p-adj_y_speed_i-adj_y_speed_d;
    } else {
        target_rxz=target_rxz_base;
        target_ryz=target_ryz_base;
    }

    err_alt_p=target_alt-cur_alt;
    adj_alt_p=(err_alt_p*(int32_t)ctrl_alt_p+128)>>8;
    target_alt_speed=adj_alt_p;
    if (target_alt_speed>ctrl_alt_speed_max) target_alt_speed=ctrl_alt_speed_max;
    else if (target_alt_speed<-ctrl_alt_speed_max) target_alt_speed=-ctrl_alt_speed_max;

    err_alt_speed_p=target_alt_speed-cur_alt_speed;
    err_alt_speed_d=cur_alt_speed-prev_alt_speed;
    err_alt_speed_i+=(int32_t)err_alt_speed_p;
    if (err_alt_speed_i>ctrl_alt_speed_imax) err_alt_speed_i=ctrl_alt_speed_imax;
    else if (err_alt_speed_i<-ctrl_alt_speed_imax) err_alt_speed_i=-ctrl_alt_speed_imax;

    adj_alt_speed_p=((int32_t)err_alt_speed_p*(int32_t)ctrl_alt_speed_p+128)>>8;
    adj_alt_speed_d=-((int32_t)err_alt_speed_d*(int32_t)ctrl_alt_speed_d+128)>>8;
    adj_alt_speed_i=(err_alt_speed_i*(int32_t)ctrl_alt_speed_i+8192)>>14;
    adj_alt_speed=adj_alt_speed_p+adj_alt_speed_d+adj_alt_speed_i;

    if (ctrl_mode==CTRL_MODE_ALT || ctrl_mode==CTRL_MODE_GPS) {
        throttle=ctrl_throttle_hover+adj_alt_speed;
        if (throttle>ctrl_throttle_max) throttle=ctrl_throttle_max;
        if (throttle<ctrl_throttle_min) throttle=ctrl_throttle_min;
    }

    err_pitch_p=target_rxz-cur_rxz;
    err_roll_p=target_ryz-cur_ryz;
    err_yaw_p=((int32_t)target_rxy*(int32_t)scale_comp_x-(int32_t)target_rxx*(int32_t)scale_comp_y+8192)>>14;
    if ((int32_t)target_rxx*(int32_t)scale_comp_x+(int32_t)target_rxy*(int32_t)scale_comp_y<0) {
        if (err_yaw_p>=0) err_yaw_p=16384;
        else err_yaw_p=-16384;
    }

    adj_pitch_p=((int32_t)err_pitch_p*(int32_t)ctrl_pitch_p+512)>>10;
    adj_roll_p=((int32_t)err_roll_p*(int32_t)ctrl_roll_p+512)>>10;
    adj_yaw_p=((int32_t)err_yaw_p*(int32_t)ctrl_yaw_p+512)>>10;

    target_pitch_speed=adj_pitch_p;
    target_roll_speed=adj_roll_p;
    target_yaw_speed=adj_yaw_p;

    if (target_pitch_speed>ctrl_pitch_speed_max) target_pitch_speed=ctrl_pitch_speed_max;
    else if (target_pitch_speed<-ctrl_pitch_speed_max) target_pitch_speed=-ctrl_pitch_speed_max;
    if (target_roll_speed>ctrl_roll_speed_max) target_roll_speed=ctrl_roll_speed_max;
    else if (target_roll_speed<-ctrl_roll_speed_max) target_roll_speed=-ctrl_roll_speed_max;
    if (target_yaw_speed>ctrl_yaw_speed_max) target_yaw_speed=ctrl_yaw_speed_max;
    else if (target_yaw_speed<-ctrl_yaw_speed_max) target_yaw_speed=-ctrl_yaw_speed_max;

    prev_pitch_speed=pitch_speed;
    prev_roll_speed=roll_speed;
    prev_yaw_speed=yaw_speed;

    pitch_speed=-cur_gyro[1];
    roll_speed=cur_gyro[0];
    yaw_speed=-cur_gyro[2];

    err_pitch_speed_p=target_pitch_speed-pitch_speed;
    err_roll_speed_p=target_roll_speed-roll_speed;
    err_yaw_speed_p=target_yaw_speed-yaw_speed;

    err_pitch_speed_d=pitch_speed-prev_pitch_speed;
    err_roll_speed_d=roll_speed-prev_roll_speed;
    err_yaw_speed_d=yaw_speed-prev_yaw_speed;

    err_pitch_speed_i+=(int32_t)err_pitch_speed_p;
    if (err_pitch_speed_i>ctrl_pitch_speed_imax) err_pitch_speed_i=ctrl_pitch_speed_imax;
    else if (err_pitch_speed_i<-ctrl_pitch_speed_imax) err_pitch_speed_i=-ctrl_pitch_speed_imax;
    err_roll_speed_i+=(int32_t)err_roll_speed_p;
    if (err_roll_speed_i>ctrl_roll_speed_imax) err_roll_speed_i=ctrl_roll_speed_imax;
    else if (err_roll_speed_i<-ctrl_roll_speed_imax) err_roll_speed_i=-ctrl_roll_speed_imax;
    err_yaw_speed_i+=(int32_t)err_yaw_speed_p;
    if (err_yaw_speed_i>ctrl_yaw_speed_imax) err_yaw_speed_i=ctrl_yaw_speed_imax;
    else if (err_yaw_speed_i<-ctrl_yaw_speed_imax) err_yaw_speed_i=-ctrl_yaw_speed_imax;

    if (throttle<300) { // XXX
        err_pitch_speed_i=0;
        err_roll_speed_i=0;
        err_yaw_speed_i=0;
    }

    adj_pitch_speed_p=((int32_t)err_pitch_speed_p*(int32_t)ctrl_pitch_speed_p+512)>>10;
    adj_pitch_speed_d=-((int32_t)err_pitch_speed_d*(int32_t)ctrl_pitch_speed_d+512)>>10;
    adj_pitch_speed_i=(err_pitch_speed_i*(int32_t)ctrl_pitch_speed_i+8192)>>14;
    adj_pitch_speed=adj_pitch_speed_p+adj_pitch_speed_d+adj_pitch_speed_i;

    adj_roll_speed_p=((int32_t)err_roll_speed_p*(int32_t)ctrl_roll_speed_p+512)>>10;
    adj_roll_speed_d=-((int32_t)err_roll_speed_d*(int32_t)ctrl_roll_speed_d+512)>>10;
    adj_roll_speed_i=(err_roll_speed_i*(int32_t)ctrl_roll_speed_i+8192)>>14;
    adj_roll_speed=adj_roll_speed_p+adj_roll_speed_d+adj_roll_speed_i;

    adj_yaw_speed_p=((int32_t)err_yaw_speed_p*(int32_t)ctrl_yaw_speed_p+512)>>10;
    adj_yaw_speed_d=-((int32_t)err_yaw_speed_d*(int32_t)ctrl_yaw_speed_d+512)>>10;
    adj_yaw_speed_i=(err_yaw_speed_i*(int32_t)ctrl_yaw_speed_i+8192)>>14;
    adj_yaw_speed=adj_yaw_speed_p+adj_yaw_speed_d+adj_yaw_speed_i;

    speed_nw=throttle+adj_pitch_speed+adj_roll_speed-adj_yaw_speed;
    speed_ne=throttle+adj_pitch_speed-adj_roll_speed+adj_yaw_speed;
    speed_se=throttle-adj_pitch_speed-adj_roll_speed-adj_yaw_speed;
    speed_sw=throttle-adj_pitch_speed+adj_roll_speed+adj_yaw_speed;
    int16_t speed_limit=throttle*2;
    if (speed_nw>speed_limit) speed_nw=speed_limit;
    if (speed_ne>speed_limit) speed_ne=speed_limit;
    if (speed_se>speed_limit) speed_se=speed_limit;
    if (speed_sw>speed_limit) speed_sw=speed_limit;
    int16_t max_speed;
    max_speed=speed_nw;
    if (speed_ne>max_speed) max_speed=speed_ne;
    if (speed_se>max_speed) max_speed=speed_se;
    if (speed_sw>max_speed) max_speed=speed_sw;
    if (max_speed>1000) {
        int16_t speed_off=max_speed-1000;
        speed_nw-=speed_off;
        speed_ne-=speed_off;
        speed_se-=speed_off;
        speed_sw-=speed_off;
    }
    if (speed_nw<0) speed_nw=0;
    if (speed_ne<0) speed_ne=0;
    if (speed_se<0) speed_se=0;
    if (speed_sw<0) speed_sw=0;

    mot_nw=1000+speed_nw;
    mot_ne=1000+speed_ne;
    mot_se=1000+speed_se;
    mot_sw=1000+speed_sw;

    if (cur_rzz<=0) {
        if (ctrl_mode!=CTRL_MODE_DISABLE) {
            set_throttle(0);
        }
        mot_nw=1000;
        mot_ne=1000;
        mot_se=1000;
        mot_sw=1000;
    }
    if (ctrl_mode!=CTRL_MODE_DISABLE) {
        set_pwm(MOT_NW_PORT,mot_nw);
        set_pwm(MOT_NE_PORT,mot_ne);
        set_pwm(MOT_SE_PORT,mot_se);
        set_pwm(MOT_SW_PORT,mot_sw);
    }
}

void set_control_mode(int m) {
    ctrl_mode=m;
    if (m==CTRL_MODE_ALT || m==CTRL_MODE_GPS) {
        err_alt_speed_i=0; // XXX
        err_x_speed_i=0; // XXX
        err_y_speed_i=0; // XXX
    }
}

int set_throttle(uint16_t t) {
    if (t>1000) return -1;
    ctrl_mode=CTRL_MODE_STAB;
    throttle=t;
    return 0;
}

void dmpDataReady() {
    mpuInterrupt = true;
}

char spi_mode=0;
#define SPI_WAIT_COMMAND_HEAD 0
#define SPI_WAIT_COMMAND_SIZE 1
#define SPI_READ_COMMAND 2
#define SPI_WRITE_RESPONSE 3

uint8_t spi_cmd[128];
uint8_t spi_cmd_size;
uint8_t spi_cmd_read_num;

uint8_t spi_resp[256];
uint8_t spi_resp_size;
uint8_t spi_resp_write_num;

void spi_process_command() {
    uint8_t *c;
    received_cmd=1;
    char cmd=spi_cmd[0];
    if (cmd=='i') {
        c=spi_resp;
        *c++=ctrl_mode;
        *c++=throttle>>8;
        *c++=throttle&0xff;
        *c++=pwm_durations[MOT_NW_PORT]>>8;
        *c++=pwm_durations[MOT_NW_PORT]&0xff;
        *c++=pwm_durations[MOT_NE_PORT]>>8;
        *c++=pwm_durations[MOT_NE_PORT]&0xff;
        *c++=pwm_durations[MOT_SE_PORT]>>8;
        *c++=pwm_durations[MOT_SE_PORT]&0xff;
        *c++=pwm_durations[MOT_SW_PORT]>>8;
        *c++=pwm_durations[MOT_SW_PORT]&0xff;
        *c++=cur_accel[0]>>8;
        *c++=cur_accel[0]&0xff;
        *c++=cur_accel[1]>>8;
        *c++=cur_accel[1]&0xff;
        *c++=cur_accel[2]>>8;
        *c++=cur_accel[2]&0xff;
        *c++=cur_gyro[0]>>8;
        *c++=cur_gyro[0]&0xff;
        *c++=cur_gyro[1]>>8;
        *c++=cur_gyro[1]&0xff;
        *c++=cur_gyro[2]>>8;
        *c++=cur_gyro[2]&0xff;
        *c++=cur_rxz>>8;
        *c++=cur_rxz&0xff;
        *c++=cur_ryz>>8;
        *c++=cur_ryz&0xff;
        *c++=scale_comp_x>>8;
        *c++=scale_comp_x&0xff;
        *c++=scale_comp_y>>8;
        *c++=scale_comp_y&0xff;
        *c++=target_rxz>>8;
        *c++=target_rxz&0xff;
        *c++=target_ryz>>8;
        *c++=target_ryz&0xff;
        *c++=target_rxx>>8;
        *c++=target_rxx&0xff;
        *c++=target_rxy>>8;
        *c++=target_rxy&0xff;
        *c++=target_pitch_speed>>8;
        *c++=target_pitch_speed&0xff;
        *c++=target_roll_speed>>8;
        *c++=target_roll_speed&0xff;
        *c++=target_yaw_speed>>8;
        *c++=target_yaw_speed&0xff;
        *c++=err_pitch_speed_d>>8;
        *c++=err_pitch_speed_d&0xff;
        *c++=err_roll_speed_d>>8;
        *c++=err_roll_speed_d&0xff;
        *c++=err_yaw_speed_d>>8;
        *c++=err_yaw_speed_d&0xff;
        *c++=err_pitch_speed_i>>24;
        *c++=(err_pitch_speed_i>>16)&0xff;
        *c++=(err_pitch_speed_i>>8)&0xff;
        *c++=err_pitch_speed_i&0xff;
        *c++=err_roll_speed_i>>24;
        *c++=(err_roll_speed_i>>16)&0xff;
        *c++=(err_roll_speed_i>>8)&0xff;
        *c++=err_roll_speed_i&0xff;
        *c++=err_yaw_speed_i>>24;
        *c++=(err_yaw_speed_i>>16)&0xff;
        *c++=(err_yaw_speed_i>>8)&0xff;
        *c++=err_yaw_speed_i&0xff;
        *c++=cur_alt>>24;
        *c++=(cur_alt>>16)&0xff;
        *c++=(cur_alt>>8)&0xff;
        *c++=cur_alt&0xff;
        *c++=cur_alt_speed>>8;
        *c++=cur_alt_speed&0xff;
        *c++=bat_volt>>8;
        *c++=bat_volt&0xff;
        *c++=bat_amp>>8;
        *c++=bat_amp&0xff;
        *c++=smooth_accel[2]>>8;
        *c++=smooth_accel[2]&0xff;
        *c++=target_alt>>24;
        *c++=(target_alt>>16)&0xff;
        *c++=(target_alt>>8)&0xff;
        *c++=target_alt&0xff;
        *c++=target_alt_speed>>8;
        *c++=target_alt_speed&0xff;
        *c++=err_alt_speed_d>>8;
        *c++=err_alt_speed_d&0xff;
        *c++=err_alt_speed_i>>24;
        *c++=(err_alt_speed_i>>16)&0xff;
        *c++=(err_alt_speed_i>>8)&0xff;
        *c++=err_alt_speed_i&0xff;
        *c++=gps_num_sat;
        *c++=gps_north_speed>>8;
        *c++=gps_north_speed&0xff;
        *c++=gps_east_speed>>8;
        *c++=gps_east_speed&0xff;
        *c++=body_x_speed>>8;
        *c++=body_x_speed&0xff;
        *c++=body_y_speed>>8;
        *c++=body_y_speed&0xff;
        *c++=ctrl_throttle_hover>>8;
        *c++=ctrl_throttle_hover&0xff;
        *c++=adj_alt_speed_p>>8;
        *c++=adj_alt_speed_p&0xff;
        *c++=adj_alt_speed_d>>8;
        *c++=adj_alt_speed_d&0xff;
        *c++=adj_alt_speed_i>>8;
        *c++=adj_alt_speed_i&0xff;
        *c++=gps_lon>>24;
        *c++=(gps_lon>>16)&0xff;
        *c++=(gps_lon>>8)&0xff;
        *c++=gps_lon&0xff;
        *c++=gps_lat>>24;
        *c++=(gps_lat>>16)&0xff;
        *c++=(gps_lat>>8)&0xff;
        *c++=gps_lat&0xff;
        *c++=(gps_hacc>>8)&0xff;
        *c++=gps_hacc&0xff;
        *c++=(gps_vacc>>8)&0xff;
        *c++=gps_vacc&0xff;
        *c++=target_lon>>24;
        *c++=(target_lon>>16)&0xff;
        *c++=(target_lon>>8)&0xff;
        *c++=target_lon&0xff;
        *c++=target_lat>>24;
        *c++=(target_lat>>16)&0xff;
        *c++=(target_lat>>8)&0xff;
        *c++=target_lat&0xff;
        *c++=target_x_speed>>8;
        *c++=target_x_speed&0xff;
        *c++=target_y_speed>>8;
        *c++=target_y_speed&0xff;
        *c++=target_body_x_speed>>8;
        *c++=target_body_x_speed&0xff;
        *c++=target_body_y_speed>>8;
        *c++=target_body_y_speed&0xff;
        *c++=err_x_speed_i>>24;
        *c++=(err_x_speed_i>>16)&0xff;
        *c++=(err_x_speed_i>>8)&0xff;
        *c++=err_x_speed_i&0xff;
        *c++=err_y_speed_i>>24;
        *c++=(err_y_speed_i>>16)&0xff;
        *c++=(err_y_speed_i>>8)&0xff;
        *c++=err_y_speed_i&0xff;
        *c++=adj_pitch_speed_p>>8;
        *c++=adj_pitch_speed_p&0xff;
        *c++=adj_roll_speed_p>>8;
        *c++=adj_roll_speed_p&0xff;
        *c++=adj_yaw_speed_p>>8;
        *c++=adj_yaw_speed_p&0xff;
        *c++=adj_pitch_speed_i>>8;
        *c++=adj_pitch_speed_i&0xff;
        *c++=adj_roll_speed_i>>8;
        *c++=adj_roll_speed_i&0xff;
        *c++=adj_yaw_speed_i>>8;
        *c++=adj_yaw_speed_i&0xff;
        *c++=adj_x_speed_p>>8;
        *c++=adj_x_speed_p&0xff;
        *c++=adj_x_speed_i>>8;
        *c++=adj_x_speed_i&0xff;
        *c++=adj_x_speed_d>>8;
        *c++=adj_x_speed_d&0xff;
        *c++=adj_y_speed_p>>8;
        *c++=adj_y_speed_p&0xff;
        *c++=adj_y_speed_i>>8;
        *c++=adj_y_speed_i&0xff;
        *c++=adj_y_speed_d>>8;
        *c++=adj_y_speed_d&0xff;
        *c++=body_x_accel>>8;
        *c++=body_x_accel&0xff;
        *c++=body_y_accel>>8;
        *c++=body_y_accel&0xff;
        *c++=body_x_speed_smooth>>8;
        *c++=body_x_speed_smooth&0xff;
        *c++=body_y_speed_smooth>>8;
        *c++=body_y_speed_smooth&0xff;
        spi_resp_size=c-spi_resp;
    } else if (cmd=='c') {
        c=spi_resp;
        *c++=ctrl_pitch_speed_p>>8;
        *c++=ctrl_pitch_speed_p&0xff;
        *c++=ctrl_pitch_speed_d>>8;
        *c++=ctrl_pitch_speed_d&0xff;
        *c++=ctrl_pitch_speed_i>>8;
        *c++=ctrl_pitch_speed_i&0xff;
        *c++=ctrl_pitch_speed_imax>>24;
        *c++=(ctrl_pitch_speed_imax>>16)&0xff;
        *c++=(ctrl_pitch_speed_imax>>8)&0xff;
        *c++=ctrl_pitch_speed_imax&0xff;
        *c++=ctrl_roll_speed_p>>8;
        *c++=ctrl_roll_speed_p&0xff;
        *c++=ctrl_roll_speed_d>>8;
        *c++=ctrl_roll_speed_d&0xff;
        *c++=ctrl_roll_speed_i>>8;
        *c++=ctrl_roll_speed_i&0xff;
        *c++=ctrl_roll_speed_imax>>24;
        *c++=(ctrl_roll_speed_imax>>16)&0xff;
        *c++=(ctrl_roll_speed_imax>>8)&0xff;
        *c++=ctrl_roll_speed_imax&0xff;
        *c++=ctrl_yaw_speed_p>>8;
        *c++=ctrl_yaw_speed_p&0xff;
        *c++=ctrl_yaw_speed_d>>8;
        *c++=ctrl_yaw_speed_d&0xff;
        *c++=ctrl_yaw_speed_i>>8;
        *c++=ctrl_yaw_speed_i&0xff;
        *c++=ctrl_yaw_speed_imax>>24;
        *c++=(ctrl_yaw_speed_imax>>16)&0xff;
        *c++=(ctrl_yaw_speed_imax>>8)&0xff;
        *c++=ctrl_yaw_speed_imax&0xff;
        *c++=ctrl_pitch_p>>8;
        *c++=ctrl_pitch_p&0xff;
        *c++=ctrl_roll_p>>8;
        *c++=ctrl_roll_p&0xff;
        *c++=ctrl_yaw_p>>8;
        *c++=ctrl_yaw_p&0xff;
        *c++=ctrl_pitch_speed_max>>8;
        *c++=ctrl_pitch_speed_max&0xff;
        *c++=ctrl_roll_speed_max>>8;
        *c++=ctrl_roll_speed_max&0xff;
        *c++=ctrl_yaw_speed_max>>8;
        *c++=ctrl_yaw_speed_max&0xff;
        *c++=ctrl_alt_p>>8;
        *c++=ctrl_alt_p&0xff;
        *c++=ctrl_alt_speed_max>>8;
        *c++=ctrl_alt_speed_max&0xff;
        *c++=ctrl_alt_speed_p>>8;
        *c++=ctrl_alt_speed_p&0xff;
        *c++=ctrl_alt_speed_d>>8;
        *c++=ctrl_alt_speed_d&0xff;
        *c++=ctrl_alt_speed_i>>8;
        *c++=ctrl_alt_speed_i&0xff;
        *c++=ctrl_alt_speed_imax>>24;
        *c++=(ctrl_alt_speed_imax>>16)&0xff;
        *c++=(ctrl_alt_speed_imax>>8)&0xff;
        *c++=ctrl_alt_speed_imax&0xff;
        *c++=ctrl_throttle_min>>8;
        *c++=ctrl_throttle_min&0xff;
        *c++=ctrl_throttle_max>>8;
        *c++=ctrl_throttle_max&0xff;
        *c++=ctrl_throttle_hover>>8;
        *c++=ctrl_throttle_hover&0xff;
        *c++=ctrl_x_speed_p>>8;
        *c++=ctrl_x_speed_p&0xff;
        *c++=ctrl_y_speed_p>>8;
        *c++=ctrl_y_speed_p&0xff;
        *c++=ctrl_x_speed_i>>8;
        *c++=ctrl_x_speed_i&0xff;
        *c++=ctrl_y_speed_i>>8;
        *c++=ctrl_y_speed_i&0xff;
        *c++=ctrl_x_speed_imax>>24;
        *c++=(ctrl_x_speed_imax>>16)&0xff;
        *c++=(ctrl_x_speed_imax>>8)&0xff;
        *c++=ctrl_x_speed_imax&0xff;
        *c++=ctrl_y_speed_imax>>24;
        *c++=(ctrl_y_speed_imax>>16)&0xff;
        *c++=(ctrl_y_speed_imax>>8)&0xff;
        *c++=ctrl_y_speed_imax&0xff;
        *c++=ctrl_x_pos_p>>8;
        *c++=ctrl_x_pos_p&0xff;
        *c++=ctrl_y_pos_p>>8;
        *c++=ctrl_y_pos_p&0xff;
        *c++=ctrl_x_speed_max>>8;
        *c++=ctrl_x_speed_max&0xff;
        *c++=ctrl_y_speed_max>>8;
        *c++=ctrl_y_speed_max&0xff;
        *c++=ctrl_x_speed_d>>8;
        *c++=ctrl_x_speed_d&0xff;
        *c++=ctrl_y_speed_d>>8;
        *c++=ctrl_y_speed_d&0xff;
        spi_resp_size=c-spi_resp;
    } else if (cmd=='s') {
        c=spi_resp;
        *c++=mpu_num_resets>>24;
        *c++=(mpu_num_resets>>16)&0xff;
        *c++=(mpu_num_resets>>8)&0xff;
        *c++=mpu_num_resets&0xff;
        *c++=num_baro_samples>>24;
        *c++=(num_baro_samples>>16)&0xff;
        *c++=(num_baro_samples>>8)&0xff;
        *c++=num_baro_samples&0xff;
        spi_resp_size=c-spi_resp;
    } else if (cmd=='m') {
        c=spi_resp;
        *c++=comp_x>>8;
        *c++=comp_x&0xff;
        *c++=comp_y>>8;
        *c++=comp_y&0xff;
        *c++=comp_z>>8;
        *c++=comp_z&0xff;
        spi_resp_size=c-spi_resp;
    } else if (cmd=='D') {
        int mode=spi_cmd[1];
        set_control_mode(mode);
        spi_resp_size=0;
    } else if (cmd=='M') {
        uint8_t channel=spi_cmd[1];
        uint16_t duration=(spi_cmd[2]<<8)|spi_cmd[3];
        ctrl_mode=CTRL_MODE_DISABLE;
        set_pwm(channel,duration);
        spi_resp_size=0;
    } else if (cmd=='P') {
        ctrl_mode=CTRL_MODE_DISABLE;
        uint16_t pwm1=(spi_cmd[1]<<8)|spi_cmd[2];
        uint16_t pwm2=(spi_cmd[3]<<8)|spi_cmd[4];
        uint16_t pwm3=(spi_cmd[5]<<8)|spi_cmd[6];
        uint16_t pwm4=(spi_cmd[7]<<8)|spi_cmd[8];
        uint16_t pwm5=(spi_cmd[9]<<8)|spi_cmd[10];
        uint16_t pwm6=(spi_cmd[11]<<8)|spi_cmd[12];
        set_pwm(0,pwm1);
        set_pwm(1,pwm2);
        set_pwm(2,pwm3);
        set_pwm(3,pwm4);
        set_pwm(4,pwm5);
        set_pwm(5,pwm6);
        spi_resp_size=0;
    } else if (cmd=='O') {
        mpu_off[0]=(spi_cmd[1]<<8)|spi_cmd[2];
        mpu_off[1]=(spi_cmd[3]<<8)|spi_cmd[4];
        mpu_off[2]=(spi_cmd[5]<<8)|spi_cmd[6];
        mpu_off[3]=(spi_cmd[7]<<8)|spi_cmd[8];
        mpu_off[4]=(spi_cmd[9]<<8)|spi_cmd[10];
        mpu_off[5]=(spi_cmd[11]<<8)|spi_cmd[12];
        do_update_mpu_off=1;
        spi_resp_size=0;
    } else if (cmd=='C') {
        ctrl_pitch_speed_p=(spi_cmd[1]<<8)|spi_cmd[2];
        ctrl_pitch_speed_d=(spi_cmd[3]<<8)|spi_cmd[4];
        ctrl_pitch_speed_i=(spi_cmd[5]<<8)|spi_cmd[6];
        ctrl_pitch_speed_imax=((int32_t)spi_cmd[7]<<24)|((int32_t)spi_cmd[8]<<16)|((int32_t)spi_cmd[9]<<8)|(int32_t)spi_cmd[10];
        ctrl_roll_speed_p=(spi_cmd[11]<<8)|spi_cmd[12];
        ctrl_roll_speed_d=(spi_cmd[13]<<8)|spi_cmd[14];
        ctrl_roll_speed_i=(spi_cmd[15]<<8)|spi_cmd[16];
        ctrl_roll_speed_imax=((int32_t)spi_cmd[17]<<24)|((int32_t)spi_cmd[18]<<16)|((int32_t)spi_cmd[19]<<8)|(int32_t)spi_cmd[20];
        ctrl_yaw_speed_p=(spi_cmd[21]<<8)|spi_cmd[22];
        ctrl_yaw_speed_d=(spi_cmd[23]<<8)|spi_cmd[24];
        ctrl_yaw_speed_i=(spi_cmd[25]<<8)|spi_cmd[26];
        ctrl_yaw_speed_imax=((int32_t)spi_cmd[27]<<24)|((int32_t)spi_cmd[28]<<16)|((int32_t)spi_cmd[29]<<8)|(int32_t)spi_cmd[30];
        ctrl_pitch_p=(spi_cmd[31]<<8)|spi_cmd[32];
        ctrl_roll_p=(spi_cmd[33]<<8)|spi_cmd[34];
        ctrl_yaw_p=(spi_cmd[35]<<8)|spi_cmd[36];
        ctrl_pitch_speed_max=(spi_cmd[37]<<8)|spi_cmd[38];
        ctrl_roll_speed_max=(spi_cmd[39]<<8)|spi_cmd[40];
        ctrl_yaw_speed_max=(spi_cmd[41]<<8)|spi_cmd[42];
        ctrl_alt_p=(spi_cmd[43]<<8)|spi_cmd[44];
        ctrl_alt_speed_max=(spi_cmd[45]<<8)|spi_cmd[46];
        ctrl_alt_speed_p=(spi_cmd[47]<<8)|spi_cmd[48];
        ctrl_alt_speed_d=(spi_cmd[49]<<8)|spi_cmd[50];
        ctrl_alt_speed_i=(spi_cmd[51]<<8)|spi_cmd[52];
        ctrl_alt_speed_imax=((int32_t)spi_cmd[53]<<24)|((int32_t)spi_cmd[54]<<16)|((int32_t)spi_cmd[55]<<8)|(int32_t)spi_cmd[56];
        ctrl_throttle_min=(spi_cmd[57]<<8)|spi_cmd[58];
        ctrl_throttle_max=(spi_cmd[59]<<8)|spi_cmd[60];
        ctrl_throttle_hover=(spi_cmd[61]<<8)|spi_cmd[62];
        ctrl_x_speed_p=(spi_cmd[63]<<8)|spi_cmd[64];
        ctrl_y_speed_p=(spi_cmd[65]<<8)|spi_cmd[66];
        ctrl_x_speed_i=(spi_cmd[67]<<8)|spi_cmd[68];
        ctrl_y_speed_i=(spi_cmd[69]<<8)|spi_cmd[70];
        ctrl_x_speed_imax=((int32_t)spi_cmd[71]<<24)|((int32_t)spi_cmd[72]<<16)|((int32_t)spi_cmd[73]<<8)|(int32_t)spi_cmd[74];
        ctrl_y_speed_imax=((int32_t)spi_cmd[75]<<24)|((int32_t)spi_cmd[76]<<16)|((int32_t)spi_cmd[77]<<8)|(int32_t)spi_cmd[78];
        ctrl_x_pos_p=(spi_cmd[79]<<8)|spi_cmd[80];
        ctrl_y_pos_p=(spi_cmd[81]<<8)|spi_cmd[82];
        ctrl_x_speed_max=(spi_cmd[83]<<8)|spi_cmd[84];
        ctrl_y_speed_max=(spi_cmd[85]<<8)|spi_cmd[86];
        ctrl_x_speed_d=(spi_cmd[87]<<8)|spi_cmd[88];
        ctrl_y_speed_d=(spi_cmd[89]<<8)|spi_cmd[90];
        spi_resp_size=0;
    } else if (cmd=='T') {
        uint16_t t=(spi_cmd[1]<<8)|spi_cmd[2];
        set_throttle(t);
        spi_resp_size=0;
    } else if (cmd=='A') {
        target_rxz_base=(spi_cmd[1]<<8)|spi_cmd[2];
        target_ryz_base=(spi_cmd[3]<<8)|spi_cmd[4];
        target_rxx=(spi_cmd[5]<<8)|spi_cmd[6];
        target_rxy=(spi_cmd[7]<<8)|spi_cmd[8];
        spi_resp_size=0;
    } else if (cmd=='R') {
        target_pitch_speed=(spi_cmd[1]<<8)|spi_cmd[2];
        target_roll_speed=(spi_cmd[3]<<8)|spi_cmd[4];
        target_yaw_speed=(spi_cmd[5]<<8)|spi_cmd[6];
        spi_resp_size=0;
    } else if (cmd=='B') {
        target_alt=((int32_t)spi_cmd[1]<<24)|((int32_t)spi_cmd[2]<<16)|((int32_t)spi_cmd[3]<<8)|(int32_t)spi_cmd[4];
        spi_resp_size=0;
    } else if (cmd=='P') {
        target_lon=((int32_t)spi_cmd[1]<<24)|((int32_t)spi_cmd[2]<<16)|((int32_t)spi_cmd[3]<<8)|(int32_t)spi_cmd[4];
        target_lat=((int32_t)spi_cmd[5]<<24)|((int32_t)spi_cmd[6]<<16)|((int32_t)spi_cmd[7]<<8)|(int32_t)spi_cmd[8];
        spi_resp_size=0;
    } else {
        spi_resp_size=0;
    }
}

void update_mpu_off() {
    mpu.setXAccelOffset(mpu_off[0]);
    mpu.setYAccelOffset(mpu_off[1]);
    mpu.setZAccelOffset(mpu_off[2]);
    mpu.setXGyroOffset(mpu_off[3]);
    mpu.setYGyroOffset(mpu_off[4]);
    mpu.setZGyroOffset(mpu_off[5]);
}

// UBLOX GPS ///////////////////////////////////////////////////////////////////////

struct ubx_header {
  uint8_t preamble1;
  uint8_t preamble2;
  uint8_t msg_class;
  uint8_t msg_id;
  uint16_t length;
};

struct ubx_nav_posllh {
  uint32_t time;  // GPS msToW
  int32_t longitude;
  int32_t latitude;
  int32_t altitude_ellipsoid;
  int32_t altitude_msl;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
};

struct ubx_nav_solution {
  uint32_t time;
  int32_t time_nsec;
  int16_t week;
  uint8_t fix_type;
  uint8_t fix_status;
  int32_t ecef_x;
  int32_t ecef_y;
  int32_t ecef_z;
  uint32_t position_accuracy_3d;
  int32_t ecef_x_velocity;
  int32_t ecef_y_velocity;
  int32_t ecef_z_velocity;
  uint32_t speed_accuracy;
  uint16_t position_DOP;
  uint8_t res;
  uint8_t satellites;
  uint32_t res2;
};

struct ubx_nav_velned {
  uint32_t time;  // GPS msToW
  int32_t ned_north;
  int32_t ned_east;
  int32_t ned_down;
  uint32_t speed_3d;
  uint32_t speed_2d;
  int32_t heading_2d;
  uint32_t speed_accuracy;
  uint32_t heading_accuracy;
};

enum ubs_protocol_bytes {
  PREAMBLE1 = 0xb5,
  PREAMBLE2 = 0x62,
  CLASS_NAV = 0x01,
  CLASS_ACK = 0x05,
  CLASS_CFG = 0x06,
  MSG_ACK_NACK = 0x00,
  MSG_ACK_ACK = 0x01,
  MSG_POSLLH = 0x2,
  MSG_STATUS = 0x3,
  MSG_SOL = 0x6,
  MSG_VELNED = 0x12,
  MSG_CFG_PRT = 0x00,
  MSG_CFG_RATE = 0x08,
  MSG_CFG_SET_RATE = 0x01,
  MSG_CFG_NAV_SETTINGS = 0x24
};

enum ubs_nav_fix_type {
  FIX_NONE = 0,
  FIX_DEAD_RECKONING = 1,
  FIX_2D = 2,
  FIX_3D = 3,
  FIX_GPS_DEAD_RECKONING = 4,
  FIX_TIME = 5
};

enum ubx_nav_status_bits {
  NAV_STATUS_FIX_VALID = 1
};

// Receive buffer
static union {
  ubx_nav_posllh posllh;
  ubx_nav_solution solution;
  ubx_nav_velned velned;
  uint8_t bytes[];
 } _buffer;

bool gps_read_byte(uint8_t data){
  static uint8_t  _step = 0; // State machine state
  static uint8_t  _msg_id;
  static uint16_t _payload_length;
  static uint16_t _payload_counter;
  static uint8_t  _ck_a; // Packet checksum accumulators
  static uint8_t  _ck_b;

  uint8_t st  = _step+1;
  bool    ret = false;

  if (st == 2)
    if (PREAMBLE2 != data) st--; // in case of faillure of the 2nd header byte, still test the first byte
  if (st == 1) {
    if(PREAMBLE1 != data) st--;
  } else if (st == 3) { // CLASS byte, not used, assume it is CLASS_NAV
    _ck_b = _ck_a = data;  // reset the checksum accumulators
  } else if (st > 3 && st < 8) {
    _ck_b += (_ck_a += data);  // checksum byte
    if (st == 4) {
      _msg_id = data;
    } else if (st == 5) {
      _payload_length = data; // payload length low byte
    } else if (st == 6) {
      _payload_length += (uint16_t)(data<<8);
      if (_payload_length > 512) st = 0;
      _payload_counter = 0;  // prepare to receive payload
    } else {
      if (_payload_counter+1 < _payload_length) st--; // stay in the same state while data inside the frame
      if (_payload_counter < sizeof(_buffer)) _buffer.bytes[_payload_counter] = data;
      _payload_counter++;
    }
  } else if (st == 8) {
    if (_ck_a != data) st = 0;  // bad checksum
  } else if (st == 9) {
    st = 0;
    if (_ck_b == data) { // good checksum
      if (_msg_id == MSG_POSLLH) {
        if (gps_enable) {
          gps_lon = _buffer.posllh.longitude;
          gps_lat = _buffer.posllh.latitude;
          gps_alt = _buffer.posllh.altitude_msl;
          gps_hacc = _buffer.posllh.horizontal_accuracy;
          gps_vacc = _buffer.posllh.vertical_accuracy;
        }
        ret= true;        // POSLLH message received, allow blink GUI icon and LED, frame available for nav computation
      } else if (_msg_id ==  MSG_SOL) {
        gps_enable=0;
        if((_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D || _buffer.solution.fix_type == FIX_2D)) gps_enable=1;
        gps_num_sat = _buffer.solution.satellites;
      } else if (_msg_id ==  MSG_VELNED) {
        gps_north_speed         = _buffer.velned.ned_north;  // cm/s
        gps_east_speed         = _buffer.velned.ned_east;  // cm/s
        gps_down_speed         = _buffer.velned.ned_down;  // cm/s
        calc_position();
      }
    }
  }
  _step = st;
  return ret;
}

ISR (SPI_STC_vect)
{
    char c=SPDR;
    if (spi_mode==SPI_WAIT_COMMAND_HEAD) {
        SPDR=1;
        if (c=='C') {
            spi_mode=SPI_WAIT_COMMAND_SIZE;
        }
    } else if (spi_mode==SPI_WAIT_COMMAND_SIZE) {
        SPDR=2;
        spi_cmd_size=c;
        spi_cmd_read_num=0;
        spi_mode=SPI_READ_COMMAND;
    } else if (spi_mode==SPI_READ_COMMAND) {
        spi_cmd[spi_cmd_read_num++]=c;
        if (spi_cmd_read_num>=spi_cmd_size) {
            spi_process_command();
            if (spi_resp_size==0) {
                SPDR=0;
                spi_mode=SPI_WAIT_COMMAND_HEAD;
            } else {
                spi_resp_write_num=0;
                SPDR=spi_resp[spi_resp_write_num++];
                if (spi_resp_write_num>=spi_resp_size) {
                    spi_mode=SPI_WAIT_COMMAND_HEAD;
                }
                spi_mode=SPI_WRITE_RESPONSE;
            }
        } else {
            SPDR=3;
        }
    } else if (spi_mode==SPI_WRITE_RESPONSE) {
        SPDR=spi_resp[spi_resp_write_num++];
        if (spi_resp_write_num>=spi_resp_size) {
            spi_mode=SPI_WAIT_COMMAND_HEAD;
        }
    }
}

void serialEvent3() {
    while (Serial3.available()) {
        char c=(char)Serial3.read();
        gps_read_byte(c);
    }
}

static const uint8_t UBLOX_INIT[] = {
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19,           // VGS: Course over ground and Ground speed
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,           // GSV: GNSS Satellites in View
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,           // GLL: Latitude and longitude, with time of position fix and status
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,           // GGA: Global positioning system fix data
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,           // GSA: GNSS DOP and Active Satellites
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,           // RMC: Recommended Minimum data
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,           // set POSLLH MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,           // set STATUS MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,           // set SOL MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,           // set VELNED MSG rate
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A,             // set rate to 5Hz
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0x01, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00,             // CFG-NAV5 update start
    0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01,
    0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x2D, //  CFG-NAV5 update end
    0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0xE5, // enable SBAS auto
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, // pedestrian dynmodel // XXX: duplicate with CFG_NAV5 above...
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, // pedestrian dynmodel
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, // pedestrian dynmodel
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x82, // pedestrian dynmodel
};


uint32_t gps_init_speed[5] = {9600,19200,38400,57600,115200};

void init_gps() {
    int i;
    for (i=0; i<5; i++) {
        Serial3.begin(gps_init_speed[i]);
        delay(100);
        Serial3.write("$PUBX,41,1,0003,0001,38400,0*26\r\n"); // set 38400 baud
        Serial3.flush();
        delay(100);
    }
    delay(200);
    Serial3.begin(38400);
    for(i=0; i<sizeof(UBLOX_INIT); i++) {                        // send configuration data in UBX protocol
        Serial3.write(UBLOX_INIT[i]);
        delay(5); //simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
    }
    Serial3.flush();
}

void abort() {
    int i;
    ctrl_mode=CTRL_MODE_DISABLE;
    for (i=0; i<6; i++) {
        set_pwm(i,1000);
    }
}

void setup()
{
    int i;
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZ_PIN, OUTPUT);
    pinMode(MISO,OUTPUT);
    pinMode(PWM1_PIN,OUTPUT);
    pinMode(PWM2_PIN,OUTPUT);
    pinMode(PWM3_PIN,OUTPUT);
    pinMode(PWM4_PIN,OUTPUT);
    pinMode(PWM5_PIN,OUTPUT);
    pinMode(PWM6_PIN,OUTPUT);

    // timer3: fast pwm, inverted mode, /8 prescaler
    TCCR3A=(1<<WGM31)|(1<<COM3A1)|(1<<COM3A0)|(1<<COM3B1)|(1<<COM3B0)|(1<<COM3C1)|(1<<COM3C0);
    TCCR3B=(1<<WGM33)|(1<<WGM32)|(1<<CS31);
#ifdef PWM_50HZ
    ICR3=(uint16_t)20000*2-1; // 50Hz PWM
#elif defined PWM_400HZ
    ICR3=2500*2-1; // 400Hz PWM
#endif

    // timer4: fast pwm, inverted mode, /8 prescaler
    TCCR4A=(1<<WGM41)|(1<<COM4A1)|(1<<COM4A0)|(1<<COM4B1)|(1<<COM4B0)|(1<<COM4C1)|(1<<COM4C0);
    TCCR4B=(1<<WGM43)|(1<<WGM42)|(1<<CS41);
#ifdef PWM_50HZ
    ICR4=(uint16_t)20000*2-1; // 50Hz PWM
#elif defined PWM_400HZ
    ICR4=2500*2-1; // 400Hz PWM
#endif

    for (i=0; i<6; i++) {
        //set_pwm(i,1000);
        set_pwm(i,1500); // XXX
    }

     // turn on SPI in slave mode
    SPCR |= _BV(SPE);

    Wire.begin();
    TWBR = 24; // 400kHz I2C clock 

    delay(1000);

    mpu.initialize();
    mpu.dmpInitialize();

    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpu.setI2CBypassEnabled(true);

    mpu_off[0]=mpu.getXAccelOffset();
    mpu_off[1]=mpu.getYAccelOffset();
    mpu_off[2]=mpu.getZAccelOffset();
    mpu_off[3]=mpu.getXGyroOffset();
    mpu_off[4]=mpu.getYGyroOffset();
    mpu_off[5]=mpu.getZGyroOffset();

    delay(1000);

    init_baro();

    compass.initialize();

    SPI.attachInterrupt();

    init_gps();

    for (i=0; i<3; i++) {
        digitalWrite(LED_PIN,HIGH);
        delay(100);
        digitalWrite(LED_PIN,LOW);
        delay(900);
    }
}

void loop()
{
    int i;
    int32_t t=millis();
    if (last_check_t && t-last_check_t>=5000) {
        if (!received_cmd) {
            ctrl_mode=CTRL_MODE_DISABLE;
            for (i=0; i<6; i++) {
                set_pwm(i,1000);
            }
        }
        received_cmd=0;
        last_check_t=t;
    }
    if (t-last_motion_t>=10) {
        read_motion();
        last_motion_t=t;
        do_control();
    }
    if (mpuInterrupt || mpu_fifo_count>=MPU_PKT_SIZE) {
        int stat;
        uint8_t buf[MPU_PKT_SIZE];
        stat=mpu.getIntStatus();
        mpu_fifo_count=mpu.getFIFOCount();
        if ((stat & 0x10) || mpu_fifo_count == 1024) {
            mpu.resetFIFO();
            mpu_num_resets++;
        } else if (stat & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (mpu_fifo_count < MPU_PKT_SIZE) mpu_fifo_count = mpu.getFIFOCount();

            // read a packet from FIFO
            mpu.getFIFOBytes(buf, MPU_PKT_SIZE);
            mpu_fifo_count-=MPU_PKT_SIZE;

            mpu.dmpGetQuaternion(cur_q, buf);
            //mpu.dmpGetAccel(cur_accel, buf); // doesn't work to read from packet?
            //mpu.dmpGetGyro(cur_gyro, buf);
            mpu_num_samples++;
        }
        mpuInterrupt=false;
    }
    if (t-last_baro_t>=20) {
        read_baro();
        last_baro_t=t;
    }
    if (t-last_compass_t>=100) {
        read_compass();
        last_compass_t=t;
    }
    if (t-last_bat_t>=5000) {
        read_volt();
        read_amp();
        last_bat_t=t;
    }
    if (do_update_mpu_off) {
        update_mpu_off();
        do_update_mpu_off=0;
    }
}
