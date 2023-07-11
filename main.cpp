#include "mbed.h"

I2C i2c(P0_0,P0_1);         //SDA,SCL oled
SPI spi(P0_9,P0_8,P0_7);    //DAC
DigitalOut cs1(P0_6);       //DAC cs
DigitalOut cs2(P0_5);

//CH1
AnalogIn a0(P0_23);     //VM1
AnalogIn a1(P0_24);     //IM1
DigitalIn a2(P0_25);    //ISRC1
DigitalIn a3(P0_26);    //ISNK1
DigitalOut io0(P2_0);   //EN1. on=1, off=0
DigitalIn io1(P2_1);    //TSD1

//CH2
AnalogIn a4(P1_30);     //VM2
AnalogIn a5(P1_31);     //IM2
DigitalIn tx(P0_2);     //ISNK2
DigitalIn rx(P0_3);     //ISRC2
DigitalOut io2(P2_2);   //EN2
DigitalIn io3(P2_3);    //TSD2

//SW
DigitalIn sw0(P1_21);   //rotary1 a
DigitalIn sw1(P1_22);   //rotary1 b
DigitalIn sw2(P1_23);   //rotary2 a
DigitalIn sw3(P1_24);   //rotary2 b
DigitalIn sw4(P1_25);   //en1 sw
DigitalIn sw5(P1_26);   //en2 sw

//OLED
const int oled1=0x78;   //oled i2c adr 0x3c<<1
const int oled2=0x7A;   //oled i2c adr 0x3d<<1
void oled_init(int adr);     //lcd init func
void char_disp(int adr, int8_t position, char data);    //char disp func
void val_disp(int adr, int8_t position, int8_t digit,int val);  //val disp func
void cont(int adr,uint8_t);     //contrast set

//Rotary state and display
const uint16_t tick_t_on=1000;    //lcd tick on time
const uint16_t tick_t_off=200;    //lcd tick off time
const uint16_t refresh=300;      //val disp refresh rate
uint16_t tc=0;          //tick counter
uint16_t ref_t=0;        //disp refresh counter
uint8_t r1_state;       //rotary 1 state
uint8_t r2_state;       //rotary 2 state
uint8_t r1_val=0;       //0->idle, 1->incr, 2->decr
uint8_t r2_val=0;       //0->idle, 1->incr, 2->decr
uint8_t cur_pos=0;
uint8_t tick_pos=0;
uint8_t disp_val[4];
int16_t vs1_p=0,vs2_p=0;    //past vset values
uint8_t en1_p=0,en2_p=0;    //past en values
uint8_t er1=0,er2=0;        //error flag

//calibration data
#define vs1_cal 1.023076//1.019252 //vs calib
#define vs2_cal 1.011236//1.012715 //vs calib
#define is1_cal 1.003512//0.995767 //is calib
#define is2_cal 1//1.002556 //is calib
#define vm1_cal 1.001477//1.002265 //vm calib
#define vm2_cal 1//1.011590 //vm calib
#define im1_cal 0.993676//0.996264 //im calib
#define im2_cal 1//0.987815 //im calib
float temp;
int16_t is_send;
uint16_t vs_send;

//DAC settings
#define vc 1800 //center  voltage
#define att 0.1 //vm attenuation
#define g_c 3   //LT1991 dif amp gain
#define g 3     //LT1970A dif amp gain
#define R 2   //sens R
#define res 4   //4.096/2^10 dac resolution
uint16_t calc_a(uint16_t is);
uint16_t calc_b(uint16_t is);
uint16_t calc_c(int16_t vs);
uint16_t calc_d(int16_t vs);
int16_t vs1,vs2;    //mV unit
int16_t is1=100,is2=100;   //mA unit. initial 100mA
uint8_t en1=0,en2=0;//en state. off=0
uint8_t spi_rate;           //spi transfer rate

//meas settings
float vm1_float,im1_float,vm2_float,im2_float;
float vm1_all,im1_all,vm2_all,im2_all;
int16_t vm1,im1,vm2,im2;
const uint16_t meas_t=500;            //meas average rate
uint16_t mc=0;                     //meas average counter
const uint16_t meas_disp_rate=1000;   //im display rate
uint16_t meas_disp_cnt=0;             //im display rate counter

int main(){
    spi.format(16,0);   //spi mode setting. 2byte(16bit) transfer, mode 0
    cs1=1;  //CS1 high
    cs2=1;  //cs2 high
    io0=0;  //ch1 off
    io2=0;  //ch2 off

    thread_sleep_for(100);  //wait for lcd power on
    oled_init(oled1);
    cont(oled1,0xff);
    char_disp(oled1,2,'.');
    char_disp(oled1,5,'V');
    char_disp(oled1,6,' ');
    char_disp(oled1,10,'m');
    char_disp(oled1,11,'A');
    char_disp(oled1,0x20+2,'.');
    char_disp(oled1,0x20+5,'V');
    char_disp(oled1,0x20+6,' ');
    char_disp(oled1,0x20+11,'m');
    char_disp(oled1,0x20+12,'A');
    char_disp(oled1,13,'O');

    oled_init(oled2);
    cont(oled2,0xff);
    char_disp(oled2,2,'.');
    char_disp(oled2,5,'V');
    char_disp(oled2,6,' ');
    char_disp(oled2,10,'m');
    char_disp(oled2,11,'A');
    char_disp(oled2,0x20+2,'.');
    char_disp(oled2,0x20+5,'V');
    char_disp(oled2,0x20+6,' ');
    char_disp(oled2,0x20+11,'m');
    char_disp(oled2,0x20+12,'A');
    char_disp(oled2,13,'O');

    while (true){
        //rotary scan
        r1_state=((r1_state<<1)+sw0)&0b0011;
        if((r1_state==2)&&(sw1==1))r1_val=1;      //r1 incr
        else if((r1_state==2)&&(sw1==0))r1_val=2; //r1 decr

        r2_state=((r2_state<<1)+sw2)&0b0011;
        if((r2_state==2)&&(sw3==1))r2_val=1;      //r2 incr
        else if((r2_state==2)&&(sw3==0))r2_val=2; //r2 decr        

        if(r2_val==1){
            if(cur_pos<14) ++cur_pos;
            else cur_pos=0;
        }else if(r2_val==2){
            if(cur_pos>0) --cur_pos;
            else cur_pos=14;
        }

        switch(cur_pos){
            case 0:
                tick_pos=16;break;
            case 1:
                tick_pos=0;
                if(r1_val==1)vs1=abs(vs1);
                else if(r1_val==2) vs1=-1*abs(vs1);break;
            case 2:
                tick_pos=1;
                if(r1_val==1)vs1=vs1+1000;
                else if(r1_val==2) vs1=vs1-1000;break;
            case 3:
                tick_pos=3;
                if(r1_val==1)vs1=vs1+100;
                else if(r1_val==2) vs1=vs1-100;break;
            case 4:
                tick_pos=4;
                if(r1_val==1)vs1=vs1+10;
                else if(r1_val==2) vs1=vs1-10;break;
            case 5:
                tick_pos=7;
                if(r1_val==1)is1=is1+100;
                else if(r1_val==2) is1=is1-100;break;
            case 6:
                tick_pos=8;
                if(r1_val==1)is1=is1+10;
                else if(r1_val==2) is1=is1-10;break;
            case 7:
                tick_pos=9;
                if(r1_val==1)is1=is1+1;
                else if(r1_val==2) is1=is1-1;break;
            case 8:
                tick_pos=0+10;
                if(r1_val==1)vs2=abs(vs2);
                else if(r1_val==2) vs2=-1*abs(vs2);break;
            case 9:
                tick_pos=1+10;
                if(r1_val==1)vs2=vs2+1000;
                else if(r1_val==2) vs2=vs2-1000;break;
            case 10:
                tick_pos=3+10;
                if(r1_val==1)vs2=vs2+100;
                else if(r1_val==2) vs2=vs2-100;break;
            case 11:
                tick_pos=4+10;
                if(r1_val==1)vs2=vs2+10;
                else if(r1_val==2) vs2=vs2-10;break;
            case 12:
                tick_pos=7+10;
                if(r1_val==1)is2=is2+100;
                else if(r1_val==2) is2=is2-100;break;
            case 13:
                tick_pos=8+10;
                if(r1_val==1)is2=is2+10;
                else if(r1_val==2) is2=is2-10;break;
            case 14:
                tick_pos=9+10;
                if(r1_val==1)is2=is2+1;
                else if(r1_val==2) is2=is2-1;break;
        }
        r1_val=0;
        r2_val=0;

        //EN SW check
        if(sw4==1){         //sw off
            en1=0;
            io0=0;
        }else if(sw4==0){   //se on
            en1=1;
            io0=1;
        }
        if(sw5==1){         //sw off
            en2=0;
            io2=0;
        }else if(sw5==0){   //se on
            en2=1;
            io2=1;
        }

        //vset overflow check
        if(vs1>=9999)vs1=9999;
        if(vs1<=-9999)vs1=-9999;
        if(is1>=200)is1=200;
        if(is1<=0)is1=0;
        if(vs2>=9999)vs2=9999;
        if(vs2<=-9999)vs2=-9999;
        if(is2>=200)is2=200;
        if(is2<=0)is2=0;
        
        //vset disp
        ++tc;
        if(tc<tick_t_on){
            ++ref_t;
            if(ref_t==refresh){
                ref_t=0;
                //ch1 disp
                if(vs1_p==vs1){
                    if(vs1>=0)char_disp(oled1,0,'+');
                    else if(vs1<0)char_disp(oled1,0,'-');
                    disp_val[1]=(abs(vs1)/10)%100;    //10+100 mV
                    disp_val[0]=abs(vs1)/1000;        //1 V
                    val_disp(oled1,1,1,disp_val[0]);
                    val_disp(oled1,3,2,disp_val[1]);
                    val_disp(oled1,7,3,is1);
                }
                if(en1_p==en1){
                    if(en1==1){
                        char_disp(oled1,14,'N');
                        char_disp(oled1,15,' ');
                    }else if(en1==0){
                        char_disp(oled1,14,'F');
                        char_disp(oled1,15,'F');
                    }
                }
                //ch2 disp
                if(vs2_p==vs2){
                    if(vs2>=0)char_disp(oled2,0,'+');
                    else if(vs2<0)char_disp(oled2,0,'-');
                    disp_val[1]=(abs(vs2)/10)%100;    //10+100 mV
                    disp_val[0]=abs(vs2)/1000;        //1 V
                    val_disp(oled2,1,1,disp_val[0]);
                    val_disp(oled2,3,2,disp_val[1]);
                    val_disp(oled2,7,3,is2);
                }
                if(en2_p==en2){
                    if(en2==1){
                        char_disp(oled2,14,'N');
                        char_disp(oled2,15,' ');
                    }else if(en2==0){
                        char_disp(oled2,14,'F');
                        char_disp(oled2,15,'F');
                    }
                }
            }
        }else if(tc==tick_t_on){
            if(tick_pos<10)char_disp(oled1,tick_pos,' ');
            if(tick_pos>=10)char_disp(oled2,tick_pos-10,' ');
        }else if(tc==tick_t_on+tick_t_off){
            tc=0;
        }else{
            //nothing
        }

        //store past values
        vs1_p=vs1;
        vs2_p=vs2;
        en1_p=en1;
        en2_p=en2;

        //VM&IM average
        if(mc==meas_t){
            mc=0;
            temp=vm1_all/meas_t*vm1_cal;
            vm1=(int16_t)temp;
            temp=im1_all/meas_t*im1_cal;
            im1=(int16_t)temp;
            temp=vm2_all/meas_t*vm2_cal;
            vm2=(int16_t)temp;
            temp=im2_all/meas_t*im2_cal;
            im2=(int16_t)temp;
            vm1_all=0;
            im1_all=0;
            vm2_all=0;
            im2_all=0;
        }else{
            ++mc;
            //ch1 meas
            vm1_float=((a0.read()*3300)-vc)/att+vc;
            im1_float=((a1.read()*3300)-vc)/R/g_c*-1;
            vm1_all=vm1_all+vm1_float;
            im1_all=im1_all+im1_float;
            //ch2 meas
            vm2_float=((a4.read()*3300)-vc)/att+vc;
            im2_float=((a5.read()*3300)-vc)/R/g_c*-1;
            vm2_all=vm2_all+vm2_float;
            im2_all=im2_all+im2_float;
        }
        
        //meas&error disp
        ++meas_disp_cnt;
        if(meas_disp_cnt==meas_disp_rate){
            meas_disp_cnt=0;
            //ch1 disp
            if(vm1>=0)char_disp(oled1,0x20+0,'+');
            else if(vm1<0)char_disp(oled1,0x20+0,'-');
            disp_val[1]=(abs(vm1)/10)%100;    //10+100 mV
            disp_val[0]=abs(vm1)/1000;        //1 V
            val_disp(oled1,0x20+1,1,disp_val[0]);
            val_disp(oled1,0x20+3,2,disp_val[1]);
            if(im1>=0)char_disp(oled1,0x20+7,'+');
            else if(im1<0)char_disp(oled1,0x20+7,'-');
            val_disp(oled1,0x20+8,3,abs(im1));
            //ch2 disp
            if(vm2>=0)char_disp(oled2,0x20+0,'+');
            else if(vm2<0)char_disp(oled2,0x20+0,'-');
            disp_val[1]=(abs(vm2)/10)%100;    //10+100 mV
            disp_val[0]=abs(vm2)/1000;        //1 V
            val_disp(oled2,0x20+1,1,disp_val[0]);
            val_disp(oled2,0x20+3,2,disp_val[1]);
            if(im2>=0)char_disp(oled2,0x20+7,'+');
            else if(im2<0)char_disp(oled2,0x20+7,'-');
            val_disp(oled2,0x20+8,3,abs(im2));
            
            //error check
            //ch1
            if(a2==0||a3==0||io1==0){
                if(er1==0){
                    er1=1;
                    if(a2==0||a3==0){
                        char_disp(oled1,0x20+15,'C');
                    }else if(io1==0){
                        char_disp(oled1,0x20+15,'T');
                    }
                }
            }else{
                if(er1==1){
                    er1=0;
                    char_disp(oled1,0x20+15,' ');
                }
            }
            //ch2
            if(tx==0||rx==0||io3==0){
                if(er2==0){
                    er2=1;
                    if(tx==0||rx==0){
                        char_disp(oled2,0x20+15,'C');
                    }else if(io3==0){
                        char_disp(oled2,0x20+15,'T');
                    }
                }
            }else{
                if(er2==1){
                    er2=0;
                    char_disp(oled2,0x20+15,' ');
                }
            }
        }
        
        //SPI transfer
        ++spi_rate;
        if(spi_rate==10){
            spi_rate=0;
            //ch1 dac set
            if(en1==0){         //EN1 OFF
                cs1=0;
                spi.write(calc_a(0));
                cs1=1;
                cs1=0;
                spi.write(calc_b(0));
                cs1=1;
                cs1=0;
                spi.write(calc_c(0));
                cs1=1;
                cs1=0;
                spi.write(calc_d(0));
                cs1=1;         
            }else if(en1==1){   //EN1 ON
                temp=(float)is1*is1_cal;
                is_send=(int16_t)temp;
                temp=(float)vs1*vs1_cal;
                vs_send=(uint16_t)temp;
                cs1=0;
                spi.write(calc_a(is_send));
                cs1=1;
                cs1=0;
                spi.write(calc_b(is_send));
                cs1=1;
                cs1=0;
                spi.write(calc_c(vs_send));
                cs1=1;
                cs1=0;
                spi.write(calc_d(vs_send));
                cs1=1;
            }
            //ch2 dac set
            if(en2==0){         //EN2 OFF
                cs2=0;
                spi.write(calc_a(0));
                cs2=1;
                cs2=0;
                spi.write(calc_b(0));
                cs2=1;
                cs2=0;
                spi.write(calc_c(0));
                cs2=1;
                cs2=0;
                spi.write(calc_d(0));
                cs2=1;         
            }else if(en2==1){   //EN2 ON
                temp=(float)is2*is2_cal;
                is_send=(int16_t)temp;
                temp=(float)vs2*vs2_cal;
                vs_send=(uint16_t)temp;
                cs2=0;
                spi.write(calc_a(is_send));
                cs2=1;
                cs2=0;
                spi.write(calc_b(is_send));
                cs2=1;
                cs2=0;
                spi.write(calc_c(vs_send));
                cs2=1;
                cs2=0;
                spi.write(calc_d(vs_send));
                cs2=1;
            }
        }
    }
}

//DAC reg calc func.
uint16_t calc_a(uint16_t is){
    uint16_t a_val;
    a_val=(is*10*R)/res;            //ilim dac val
    return (0b0001<<12)+(a_val<<2); //daca=0b0001
}
uint16_t calc_b(uint16_t is){
    uint16_t a_val;
    a_val=(is*10*R)/res;
    return (0b0010<<12)+(a_val<<2); //dacb=0b0010
}
uint16_t calc_c(int16_t vs){
    uint16_t c_val;
    if(vs<0) c_val=0;
    else c_val=(uint16_t)abs(vs)/g/res;
    return (0b0011<<12)+(c_val<<2); //dacc=0b0011
}
uint16_t calc_d(int16_t vs){
    uint16_t d_val;
    if(vs<0) d_val=(uint16_t)abs(vs)/g/res;
    else d_val=0;
    return (0b0100<<12)+(d_val<<2); //dacd=0b0100
}

//LCD init func
void oled_init(int adr){
    char lcd_data[2];
    lcd_data[0] = 0x0;
    lcd_data[1]=0x01;           //0x01 clear disp
    i2c.write(adr, lcd_data, 2);
    thread_sleep_for(20);
    lcd_data[1]=0x02;           //0x02 return home
    i2c.write(adr, lcd_data, 2);
    thread_sleep_for(20);
    lcd_data[1]=0x0C;           //0x0c disp on
    i2c.write(adr, lcd_data, 2);
    thread_sleep_for(20);
    lcd_data[1]=0x01;           //0x01 clear disp
    i2c.write(adr, lcd_data, 2);
    thread_sleep_for(20);
}

void char_disp(int adr, int8_t position, char data){
    char buf[2];
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(adr,buf, 2);
    buf[0]=0x40;            //ahr disp cmd
    buf[1]=data;
    i2c.write(adr,buf, 2);
}

//disp val func
void val_disp(int adr, int8_t position, int8_t digit, int val){
    char buf[2];
    char data[4];
    int8_t i;
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(adr,buf, 2);
    data[3]=0x30+val%10;        //1
    data[2]=0x30+(val/10)%10;   //10
    data[1]=0x30+(val/100)%10;  //100
    data[0]=0x30+(val/1000)%10; //1000
    buf[0]=0x40;
    for(i=0;i<digit;++i){
        buf[1]=data[i+4-digit];
        i2c.write(adr,buf, 2);
    }
}

void cont(int adr,uint8_t val){
    char buf[2];
    buf[0]=0x0;
    buf[1]=0x2a;
    i2c.write(adr,buf,2);
    buf[1]=0x79;    //SD=1
    i2c.write(adr,buf,2);
    buf[1]=0x81;    //contrast set
    i2c.write(adr,buf,2);
    buf[1]=val;    //contrast value
    i2c.write(adr,buf,2);
    buf[1]=0x78;    //SD=0
    i2c.write(adr,buf,2);
    buf[1]=0x28;    //0x2C, 0x28
    i2c.write(adr,buf,2);
}