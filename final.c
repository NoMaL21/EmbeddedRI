#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h> 
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <string.h>
#include <math.h>
#include <softTone.h>

#define TOTAL_NOTES 32

int notes[TOTAL_NOTES] = {
    330, 294, 262, 294, 330, 330, 330, 330,
    294, 294, 294, 294, 330, 330, 330, 330,
    330, 294, 262, 294, 330, 330, 330, 330,
    294, 294, 330, 294, 262, 262, 262, 262
};
void musicPlay(int gpio)
{
    int i;

    softToneCreate (gpio);

    for(i = 0 ; i < TOTAL_NOTES; i++)
    {
        softToneWrite (gpio, notes[i]); //
        delay (250); //
    }
}
void musicStop(int gpio)
{
    softToneWrite(gpio, 0);
}


#define SLAVE_ADDR_01 0x68
#define SLAVE_ADDR_02 0x48
#define BAUD_RATE 115200
static const char* UART2_DEV = "/dev/ttyAMA1";

static const char* I2C_DEV = "/dev/i2c-1";

int share;
pthread_mutex_t mid;

void serialWrite(const int fd, const unsigned char c)
{
    write(fd, &c, 1);   
}

void serialWriteBytes(const int fd, const char *s)
{
    write(fd,s, strlen(s));
}

void *threadFuncTime (void *data){
    int i2c_fd1;
    int hour, min, sec;


    if ((i2c_fd1 = wiringPiI2CSetupInterface (I2C_DEV, SLAVE_ADDR_01)) < 0 ){
        printf("wiringPi2CSetup Failed: \n");   
    }

    printf("I2C start.....\n");

    

    while(1) {
        hour = wiringPiI2CReadReg8(i2c_fd1, 0x02);
        min = wiringPiI2CReadReg8(i2c_fd1, 0x01);
        sec = wiringPiI2CReadReg8(i2c_fd1, 0x00);

        printf("%x - %x - %x\n", hour, min, sec);
        delay(1000);

        if((hour == 0x09) && (min == 0x00) && (sec == 0x00))
        {
            pthread_mutex_lock(&mid); // 뮤텍스 잠금
            share = 1;
            pthread_mutex_unlock(&mid); // 뮤텍스 잠금 해제
            
            musicPlay(12);
            musicStop(12);
        }

    }
}

void *threadFuncTemp (void *data){
    int fd_serial;
    int preval, temp;
    int adcChannel = 0;
    int i2c_fd2;
    unsigned char string1[100];
    long double tmp, r6, ft, tem;

    int a = 1;
    if ((i2c_fd2 = wiringPiI2CSetupInterface (I2C_DEV, SLAVE_ADDR_02)) < 0 ){
        printf("wiringPi2CSetup Failed: \n");   
    }

    if((fd_serial = serialOpen(UART2_DEV, BAUD_RATE)) < 0){
        printf("Unable to open serial device. \n");
    }

    //share = 1;

    while(1){
        wiringPiI2CWrite(i2c_fd2, 0x41 | adcChannel);
        preval = wiringPiI2CRead(i2c_fd2);
        temp = wiringPiI2CRead(i2c_fd2);
        delay(500);
        if(share == 1){
            r6 = (1000 * temp) / (256 - temp);
            tmp = (1 / 298.15) + ((1 / 3950) * log(r6 / 10000));

            tem = (293 * 3834) / (3834 + (293*(log(r6/10000))));
            ft = tem - 273.15;

            //printf("Current T_C = %f, \n", ft);
            //printf("Current T_R6 = %f, \n", r6);
            //printf("Current T_Censor = %d, \n", temp);         

            sprintf(string1, "오늘의 온도는 : %.1f ℃ 입니다.", ft);
            fflush(stdout);
            serialWriteBytes(fd_serial, string1);

            pthread_mutex_lock(&mid); // 뮤텍스 잠금
            share = 0;
            pthread_mutex_unlock(&mid); // 뮤텍스 잠금 해제
        }
    }
}

int main(void){
    pthread_t p_thread1; 
    pthread_t p_thread2; 
    int time; 
    int temp;
    int err;
    void* status;

    if(wiringPiSetupGpio() < 0 ){
        printf("wiringPiSetup() is failed\n");
    }

    pthread_mutex_init(&mid, NULL);

    time = pthread_create(&p_thread1, NULL, threadFuncTime, NULL);
    if(time < 0){ 
        perror("pthread_create() error\n");
        exit(0);
    }
    temp = pthread_create(&p_thread2, NULL, threadFuncTemp, NULL);
    if(temp < 0){ 
        perror("pthread_create() error\n");
        exit(0);
    }

    err = pthread_join(p_thread1, (void **)&status);
    if(err < 0){ 
        perror("pthread_join() error\n");
        exit(0);
    }

    pthread_mutex_destroy(&mid);

    return 0;
}