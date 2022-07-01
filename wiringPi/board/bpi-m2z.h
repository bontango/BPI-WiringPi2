// file bpi-m2z.h
// bontango 06.2022
// adjusted from bpi-m2p.h for Banana PI M2 Zero

#define BPI_M2Z_01      -1
#define BPI_M2Z_03      GPIO_PA02
#define BPI_M2Z_05      GPIO_PA03
#define BPI_M2Z_07      GPIO_PA04
#define BPI_M2Z_09      -1
#define BPI_M2Z_11      GPIO_PA17
#define BPI_M2Z_13      GPIO_PA27
#define BPI_M2Z_15      GPIO_PA22
#define BPI_M2Z_17      -1
#define BPI_M2Z_19      GPIO_PA10
#define BPI_M2Z_21      GPIO_PA09
#define BPI_M2Z_23      GPIO_PA11
#define BPI_M2Z_25      -1
#define BPI_M2Z_27      GPIO_PA00
#define BPI_M2Z_29      GPIO_PA05
#define BPI_M2Z_31      GPIO_PA06
#define BPI_M2Z_33      GPIO_PA13
#define BPI_M2Z_35      GPIO_PA19
#define BPI_M2Z_37      GPIO_PA26
#define BPI_M2Z_39      -1

#define BPI_M2Z_02      -1
#define BPI_M2Z_04      -1
#define BPI_M2Z_06      -1
#define BPI_M2Z_08      GPIO_PA14
#define BPI_M2Z_10      GPIO_PA15
#define BPI_M2Z_12      GPIO_PA18
#define BPI_M2Z_14      -1
#define BPI_M2Z_16      GPIO_PA23
#define BPI_M2Z_18      GPIO_PA24
#define BPI_M2Z_20      -1
#define BPI_M2Z_22      GPIO_PA25
#define BPI_M2Z_24      GPIO_PA08
#define BPI_M2Z_26      GPIO_PA07
#define BPI_M2Z_28      GPIO_PA01
#define BPI_M2Z_30      -1
#define BPI_M2Z_32      GPIO_PA12
#define BPI_M2Z_34      -1
#define BPI_M2Z_36      GPIO_PA16
#define BPI_M2Z_38      GPIO_PA20
#define BPI_M2Z_40      GPIO_PA21

//map phys_num(index) to bp gpio_num(element)
//physical to (virtual)BCM in gpio readall
int physToGpio_BPI_M2Z [64] =
{
          -1,                //0
          -1,        -1,     //1, 2
   BPI_M2Z_03,        -1,     //3, 4
   BPI_M2Z_05,        -1,     //5, 6
   BPI_M2Z_07, BPI_M2Z_08,     //7, 8
          -1, BPI_M2Z_10,     //9, 10
   BPI_M2Z_11, BPI_M2Z_12,     //11, 12
   BPI_M2Z_13,        -1,     //13, 14
   BPI_M2Z_15, BPI_M2Z_16,     //15, 16
          -1, BPI_M2Z_18,     //17, 18
   BPI_M2Z_19,        -1,     //19, 20
   BPI_M2Z_21, BPI_M2Z_22,     //21, 22
   BPI_M2Z_23, BPI_M2Z_24,     //23, 24
          -1, BPI_M2Z_26,     //25, 26
   BPI_M2Z_27, BPI_M2Z_28,     //27, 28
   BPI_M2Z_29,        -1,     //29, 30
   BPI_M2Z_31, BPI_M2Z_32,     //31, 32
   BPI_M2Z_33,        -1,     //33, 34
   BPI_M2Z_35, BPI_M2Z_36,     //35, 36
   BPI_M2Z_37, BPI_M2Z_38,     //37, 38
          -1, BPI_M2Z_40,     //39, 40
   -1,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //41-> 55
   -1,   -1, -1, -1, -1, -1, -1, -1 // 56-> 63
} ;
 //map (virtual)BCM from Pi to physical BCM for banana 2M zero
//these numbers can be found in sysfs access via /sys/class/gpio
int bcmTo_BPI_M2Z_bcm [64] =
{
        GPIO_PA19,  //0
        GPIO_PA18,  //1
        GPIO_PA12,  //2
        GPIO_PA11,  //3
        GPIO_PA06,  //4
        GPIO_PA07,  //5
        GPIO_PA08,  //6
        GPIO_PC07,  //7
        GPIO_PC03,  //8
        GPIO_PC01,  //9
        GPIO_PC00,  //10
        GPIO_PC02,  //11
        GPIO_PL02,  //12
        GPIO_PA09,  //13
        GPIO_PA13,  //14
        GPIO_PA14,  //15
        GPIO_PL04,  //16
        GPIO_PA01,  //17
        GPIO_PA16,  //18
        GPIO_PA10,  //19
        GPIO_PA21,  //20
        GPIO_PA20,  //21
        GPIO_PA03,  //22
        GPIO_PA15,  //23
        GPIO_PC04,  //24
        GPIO_PA02,  //25
        GPIO_PA17,  //26
        GPIO_PA00,  //27
        -1, -1, -1,  //28-> 30
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,  //31-> 40
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,  //41-> 50
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,  //51-> 60
        -1, -1, -1,  //61->63 
} ;

