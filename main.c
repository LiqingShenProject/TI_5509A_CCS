/*
This example utilize the DSP core of TMS320VC5509A to generate error corrected xz_x, xz_y.
Due to the non-linearity of Lens, the xy point on the focal plane suffered pin-cushion and barrel distortion
A 128X128 error correction table is generated from FPGA and sent to DSP via CE_0# external memory interface
The original x,y line buffer is also filled up by FPGA via CE_0# external memory interface
The example here use a linear interpolation method the error correct the original x,y.
By trial and error, we can get the perfect error correction suiting our laser marking requirement
This is only a snippet of a complete project. The generated xz_x and xz_y will eventually be sent out to
the laser scanner which is omitted here for the sake of brevity
*/
#include <csl.h>
#include <csl_chiphal.h>
#include <csl_chip.h>
#include <csl_pll.h>
#include <csl_i2c.h>
#include <csl_dma.h>
#include <csl_emif.h>
#include <stdio.h>
//----------------------------- FPGA memory mapping-----------------------------------------------------------
// DSP utilize its external CE0# memory interface to get data from FPGA to process it. The FPGA RTL need to use handling
// the timing and bus protocol of DRAM interface in order to communicate with DSP
// Pipe00 is data from FPGA to DSP, Pipe01 is command from FPGA to DSP
#define  Port_Pipe0Data    *(  (volatile unsigned int * )0x200000)  //   data @ external memory address 0x200000
#define  Port_Pipe1Data    *(  (volatile unsigned int * )0x200010)  //   data @ external memory address 0x200010
//----------------------------global data----------------------------------------------------
volatile unsigned short Volatile_data_temp;
unsigned short buf[16]; // store the data sent from FPGA
unsigned short xz_x,xz_y; // error corrected x y coordinate
Uint16 databyte1[1]={0xA}; // I2C DSP ready feedback
Uint16 databyte2[1]={0x5}; // I2C DSP busy feedback
//---------Data structure---------------------------------------------------------------------
// The data from FPGA consists of 16 short data. The first 2 short data are overhead context
typedef struct
{
    unsigned short task_flag;
    unsigned short ctrl_code;
    unsigned short data2;
    unsigned short data3;
    unsigned short data4;
    unsigned short data5;
    unsigned short data6;
    unsigned short data7;
    unsigned short data8;
    unsigned short data9;
    unsigned short data10;
    unsigned short data11;
    unsigned short data12;
    unsigned short data13;
    unsigned short data14;
    unsigned short data15;
} Mycomm;

Mycomm datacomm; // this is the data received from external FPGA via EMIF
Mycomm readcmd; // this is the command received from external FPGA via EMIF
//-------------------------
typedef struct
{
    unsigned short xda;
    unsigned short yda;
} MyDa;  // The data struture of each element in XY error correction table
#pragma DATA_SECTION(correctionTable, ".BOXDATA")
MyDa errorCorrectionTable[128][128]; // create 128 * 128 table
MyDa lineBuffer[8]; // create line buffer store 8 points

//---------create CSL55 peripheral configuration------------------------------------------------------
/* Create and initialize an I2C initialization structure */
// I2C with FPGA to signal the data flow
I2C_Setup Init = {
        0,              /* 7 bit address mode */
        0x0020,         /* own address - don't care if master */
        144,            /* clkout value (Mhz)  */
        400,            /* a number between 10 and 400*/
        0,              /* number of bits/byte to be received or transmitted (8)*/
        1,              /* DLB mode on*/
        1               /* FREE mode of operation on*/
};

// this following function assign value to 18 registers related to EMIF mapping
void MEM_INIT(void)

{
  EMIF_FSET(EGCR,WPE,0);  //disable writing posting
  EMIF_FSET(EGCR,MEMFREQ,1);  //100M MEMCLK;
  //EMIF_FSET(EGCR,MEMFREQ,2);  //50M MEMCLK;
  EMIF_FSET(EGCR,MEMCEN,1);  //memory:0 Disabled
  EMIF_FSET(EGCR,ARDY,1);  //
//  EMIF_RSET(EGCR,0x220);  //
  //--------------
  //---CEn1---
  //CE0  from 020000 ( word addr)
  EMIF_FSET(CE01,MTYPE,1);  //001b Asynchronous, 16-bit data bus width
  EMIF_FSET(CE01,RDSETUP,3); // fast asyn_ram,
  EMIF_FSET(CE01,RDSTROBE,16); // fast asyn_ram,
  EMIF_FSET(CE01,RDHOLD,3); // fast asyn_ram,

  //CE1  from word addr of 200000
  EMIF_FSET(CE11,MTYPE,1);  //001b Asynchronous, 16-bit data bus width
  EMIF_FSET(CE11,RDSETUP,2); // slow asyn_ram,
  EMIF_FSET(CE11,RDSTROBE,8); // slow asyn_ram,
//  EMIF_FSET(CE11,RDSTROBE,20); // slow asyn_ram,

  EMIF_FSET(CE11,RDHOLD,3); // slow asyn_ram,

  //CE2
  EMIF_FSET(CE21,MTYPE,3);  //011b Synchronous DRAM (SDRAM), 16-bit data bus width
  EMIF_FSET(CE21,RDSETUP,1); // fast asyn_ram,
  EMIF_FSET(CE21,RDSTROBE,1); // fast asyn_ram,
  EMIF_FSET(CE21,RDHOLD,1); // fast asyn_ram,

  EMIF_FSET(CE22,WREXHLD,3);  //fast asyn_ram
  EMIF_FSET(CE22,WRSETUP,1);  //fast asyn_ram
  EMIF_FSET(CE22,WRSTROBE,1);  //fast asyn_ram
  EMIF_FSET(CE22,WRHOLD,3);   //fast asyn_ram
  //CE3
  EMIF_FSET(CE31,MTYPE,3);  //011b Synchronous DRAM (SDRAM), 16-bit data bus width

  //-------------------
  //---CEn2---
  //CE0
  EMIF_FSET(CE02,RDEXHLD,3);  //fast asyn_ram
  EMIF_FSET(CE02,WREXHLD,3);  //fast asyn_ram
  EMIF_FSET(CE02,WRSETUP,3);  //fast asyn_ram
  EMIF_FSET(CE02,WRSTROBE,16);  //fast asyn_ram
  EMIF_FSET(CE02,WRHOLD,3);   //fast asyn_ram
  //CE1
  EMIF_FSET(CE12,RDEXHLD,3);  //slow asyn_ram   0 � n ≤ 3 (n + 1) CPU clock cycles
  EMIF_FSET(CE12,WREXHLD,1);  //slow asyn_ram   0 ≤ n ≤ 3 (n + 1) CPU clock cycles
  EMIF_FSET(CE12,WRSETUP,1);  //slow asyn_ram  3 ≤ n ≤ 15 n CPU clock cycles
//  EMIF_FSET(CE12,WRSTROBE,63); //slow asyn_ram    2 ≤ n ≤ 63 n CPU clock cycles
  EMIF_FSET(CE12,WRSTROBE,7); //slow asyn_ram  2 � n ≤ 63 n CPU clock cycles

  EMIF_FSET(CE12,WRHOLD,3);   //slow asyn_ram   0 ≤ n ≤ 3 n CPU clock cycles
  //----CEn3----
  //CE0
  EMIF_FSET(CE03,TIMOUT,0);  //Time-out feature disabled
  EMIF_FSET(CE13,TIMOUT,0);  //Time-out feature disabled

  //----------------------

  EMIF_FSET(SDC1,TRC,0x110);
  EMIF_FSET(SDC1,SDSIZE,0);//4M x 16 bits (64M bits)
  EMIF_FSET(SDC1,RFEN,1);//SDRAM auto-refreshes enabled
  EMIF_FSET(SDC1,TRCD,1);//
  EMIF_FSET(SDC1,TRP,1);//
  EMIF_FSET(SDC2,TRAS,3);
  EMIF_FSET(SDC2,TACTV2ACTV,1);
  //------------------------------
  //at last reset the emif
  EMIF_FSET(EMIRST,EMIRST,5);  //
  EMIF_FSET(INIT,INIT,0);
}

//---------Function prototypes--------------------------------------------------------------------
void readdata(void)
{
    int i;
    Volatile_data_temp=Port_Pipe0Data;
    for(i=0;i<16;i++)
    {
      buf[i] = Volatile_data_temp ++;
    }
    memcpy(&datacomm,&buf[0],16);
}//for read data from FPGA

void readcommand(void)
{
   int i;
   Volatile_data_temp=Port_Pipe1Data;
   for(i=0;i<16;i++)
   {
      buf[i] =Volatile_data_temp ++;
   }
   memcpy(&readcmd,&buf[0],16);
}//for read command from FPGA

void setTable(void)
{
    unsigned short tableBuf[30],xc,yc,index,i;
    xc=0;
    yc=0;
    datacomm.task_flag=0;
    while (yc < 127)
    {
      readdata();
      if ((datacomm.task_flag ==4) & (datacomm.ctrl_code ==4)) // check the context
       {
          for (i=0;i<14;i++)
          {
              tableBuf[i]=buf[i+2];
           }
           index=0;
           for (i=0;i<7;i++)
           {
               errorCorrectionTable[xc][yc].xda=tableBuf[index];
               index++;
               errorCorrectionTable[xc][yc].yda=tableBuf[index];
               index++;
               xc++;
               if (xc>127)
               {
                   xc=0;
                   yc++;
               }
           }
        }
    }
}// for set up correction table

void getXYdata(void)
{
    unsigned short dataBuf[30],i;
    datacomm.task_flag=0;
    readdata();
    if ((datacomm.task_flag ==1) & (datacomm.ctrl_code ==1)) // check the context
        {
          for (i=0;i<14;i++)
          {
              dataBuf[i]=buf[i+2];
          }
          for (i=0;i<7;i++)
          {
              lineBuffer[i].xda=dataBuf[i];
          }
        }
}// for set up line buffer

// linear interpolation based on error correction table
// The 2-D plane is divided into 128X128 grid box, the length of each Grid (i,j) is g
// The code first need to decide which grid box the target point fall into
// and then judege if the point is above or below the diagonal line of the grid box
// use linear interpolation formula to generate error correction value
void errorCorrection(Uint16 x,Uint16 y)
{

    int xv,yv,xv2,yv2;
    Uint16 xx,yy,xa,ya;
    Uint16 pdxa,pdya;
    long tempx,tempy;
    long nxz_x,nxz_y;

    //the range of unsigned is [0,65535], since x,y are divided into 128 boxes，
    //so for every 65536/128 = 512 increment to the next box

    xx=x>>9; //get index of box in x direction
    yy=y>>9; //get index of box in y direction
    xa=x & 511; //get the remainder of x/512,  xa/512 = dx/g
    ya=y & 511; //get the remainder of y/512,  ya/512 = dy/g
    pdxa=x & 511;
    pdya=y & 511;

    if (pdxa>=pdya) // if the point locate below the or on diagonal line of the box
    {
        xv =errorCorrectionTable[xx+1][yy].xda-errorCorrectionTable[xx][yy].xda; //Gridx(i+1,j)-Gridx(i,j)
        yv=errorCorrectionTable[xx+1][yy+1].yda-errorCorrectionTable[xx+1][yy].yda; //Gridy(i+1,j+1)-Gridy(i+1,j)
        xv2=errorCorrectionTable[xx+1][yy+1].xda-errorCorrectionTable[xx+1][yy].xda; //Gridx(i+1,j+1)-Gridx(i+1,j)
        yv2=errorCorrectionTable[xx+1][yy].yda-errorCorrectionTable[xx][yy].yda; //Gridy(i+1,j)-Gridy(i,j)

        //xv+xv2 = Gridx(i+1,j)- Gridx(i,j) + Gridx(i+1,j+1)-Gridx(i+1,j) = Gridx(i+1,j+1) - Gridx(i,j)

        tempx=((long)xv)    *   ((long)xa);
        tempy=((long)yv)    *   ((long)ya);
        nxz_x= ((long)xv2)   *   ((long)ya);
        nxz_y= ((long)yv2)   *   ((long)xa);

        /**
        xz_x=errorCorrectionTable[xx][yy].xda+(tempx+nxz_x)/512
            =errorCorrectionTable[xx][yy].xda+(((long)xv)    *   ((long)xa)+((long)xv2)   *   ((long)ya))/512
            =errorCorrectionTable[xx][yy].xda + xv*(xa/512)+xv2*(ya/512)
            = Gridx(i,j)+ (Gridx(i+1,j)-Gridx(i,j))*(dx/g) + (Gridx(i+1,j+1)-Gridx(i+1,j))*(dy/g)

        xz_y=errorCorrectionTable[xx][yy].yda+(tempy+nxz_y)/512
            =errorCorrectionTable[xx][yy].yda+(((long)yv)    *   ((long)ya)+((long)yv2)   *   ((long)xa))/512
            =errorCorrectionTable[xx][yy].yda + yv*(ya/512)+yv2*(xa/512)
            =Gridx(i,j)+ (Gridy(i+1,j+1)-Gridy(i+1,j))*(dy/g) + (Gridy(i+1,j)-Gridy(i,j))*(dx/g)

        **/
        xz_x=errorCorrectionTable[xx][yy].xda+(tempx+nxz_x)/512;
        xz_y=errorCorrectionTable[xx][yy].yda+(tempy+nxz_y)/512;

    }
    else //if the point is above the diagonal line

        xv=errorCorrectionTable[xx+1][yy+1].xda-errorCorrectionTable[xx][yy+1].xda;//Gridx(i+1,j+1)-Gridx(i,j+1)
        yv=errorCorrectionTable[xx][yy+1].yda-errorCorrectionTable[xx][yy].yda; //Gridy(i,j+1)-Gridy(i,j)
        xv2=errorCorrectionTable[xx][yy+1].xda-errorCorrectionTable[xx][yy].xda;//Gridx(i,j+1)-Gridx(i,j)
        yv2=errorCorrectionTable[xx+1][yy+1].yda-errorCorrectionTable[xx][yy+1].yda;//Gridy(i+1,j+1)-Gridy(i,j+1)


        tempx=((long)xv)  *((long)xa);
        tempy=((long)yv)  *((long)ya);
        nxz_x= ((long)xv2) *((long)ya);
        nxz_y= ((long)yv2) *((long)xa);

        /**
        xz_x=errorCorrectionTable[xx][yy].xda+(tempx+nxz_x)/512
            =errorCorrectionTable[xx][yy].xda+(((long)xv)    *   ((long)xa)+((long)xv2)   *   ((long)ya))/512
            =errorCorrectionTable[xx][yy].xda + xv*(xa/512)+xv2*(ya/512)
            = Gridx(i,j)+ (Gridx(i+1,j+1)-Gridx(i,j+1))*(dx/g) + (Gridx(i,j+1)-Gridx(i,j))*(dy/g)

        xz_y=errorCorrectionTable[xx][yy].yda+(tempy+nxz_y)/512
            =errorCorrectionTable[xx][yy].yda+(((long)yv)    *   ((long)ya)+((long)yv2)   *   ((long)xa))/512
            =errorCorrectionTable[xx][yy].yda + yv*(ya/512)+yv2*(xa/512)
            =Gridx(i,j)+ (Gridy(i,j+1)-Gridy(i,j))*(dy/g) + (Gridy(i+1,j+1)-Gridy(i,j+1))*(dx/g)

        **/
        xz_x=errorCorrectionTable[xx][yy].xda+(tempx+nxz_x)/512;
        xz_y=errorCorrectionTable[xx][yy].yda+(tempy+nxz_y)/512;

}

//---------main routine---------
void main(void)
{
    /* peripheral initialization */
    CSL_init();
    I2C_setup(&Init);
    PLL_setFreq(10,1);  //200M

    CHIP_RSET(SYSR,0x0002);//CLKDIV 002 = CLKOUT represents the CPU clock divided by 2
    CHIP_FSET(ST3_55,MPNMC,1);   // by June in 20100505==sdram 4M
    MEM_INIT();

    while (1)
    {
        // I2C_write (Uint16 *data, int length, int master, Uint16 slaveaddress,int transfermode, int timeout)
        // write data to issue DSP is ready
        I2C_write(databyte1,1,1,0x20,1,30000);
        int t;
        for (t=0; t<200; t++); // delay
        readcmd.task_flag = 0;
        readcommand();
        if (readcmd.task_flag > 0)
        {
           I2C_write(databyte2,1,1,0x20,1,30000); // acknowledge current command is being processed
        // read control c
         switch (readcmd.ctrl_code)
         {
          case 2000: {
            getXYdata(); //fill up lineBuffer[8]
            int i;
            for (i=0; i<8; i++)
                errorCorrection(lineBuffer[i].xda, lineBuffer[i].yda);
          }
          break;
          case 1000: setTable(); // get the error correction table
         }
        }
    }
}
