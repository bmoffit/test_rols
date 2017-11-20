/*************************************************************************
 *
 *  ti_master_list_withTimer.c - Library of routines for readout and buffering of
 *                     events using a JLAB Trigger Interface V3 (TI) with
 *                     a Linux VME controller in CODA 3.0.
 *
 *                     This for a TI in Master Mode controlling multiple ROCs
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1024*64      /* Size in Bytes */

/* Define TI Type (TI_MASTER or TI_SLAVE) */
#define TI_MASTER
/* EXTernal trigger source (e.g. front panel ECL input), POLL for available data */
#define TI_READOUT TI_READOUT_EXT_INT
/* TI VME address, or 0 for Auto Initialize (search for TI by slot) */
#define TI_ADDR  0

/* Measured longest fiber length in system */
#define FIBER_LATENCY_OFFSET 0x4A

int myprimaryttest(int code);
int tiprimaryttest(int code) {return myprimaryttest(code);}

#define tiprimaryttest(x) noprimarytest(x)

#include "dmaBankTools.h"   /* Macros for handling CODA banks */
#include "tiprimary_list.c" /* Source required for CODA readout lists using the TI */

/* Define initial blocklevel and buffering level */
#define BLOCKLEVEL 1
#define BUFFERLEVEL 1

int blockLevel=BLOCKLEVEL;

void
periodicSoftwareTrigger()
{
  tiSoftTrig(1, 1, 1, 0);
}


int
myprimaryttest(int code)
{
  static int firstRun = 1;
  static struct timespec startTime, nowTime;

  if(firstRun)
    {
      clock_gettime(CLOCK_REALTIME, &startTime);
      firstRun = 0;
    }
  else
    {
      clock_gettime(CLOCK_REALTIME, &nowTime);

      if((nowTime.tv_sec - startTime.tv_sec) >= 2)
	{
	  periodicSoftwareTrigger();
	  clock_gettime(CLOCK_REALTIME, &startTime);
	}
    }

  if(dmaPEmpty(vmeOUT))
    return 0;
  else
    return 1;
}

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  int stat;

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,1);

  /* Define BLock Level */
  blockLevel = BLOCKLEVEL;


  /*****************
   *   TI SETUP
   *****************/

  /*
   * Set Trigger source
   *    For the TI-Master, valid sources:
   *      TI_TRIGGER_FPTRG     2  Front Panel "TRG" Input
   *      TI_TRIGGER_TSINPUTS  3  Front Panel "TS" Inputs
   *      TI_TRIGGER_TSREV2    4  Ribbon cable from Legacy TS module
   *      TI_TRIGGER_PULSER    5  TI Internal Pulser (Fixed rate and/or random)
   */
  /* tiSetTriggerSource(TI_TRIGGER_TSINPUTS); /\* TS Inputs enabled *\/ */
  tiSetTriggerSourceMask(TI_TRIGSRC_VME |
			 TI_TRIGSRC_LOOPBACK|
			 TI_TRIGSRC_PULSER |
			 TI_TRIGSRC_TSINPUTS);
			 
  
  /* Enable set specific TS input bits (1-6) */
  tiEnableTSInput( TI_TSINPUT_1 | TI_TSINPUT_2 );

  /* Load the trigger table that associates
   *  pins 21/22 | 23/24 | 25/26 : trigger1
   *  pins 29/30 | 31/32 | 33/34 : trigger2
   */
  tiLoadTriggerTable(0);

  tiSetTriggerHoldoff(1,10,0);
  tiSetTriggerHoldoff(2,10,0);

  /* Set the sync delay width to 0x40*32 = 2.048us */
  tiSetSyncDelayWidth(0x54, 0x40, 1);

  /* Set initial number of events per block */
  tiSetBlockLevel(blockLevel);

  /* Set Trigger Buffer Level */
  tiSetBlockBufferLevel(BUFFERLEVEL);

  tiSetBlockLimit(0);
  
  tiStatus(0);

  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{
  unsigned short iflag;
  int stat;
  int islot;

  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);

  tiStatus(0);

  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{
  int islot;

  /* Get the current Block Level */
  blockLevel = tiGetCurrentBlockLevel();
  printf("rocGo: Block Level set to %d\n",blockLevel);

  /* Enable/Set Block Level on modules, if needed, here */

  /* Example: How to start internal pulser trigger */
#ifdef INTRANDOMPULSER
  /* Enable Random at rate 500kHz/(2^7) = ~3.9kHz */
  tiSetRandomTrigger(1,0xf);
#elif defined (INTFIXEDPULSER)
  /* Enable fixed rate with period (ns) 120 +30*700*(1024^0) = 21.1 us (~47.4 kHz)
     - Generated 1000 times */
  tiSoftTrig(1,1000,700,0);
#endif
}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{
  int islot;

  sleep(1);
  /* Example: How to stop internal pulser trigger */
#ifdef INTRANDOMPULSER
  /* Disable random trigger */
  tiDisableRandomTrigger();
#elif defined (INTFIXEDPULSER)
  /* Disable Fixed Rate trigger */
  tiSoftTrig(1,0,700,0);
#endif
  UEOPEN(129,BT_UI4,5);
  *rol->dabufp++ = 0x12345678;
  *rol->dabufp++ = 0x12345678;
  *rol->dabufp++ = 0x87654321;
  UECLOSE;

  sleep(1);
  
  tiStatus(0);

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int ii, islot;
  int stat, dCnt, len=0, idata;
  unsigned int val;
  unsigned int *start;

  /* Set TI output 1 high for diagnostics */
  tiSetOutputPort(1,0,0,0);

  EVENTOPEN(1,BT_BANK);
  
  /* Readout the trigger block from the TI
     Trigger Block MUST be reaodut first */
  BANKOPEN(4,BT_UI4,blockLevel);
  dCnt = tiReadTriggerBlock(dma_dabufp);

  if(dCnt<=0)
    {
      printf("No TI Trigger data or error.  dCnt = %d\n",dCnt);
    }
  else
    { /* TI Data is already in a bank structure.  Bump the pointer */
      dma_dabufp += dCnt;
    }
  BANKCLOSE;
  
  /* EXAMPLE: How to open a bank (type=5) and add data words by hand */
  BANKOPEN(5,BT_UI4,blockLevel);
  *dma_dabufp++ = tiGetIntCount();
  *dma_dabufp++ = 0xdead;
  *dma_dabufp++ = 0xcebaf111;
  *dma_dabufp++ = 0xcebaf222;
  BANKCLOSE;

  EVENTCLOSE;
  
  /* Set TI output 0 low */
  tiSetOutputPort(0,0,0,0);

}

void
rocCleanup()
{
  int islot=0;

  printf("%s: Reset all Modules\n",__FUNCTION__);

}


/*
  Local Variables:
  compile-command: "make -k -B ti_master_list_withTimer.so"
  End:
 */
