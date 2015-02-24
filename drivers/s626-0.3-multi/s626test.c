#include "s626drv.h"
#include "App626.h"
#include "s626mod.h"
#include "s626core.h"
#include "s626api.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <pthread.h>

static pthread_mutex_t CriticalSection = PTHREAD_MUTEX_INITIALIZER;

static void InterruptAppISR(DWORD board)
{
  static int cnt1 = 0;
  static WORD cntr_chan = CNTR_2B;
  static DWORD IntCounts[16] = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 };
  time_t time_now;
  WORD intStatus[4];
  register WORD status;
  register DWORD *pCounts;
  register WORD mask;

  S626_InterruptStatus(board, intStatus);	/* Fetch IRQ status for all sources. */
  /*printf("\tintStatus = 0x%x 0x%x 0x%x 0x%x\n", intStatus[0], intStatus[1], intStatus[2], intStatus[3]); */

  /*if (intStatus[3]==0x4000) {  // interrupt from Counter 2A overflow */
  if (intStatus[3] & 0xfc00) {	/* interrupts from all Counter's overflow */
    cnt1++;
    if (cnt1 % 10 == 0) {
      /*printf("\tintStatus = 0x%x 0x%x 0x%x 0x%x\n", intStatus[0], intStatus[1], intStatus[2], intStatus[3]); */
      /*printf ("%6d - %10d\n", cnt1, time (0)); */
      time(&time_now);
      /*printf ("\tinterrupt # %d ( from Counter-2A ), occured at %s \n", cnt1, ctime(&time_now)); */
      printf("\tinterrupt # %d ( from Counter-%d ), occured at %s \n", cnt1,
	     cntr_chan, ctime(&time_now));
    }
    S626_CounterCapFlagsReset(board, cntr_chan);	/*reset counter interrupts */

    /* Unmask board’s master interrupt enable. */
    S626_InterruptEnable(board, TRUE);	/*Re-enable interrupts */
  }

  if (intStatus[0]) {		/* interrupts from DIO channel 0-15 */
    /* Cache a copy of DIO channel 0-15 interrupt request (IRQ) status. */
    status = intStatus[0];	/* Cache DIO 0-15 IRQ status. */

    /* Tally DIO 0-15 interrupts. */
    pCounts = IntCounts;	/* Init pointer to interrupt counter. */

    pthread_mutex_lock(&CriticalSection);	/* * Start thread-safe section ----------- */
    for (mask = 1; mask != 0; pCounts++) {	/* * */
      if (status & mask)	/* * If DIO is requesting service ... */
	(*pCounts)++;		/* * increment DIO’s interrupt counter. */
      mask += mask;		/* * Bump mask. */
    }				/* * */
    pthread_mutex_unlock(&CriticalSection);	/* * End thread-safe section ------------- */

    /* Negate all processed DIO interrupt requests. */
    S626_DIOCapReset(board, 0, status);	/* group #0: DIO 0-15 */

    /* Unmask board’s master interrupt enable. */
    S626_InterruptEnable(board, TRUE);	/*Re-enable interrupts */
  }
}

/****** print error code */

VOID ErrorFunction1(DWORD ErrFlags)
{
  printf("Got an error on board 1 0x%x\n", ErrFlags);
}

VOID CreateEncoderCounter( HBD hbd, WORD Counter )
{
  /* Set operating mode for the given Counter. */
  S626_CounterModeSet( hbd, Counter,
		       ( LOADSRC_INDX << BF_LOADSRC ) |    /* Preload upon index. */
		       ( INDXSRC_SOFT << BF_INDXSRC ) |    /* Disable hardware index. */
		       ( CLKSRC_COUNTER << BF_CLKSRC ) |   /* Operating mode is Counter. */
		       ( CLKPOL_POS  << BF_CLKPOL ) |      /* Active high clock. */
		       /*( CNTDIR_UP  << BF_CLKPOL ) |       // Count direction is Down. */
		       ( CLKMULT_1X   << BF_CLKMULT ) |    /* Clock multiplier is 1x. */
		       ( CLKENAB_INDEX << BF_CLKENAB ) );  /* Counting is initially disabled. */

  /* Set counter core and preload value to be mid of 2^24 (since all counters are 24-bit), so as to make test easy. */
  S626_CounterPreload( hbd, Counter, 8388608 );	/* 0x800000 = 2^24/2 = 8388608 */
  S626_CounterSoftIndex( hbd, Counter );			/* Generate a index signal by software,  */
  /* so that the preload value got loaded when creating EncoderCounter. */

  /* Enable latching of accumulated counts on demand. */
  S626_CounterLatchSourceSet( hbd, Counter, LATCHSRC_AB_READ );

  /* Enable the counter. */
  S626_CounterEnableSet( hbd, Counter, CLKENAB_ALWAYS );
}

int main(int argc, char *argv[])
{
  unsigned int board = 0;	/* first 626 board #0 */
  unsigned long errFlags  = 0x0;
  enum {BUFFERLEN = 80};
  char buffer[BUFFERLEN];
  char * ptr;
  double volts;
  WORD Counter = CNTR_0A;
  int raw;
  int pos;

  S626_OpenBoard(board, 0, InterruptAppISR, 1);
  S626_InterruptEnable(board, FALSE);
  S626_SetErrCallback(board, ErrorFunction1);
  errFlags = S626_GetErrors(board);

  if (errFlags != 0x0) {
    printf("Board open/installation with Err = 0x%x : ");
    switch (errFlags) {
    case ERR_OPEN:
      printf("\t\t Can't open driver.\n");
      break;
    case ERR_CARDREG:
      printf("\t\t Can't attach driver to board.\n");
      break;
    case ERR_ALLOC_MEMORY:
      printf("\t\t Memory allocation error.\n");
      break;
    case ERR_LOCK_BUFFER:
      printf("\t\t Error locking DMA buffer.\n");
      break;
    case ERR_THREAD:
      printf("\t\t Error starting a thread.\n");
      break;
    case ERR_INTERRUPT:
      printf("\t\t Error enabling interrupt.\n");
      break;
    case ERR_LOST_IRQ:
      printf("\t\t Missed interrupt.\n");
      break;
    case ERR_INIT:
      printf("\t\t Board object not instantiated.\n");
      break;
    case ERR_SUBIDS:
      printf("\t\t PCI SubDevice/SubVendor ID mismatch.\n");
      break;
    case ERR_CFGDUMP:
      printf("\t\t PCI configuration dump failed.\n");
      break;
    default:
      printf("\t\t other Unknown errors.\n");
      break;
    }
    printf("Fix board open/installation error, and then try again. \n\n");
    return -1;
  }

  printf("Board 0 is on PCI bus %d; slot %d\n\n",
	 S626_GetAddress(0) >> 16, S626_GetAddress(0) & 0xffff);

  S626_CounterCapFlagsReset(board, Counter);
  CreateEncoderCounter(board, Counter);

  /* main continuous loop. */
  while (! feof(stdin)) {
    printf("> ");
    fflush(stdout);
    if (NULL == fgets(buffer, BUFFERLEN, stdin)) break;
    ptr = buffer;
    while (isspace(*ptr)) ptr++;
    if (*ptr == 0) {
      pos = S626_CounterReadLatch(board, Counter) - 8388608;
      printf("%d\n", pos);
    } else if (1 == sscanf(ptr, "%lf", &volts)) {
      if (volts < -10.0) volts = -10.0;
      else if (volts > 10.0) volts = 10.0;
      /* 819.1 counts per volt, 8191 for 10 V */
      raw = (int) (volts * 819.1);
      S626_WriteDAC(board, 0, raw);
    } else if (*ptr == 'q') {
      break;
    } else {
      printf("?\n");
    }
  }

  S626_WriteDAC(board, 0, 0);

  S626_CloseBoard(board);

  return 0;
}
