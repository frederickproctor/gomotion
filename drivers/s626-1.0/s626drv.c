//******************************** s626drv.c *************************************
//* This file contains loadable Linux device driver for Sensoray Model 626
//*
//* Autor:		 David Stroupe and Charlie X. Liu
//*
//* Revision:
//*         2002   0.0   David Stroupe    initial
//*    Jul. 2003   0.1   Charlie X. Liu   major number and warning fixes
//*    Mar. 2004   0.2   Charlie X. Liu   re-structured and optimized
//*    Nov. 2004   0.2.1 Charlie X. Liu   Added fix for segmentation fault when
//*                                       calling S626DRV_CloseDMAB()
//*    May, 2005   0.3   Charlie X. Liu   Added support for kernel 2.6
//*    Dec. 2006   1.0   Charlie X. Liu   Added support for using multi-626 boards
//*
//* Copyright (C) 2002-2006	Sensoray Co., Inc.
//********************************************************************************
/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
***************************************************************************/

#if 0	//not needed for Kernel 2.6
#ifndef __KERNEL__
#define __KERNEL__
#endif
#ifndef MODULE
#define MODULE
#endif
#endif

#define DEBUG 1
#undef DEBUG
#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...)
#endif
#define DEBUGI 1
#undef DEBUGI
#ifdef DEBUGI
#define DBGI(x...) printk(x)
#else
#define DBGI(x...)
#endif

// The necessary header files
#include <linux/version.h>
/* FMP */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#include <linux/config.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)

#if defined(CONFIG_MODVERSIONS) && !defined(MODVERSIONS)
#define MODVERSIONS
#endif //defined(CONFIG_MODVERSIONS) && !defined(MODVERSIONS)

#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif //MODVERSIONS

#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)

#include <linux/module.h>			// We're building a module.
#include <linux/init.h>
#include <linux/kernel.h>			// We're doing kernel work.

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
#include <linux/wait.h>                 	// for sleeping and waking up.
#endif

#include <linux/fs.h>				// File operations support.
#include <linux/proc_fs.h>
#include <linux/pci.h>				// Required for pci device.
#include <linux/delay.h>			// Fast delay generators.
#include <linux/sched.h>			// For IRQ.
#include <linux/interrupt.h>			// For 2.6 IRQ
#include <linux/ioctl.h>
#include <asm/uaccess.h>			// For get_user() and put_user().
#include <asm/io.h>
#include <asm/irq.h>

// module params
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#else
#include <linux/moduleparam.h>
#endif

#include "s626.h"
#include "s626drv.h"

MODULE_DESCRIPTION("s626 driver - Sensoray Model 626 loadable Linux device driver");
MODULE_AUTHOR("David Stroupe and Charlie X. Liu");
MODULE_LICENSE("GPL");

/* NIST */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,0)
#define PCI_NAME(x) pci_name(x)
#else
#define PCI_NAME(x) ((x)->slot_name)
#endif

/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////  CONSTANTS  ////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

// Manifest constants:

#define TRUE			1
#define FALSE			0

#define INSTALL_PASS		0			// Module/board installation success code.
#define INSTALL_FAIL		-1			// Module/board installation fail code.

// Default values that are applied in place of missing insmod parameters:
#define DFLT_MAJOR		146			// Major device number.

// for 626:
#define P_IER			0x00DC
#define P_ISR			0x010C
#define P_PSR			0x0110
#define IRQ_MASK		0x00000040
#define DMA_MAX			3			//number of dma buffers allowed - increase if more needed

// Device ID:
static struct pci_device_id S626_id_tbl[] __devinitdata = {
  {0x1131, 0x7146, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
  {0,}
};


MODULE_DEVICE_TABLE (pci, S626_id_tbl);


// Forwarding Prototypes for S626_ methods
static int S626_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg );
static int S626_open( struct inode* inode, struct file* file );
static int S626_release( struct inode* inode, struct file* file );


//////////////////////////////////////////////////////////////////////
// The access methods are placed in a table of valid file operations.  
// This driver supports open, release (close), and ioctl. 
// Use the new gcc extension syntax.
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
static struct file_operations s626_fops = {
  owner:	THIS_MODULE,
  ioctl:	S626_ioctl,
  open:		S626_open,
  release:  	S626_release
};
#else
struct file_operations s626_fops = {
	.owner		= THIS_MODULE,
	//.read		= S626_read,
	//.write	= S626_write,
	.ioctl		= S626_ioctl,
	.open		= S626_open,
	.release	= S626_release
};
#endif


typedef struct
{
	struct pci_dev			*S626_dev;
	DMABUF				*S626_dma[DMA_MAX];
	size_t				dmasize[DMA_MAX];
	DWORD				base;
	wait_queue_head_t		wait_queue;
	int				wait_condition;
	DWORD				busslot;
	BYTE				isregistered;
	BYTE				boardhandle;
	BYTE				dointerrupts;
	volatile DWORD			interruptcounter;
	volatile DWORD			interruptslost;
	char				name[10];
	BYTE				dmaallocations;

} cardinfo;

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  DATA STORAGE  ////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

static cardinfo		board		[MAX_626_BOARDS];
static int		num_boards;					// Board count obtained from insmod parameter count or pci scan.
//static struct pci_dev	*s626_dev	[MAX_626_BOARDS];		// Pointer to pci_dev structures for model 626 boards.
//static void		*log_base	[MAX_626_BOARDS];		// Logical base addresses of application registers.
static int		board_installed	[MAX_626_BOARDS]	= { FALSE, };	// Non-zero indicates that the board is actually in the system and detected.

static char		driverdevice[]				= "s626";
static int		S626_cards_found			= 0;

// Allocate storage for the optional insmod parameters and init them to their default values.
static int		major					= DFLT_MAJOR;	// Major device number.


////////////////////////////////////////  INSMOD PARAMETERS ///////////////////////////////////////
// Declare the insmod parameters and establish linkage to their respective command line parameters.
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
MODULE_PARM		( major,	"h"			);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
// DA: module_param_array changed from 2.6.? to 2.6.11.  See parameters below(value to pointer).
// Have to ifdef for kernel version within the major kernel 2.6 release.
//static int s626devs[2];
module_param		( major, 	int,			0444 );
//module_param_array	( irq, 		int, 	s626devs[0],	0444 );
#else // 2.6.10 or greater
//static int s626devs[2];
module_param		( major, 	int, 			0444 );
//module_param_array	( irq, 		int, 	&s626devs[0],	0444 );
#endif //LINUX_VERSION_CODE

// Forwarding Prototypes:
static void		S626DRV_RegWrite( int addr, int data );
static DWORD		S626DRV_RegRead( int addr );
static void		S626DRV_AllocDMAB( cardinfo *board, DWORD nbytes, DMABUF *pdma );
static void		S626DRV_CloseDMAB( cardinfo *board, DMABUF *pdma );
static int		S626DRV_RequestIRQ( cardinfo *board );


///////////////////////////////////////////////////////////////////////////
// OPEN METHOD - check that the minor device is valid. If not, return
// ENODEV.  Increment usage counter (many processes may use this device
// simultaneously), allocate buffer.

static int S626_open( struct inode *inode, struct file *file )
{
	int brd = MINOR( inode->i_rdev );

	// Check validity of minor (board) number.
	num_boards = S626_cards_found;
	if ( ( (unsigned int)brd > num_boards ) || brd < 0 )
	{
		printk ("Error: invalid minor device number in S626_open().\n");
		return -ENODEV;
	}

	// Check if the board had been opened.
	if ( !board_installed[brd] )
	{
		board_installed[brd] = TRUE;
		// One more app is using this module.
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
		MOD_INC_USE_COUNT;
#endif
		printk ("S626_open(): Board #%i of %i bas been opened.\n", brd, num_boards);
		return 0;
	}
	else
	{
		printk ("Error: device is already open.\n");
		return -ENODEV;
	}
}

////////////////////////////////////////////////////////////////////////////
// CLOSE METHOD - release the device and decrement the usage counters.

static int S626_release( struct inode* inode, struct file* file )
{
	int brd = MINOR( inode->i_rdev );
	int j;	//i,j;

	// Check validity of minor (board) number.
	num_boards = S626_cards_found;
	if ( ( (unsigned int)brd > num_boards ) || brd < 0 )
	{
		printk ("Error: invalid minor device number in S626_release().\n");
		return -ENODEV;
	}

	// Check if the board is opened.
	if ( board_installed[brd] )
	{
		// Return board to its default state.
		//if ( BoardClose( board ) ) {
		  //for (i=0;i<S626_cards_found;i++){
			//clean up all the cards found and make them all unregistered
			//disable interrupt
			#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
			writel( 0, board[brd].base+P_IER );		//make sure interrupts are disabled
			#else
			writel( 0, (volatile void __iomem *) (board[brd].base+P_IER) );
			#endif	//LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
			#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
			writel( IRQ_MASK, (board[brd].base+P_ISR) );
			#else
			writel( IRQ_MASK, (volatile void __iomem *) (board[brd].base+P_ISR) );
			#endif	//LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
			if(board[brd].dointerrupts)
			free_irq (board[brd].S626_dev->irq, &board[brd]);
			board->dointerrupts = FALSE;
			board[brd].wait_condition = KILLINTERRUPT;	//now release any blocked threads
			wake_up_interruptible (&board[brd].wait_queue);
			board[brd].isregistered = FALSE;
			for(j=0;j<DMA_MAX;j++)
			{
				if(board[brd].S626_dma[j]->LogicalBase!=0)
						S626DRV_CloseDMAB(&board[brd],board[brd].S626_dma[j]); 
			}
			board[brd].dmaallocations = 0;    
			board[brd].interruptcounter = 0;
			board[brd].interruptslost = 0;
			board[brd].wait_condition = 0;
		  //}

		board_installed[brd] = FALSE;	// released

		// One less app is using this module.
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
		MOD_DEC_USE_COUNT;
#endif
		printk ("S626_release(): Board #%i bas been released.\n", brd);
	} 
	else 
	{
		printk ("Error: device is not open or is released already.\n");
		return -ENODEV;
	}

	DBG ("Leaving S626_release() \n");
	return 0;

}


static int S626_ioctl (struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int brd = MINOR( inode->i_rdev );
	int i;
	SX_CARD_REGISTER temp;
	cardinfo *card;
	DWORD dwLOC;
	SX_PCI_CARD_INFO temp1;
	SX_PCI_SCAN_CARDS temp2;
	SX_INTERRUPT temp3;
	ioc_param temp4;
	DWORD address;
	DWORD data;
	DMABUF dmabuf;

  switch (cmd)
  {
    case S626_IOC_Register:
		if (copy_from_user( (SX_CARD_REGISTER *) & temp, (SX_CARD_REGISTER *) arg,
							 sizeof (SX_CARD_REGISTER)) ) return -EFAULT;
		for (i = 0; i < S626_cards_found; i++)
		{
		   if (i==brd) 
		   {
			if (!board[i].isregistered && temp.Card.Item[0].I.Mem.dwPhysicalAddr == board[i].base)
				break;
		   }
		}
		printk("ioctl(): S626_IOC_Register: registering board #%i of %i Sensoray Model 626 board(s).\n", 
									i, S626_cards_found);
		if (i == S626_cards_found)
		{			// no matches found
			temp.hCard = 0;
			return copy_to_user ((SX_CARD_REGISTER *) arg,
						 (SX_CARD_REGISTER *) & temp, sizeof (SX_CARD_REGISTER));
		}
		temp.hCard = (DWORD) & board[i];	//we need to know which card in the system we are talking to
		board[i].isregistered = TRUE;
		return copy_to_user ((SX_CARD_REGISTER *) arg,
					 (SX_CARD_REGISTER *) & temp, sizeof (SX_CARD_REGISTER));
		break;
    case S626_IOC_Unregister:
		if (copy_from_user( (SX_CARD_REGISTER *) & temp, (SX_CARD_REGISTER *) arg,
							 sizeof (SX_CARD_REGISTER)) ) return -EFAULT;
		card = (cardinfo *) temp.hCard;
		if (!card) return -EFAULT;
		card->isregistered = FALSE;
		if(card->dointerrupts)
		{
			  #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
			  writel( 0, card->base+P_IER );	//make sure interrupts are disabled
			  #else
			  writel( 0, (volatile void __iomem *) (card->base+P_IER) );
			  #endif
			  #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
			  writel( IRQ_MASK, card->base+P_ISR );
			  #else
			  writel( IRQ_MASK, (volatile void __iomem *) (card->base+P_ISR) );
			  #endif
			  free_irq (card->S626_dev->irq, card);
			  card->dointerrupts = FALSE;//tell the driver not to worry about this irq when unloaded
		}
		temp.hCard = 0;
		return copy_to_user ((SX_CARD_REGISTER *) arg,
					 (SX_CARD_REGISTER *) & temp, sizeof (SX_CARD_REGISTER));
		break;
    case S626_IOC_GetCardInfo:
		if (copy_from_user( (SX_PCI_CARD_INFO *) & temp1, (SX_PCI_CARD_INFO *) arg,
							 sizeof (SX_PCI_CARD_INFO)) ) return -EFAULT;
		dwLOC = temp1.pciSlot.dwBus << 16 | temp1.pciSlot.dwSlot;
		//printk("ioctl(): S626_IOC_GetCardInfo: board=%i, dwLOC=%ld\n", brd, dwLOC);
		for (i = 0; i < S626_cards_found; i++)
		{
		   if (i==brd) 
		   {
			//printk("ioctl(): S626_IOC_GetCardInfo: board[%i].base=ox%lx, board[%i].busslot=%ld\n", 
			//						i, board[i].base, i, board[i].busslot);
			if (dwLOC == board[i].busslot)
			{			//found the requested board
				temp1.dwDevNum = i;
				temp1.Card.dwItems = 2;
				temp1.Card.Item[0].item = ITEM_MEMORY;
				temp1.Card.Item[0].I.Mem.dwPhysicalAddr = board[i].base;
				temp1.Card.Item[0].I.Mem.dwUserDirectAddr = board[i].base;
				temp1.Card.Item[1].item = ITEM_INTERRUPT;
				temp1.Card.Item[1].I.Int.dwInterrupt = board[i].S626_dev->irq;
				temp1.Card.Item[1].I.Int.hInterrupt = (DWORD) & board[i];
				return copy_to_user( (SX_PCI_CARD_INFO *) arg, (SX_PCI_CARD_INFO *) & temp1,
						 				sizeof (SX_PCI_CARD_INFO) );
			}
		   }
		}
		return -1;		// no board found that matches
		break;
    case S626_IOC_Init:
		if (copy_from_user( (SX_PCI_SCAN_CARDS *) & temp2, (SX_PCI_SCAN_CARDS *) arg,
							  sizeof (SX_PCI_SCAN_CARDS)) ) return -EFAULT;
		temp2.dwCards = 0;
		for (i = 0; i < S626_cards_found; i++)
		{
		   if (i==brd) 
		   {
			temp2.cardSlot[i].dwBus = board[i].busslot >> 16;
			temp2.cardSlot[i].dwSlot = board[i].busslot & 0xFFFF;
			temp2.cardId[i].dwVendorId = 0x1131;	//we know that this is true
			temp2.cardId[i].dwDeviceId = 0x7146;
			//temp2.dwCards++;
			temp2.dwCards = i+1;
		   }
		}
		return copy_to_user ((SX_PCI_SCAN_CARDS *) arg, (SX_PCI_SCAN_CARDS *) & temp2,
								 sizeof (SX_PCI_SCAN_CARDS));
		break;
    case S626_IOC_Close:
		//kill interrupts
		break;
    case S626_IOC_RequestIrq:
		if (copy_from_user( (SX_INTERRUPT *) & temp3, (SX_INTERRUPT *) arg,
						     sizeof (SX_INTERRUPT)) ) return -EFAULT;
		card = (cardinfo *) temp3.hInterrupt;
		if (!card)		//the board pointer was not setup correctly
			return -EFAULT;
		if(!S626DRV_RequestIRQ (card))
		{
			card->dointerrupts = TRUE;
			DBG("ioctl(): S626_IOC_RequestIrq: successful!\n");
		}
		break;
    case S626_IOC_InterruptOff:
 		if (copy_from_user( (SX_INTERRUPT *) & temp3, (SX_INTERRUPT *) arg,
						     sizeof (SX_INTERRUPT)) ) return -EFAULT;
		card = (cardinfo *) temp3.hInterrupt;
		card->wait_condition = KILLINTERRUPT;
		DBG("S626_IOC_InterruptOff: card->wait_queue = 0x%lx\n", card->wait_queue);
		wake_up_interruptible (&card->wait_queue);
		return 0;
		break;
    case S626_IOC_InterruptOn:
		if (copy_from_user( (SX_INTERRUPT *) & temp3, (SX_INTERRUPT *) arg,
						     sizeof (SX_INTERRUPT))) return -EFAULT;
		card = (cardinfo *) temp3.hInterrupt;
		if(!card) return -EFAULT;
		//DBG("Driver waiting for irq #%d board %s\n", card->S626_dev->irq, card->name);
		DBG("S626_IOC_InterruptOn: Driver waiting for irq for #%d board %s\n", card->S626_dev->irq, card->name);
		if (!wait_event_interruptible( card->wait_queue, 
			((card->wait_condition == GOTINTERRUPT) || card->wait_condition == KILLINTERRUPT)) )
		{
			if(card->wait_condition == KILLINTERRUPT)
			{
				DBG("Driver wait killed for board %s\n", card->name);
 				temp3.fStopped = TRUE;
			} else {
				DBG("Driver interrupted waiting for board %s\n", card->name);
 				card->interruptslost--;
			}
			temp3.dwLost = card->interruptslost;
			temp3.dwCounter = card->interruptcounter;
 			card->wait_condition = 0;
			return copy_to_user( (SX_INTERRUPT *) arg, (SX_INTERRUPT *) & temp3,
								    sizeof (SX_INTERRUPT));
		}
		card->wait_condition = 0;
		break;
     case S626_IOC_WriteRegister:
		if (copy_from_user( (ioc_param *) & temp4, (ioc_param *) arg, 
						  sizeof (ioc_param))) return -EFAULT;
		DBG ("driver got addy=0x%x data=0x%x\n", temp4.address, temp4.data);
		S626DRV_RegWrite (temp4.address, temp4.data);
		return 0;
		break;
    case S626_IOC_ReadRegister:
		if (get_user( address, (DWORD *) arg) ) return -EFAULT;
		data = S626DRV_RegRead (address);
		DBG ("driver read data=0x%x from addy=0x%x\n", data, address);
		return put_user (data, (DWORD *) arg);
		break;
    case S626_IOC_AllocDMAB:

		/* Added on 2004-11-15, initialize dmabuf */
		dmabuf.DMAHandle = 0;
		dmabuf.LogicalBase = NULL;
		dmabuf.PhysicalBase = NULL;
		/* End added on 2004-11-15 */

		if (copy_from_user( (ioc_param *) & temp4, (ioc_param *) arg, 
						  sizeof (ioc_param))) return -EFAULT;
		S626DRV_AllocDMAB ((cardinfo *) temp4.address, temp4.data, &dmabuf);
		temp4.PhysicalBase = dmabuf.PhysicalBase;
		temp4.LogicalBase = dmabuf.LogicalBase;
		temp4.DMAHandle = dmabuf.DMAHandle;
		DBG ("Allocated %d bytes at 0x%x from 0x%x\n", temp4.data,
					 temp4.LogicalBase, temp4.PhysicalBase);
		return copy_to_user ((ioc_param *) arg, (ioc_param *) & temp4,
							 sizeof (ioc_param));
		break;
    case S626_IOC_CloseDMAB:
		if (copy_from_user( (ioc_param *) & temp4, (ioc_param *) arg, 
						  sizeof (ioc_param))) return -EFAULT;
		dmabuf.PhysicalBase = temp4.PhysicalBase;
		dmabuf.LogicalBase = temp4.LogicalBase;
		dmabuf.DMAHandle = temp4.DMAHandle;
		S626DRV_CloseDMAB ((cardinfo *) temp4.address, &dmabuf);
		DBG ("Deallocated at 0x%x from 0x%x\n", (unsigned int) temp4.LogicalBase,
							(unsigned int) temp4.PhysicalBase);
		return copy_to_user ((ioc_param *) arg, (ioc_param *) & temp4,
							 sizeof (ioc_param));
		break;
    default:
		break;
  }
  
  DBG ("Leaving S626_ioctl() \n");

  return 0;

}

DWORD convert (char *p)
{
  int low, high;
  for (high = 0;; p++)
    {
      if (*p >= '0' && *p <= '9')
	high = (high << 4) + (*p - '0');
      else if (*p >= 'A' && *p <= 'F')
	high = (high << 4) + (*p - 'A' + 10);
      else if (*p >= 'a' && *p <= 'f')
	high = (high << 4) + (*p - 'a' + 10);
      else
	break;
    }

  p++;				//skip over the : character

  for (low = 0;; p++)
    {
      if (*p >= '0' && *p <= '9')
	low = (low << 4) + (*p - '0');
      else if (*p >= 'A' && *p <= 'F')
	low = (low << 4) + (*p - 'A' + 10);
      else if (*p >= 'a' && *p <= 'f')
	low = (low << 4) + (*p - 'a' + 10);
      else
	break;
    }

  return high << 16 | low;
}


static int __devinit
S626_init_one (struct pci_dev *pdev, const struct pci_device_id *ent)
{
  int retval = 0;
  int i;
  /* NIST */
  char print_name[sizeof (PCI_NAME(pdev))];
  void *base;
  unsigned int io_addr;
  unsigned int io_size;
  struct resource *request_response;
  
  DBG ("Entering S626_init_one\n");

  /* wake up and enable device */
  if (pci_enable_device (pdev))
    {
      retval = -EIO;
    }
  else
    {
      board[S626_cards_found].S626_dev = kmalloc (sizeof (*pdev), GFP_ATOMIC);
      for(i=0;i<DMA_MAX;i++)
      {
	board[S626_cards_found].S626_dma[i] =
	  kmalloc (sizeof (DMABUF), GFP_ATOMIC);
         /* Initialize with NULL, added on 2004-11-15 */
         board[S626_cards_found].S626_dma[i]->DMAHandle = 0;
         board[S626_cards_found].S626_dma[i]->LogicalBase = NULL;
         board[S626_cards_found].S626_dma[i]->PhysicalBase = NULL;
         /* End initialize, added on 2004-11-15 */ 
      }
      memcpy (board[S626_cards_found].S626_dev, pdev, sizeof (*pdev));
      io_addr = pci_resource_start (pdev, 0);
      io_size = pci_resource_len (pdev, 0);
      //memcpy( print_name, pdev->slot_name, sizeof(pdev->slot_name) );
      //printk("S626: Detected S626 Card - Address=0x%x io_size=0x%x irq=%d Slot=%s\n", io_addr, io_size,
      printk("S626_init_one(): Detected S626 Card - Address=0x%x io_size=0x%x irq=%d Slot=%s\n", io_addr, io_size,
	     /* NIST */
	     pdev->irq, PCI_NAME(pdev));		//print_name);
      //request_response = request_mem_region (io_addr, io_size, print_name);
      request_response = request_mem_region( io_addr, io_size, "s626" );
      if (request_response == 0)
	{
	  DBG ("Memory Region request failed\n");
	  retval = -EIO;
	  goto end;
	}
      base = ioremap (io_addr, io_size);
      if (base == 0)
	{
	  DBG ("IOREMAP failed\n");
	  retval = -EIO;
	  goto end;
	}
      board[S626_cards_found].base = (DWORD) base;
      //bussslot = (BusNum << 16) | SlotNum
      board[S626_cards_found].busslot = convert (print_name);
      sprintf (board[S626_cards_found].name, "S626 #%d", S626_cards_found);
    }
  //printk("Card %s added.\n",board[S626_cards_found].name);
  printk("S626_init_one():Card %s added. Board[%i].base=0x%lx\n", board[S626_cards_found].name, S626_cards_found, board[S626_cards_found].base);
  
  S626_cards_found++;
  
end:

  return retval;
  
}

static void __devexit
S626_remove_one (struct pci_dev *pdev)
{
  int i, j;
  int remove = -1;
  unsigned int io_addr;
  unsigned int io_length;
  
  DBG ("Entering S626_remove_one\n");

  for (i = 0; i < MAX_626_BOARDS; i++)
    if (board[i].S626_dev)
      {
	/* NIST */
	DBG ("request to remove board in slot %s current board slot %s %d\n",
	     PCI_NAME(pdev), PCI_NAME(board[i].S626_dev), i);
	if (strcmp (PCI_NAME(pdev), PCI_NAME(board[i].S626_dev)) == 0)
	  {
	    DBG ("MATCH!!!\n");
	    remove = i;
	    break;
	  }
      }
  if (remove == -1)
    {
      /* NIST */
      DBG ("BAD REQUEST for Card Removal Slot %s\n", PCI_NAME(pdev));
      return;
    }
  S626_cards_found--;
  printk("Card %s removed.\n", board[S626_cards_found].name);
  DBG ("remove=%d S626_dev=0x%x\n", remove,
       (unsigned int) board[remove].S626_dev);
  if(board[remove].dointerrupts)
  {
    //this board had a handler attached
    //disable board interrupt then remove handler
    DBG("removing irq %d dev_id 0x%x\n", board[remove].S626_dev->irq, (int) &board[remove]);
    #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
    writel( 0, board[remove].base+P_IER );
    #else
    writel( 0, (volatile void __iomem *) (board[remove].base+P_IER) );
    #endif
    #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
    writel( IRQ_MASK, board[remove].base+P_ISR );
    #else
    writel( IRQ_MASK, (volatile void __iomem *) (board[remove].base+P_ISR) );
    #endif
    free_irq (board[remove].S626_dev->irq, &board[remove]);
    board[remove].dointerrupts=FALSE;
  }
  iounmap ((void *) board[remove].base);
  io_addr = pci_resource_start (board[remove].S626_dev, 0);
  io_length = pci_resource_len (board[remove].S626_dev, 0);
  release_mem_region (io_addr, io_length);
  kfree (board[remove].S626_dev);
  board[remove].S626_dev = 0;
  for(j=0;j<DMA_MAX;j++)
  {
    kfree(board[remove].S626_dma[j]);
    board[remove].S626_dma[j] = 0;
  }

};

static struct pci_driver S626_driver = {
  name:"S626",
  probe:S626_init_one,
  remove:S626_remove_one,
  id_table:S626_id_tbl,
};


static int __init
S626_init (void)
{
  int i, j;
  int retval = 0;

  DBG ("Entering S626 Module Init\n");
  for (i = 0; i < MAX_626_BOARDS; i++)
  {
      board[i].S626_dev = 0;
      init_waitqueue_head (&board[i].wait_queue);
      board[i].wait_condition = 0;
      board[i].dointerrupts = FALSE;
      board[i].isregistered = FALSE;
      board[i].dmaallocations = 0;
      board[i].interruptcounter = 0;
      board[i].interruptslost = 0;
      for(j = 0;j < DMA_MAX;j++)
      {
	board[i].dmasize[j] = 0;
	board[i].S626_dma[j] = 0;
      }
  }

	// Attempt to register the device with the kernel, 
	// and if unsuccessful, report the problem and terminate the driver.
	//major = register_chrdev (0, driverdevice, &s626_fops);
	//if ( ( retval=register_chrdev( major, driverdevice, &s626_fops ) ) < 0  )
	retval = register_chrdev( major, driverdevice, &s626_fops );
	if ( retval )
	{
		printk( "Unable to get major number %i for 626 ( retval = %i ).\n", major, retval );
		return INSTALL_FAIL;
	}
	printk("S626: %s installed - device major number = %d\n", driverdevice, major);

	// return pci_module_init (&S626_driver);
	return pci_register_driver(&S626_driver);
    
  DBG ("Leaving S626 Module Init\n");
  //return 0;
  return INSTALL_PASS;	// 0;

}


static void __exit
S626_cleanup (void)
{
  DBG ("Entering S626 Module Cleanup\n");
  pci_unregister_driver (&S626_driver);
  unregister_chrdev (major, driverdevice);
}

static void S626DRV_AllocDMAB (cardinfo * board, DWORD nbytes, DMABUF * pdma)
{
  size_t bsize;
  dma_addr_t pha;

  DBG ("Entering S626DRV_AllocDMAB():\n");
  bsize = (size_t) nbytes;


  pdma->LogicalBase = pci_alloc_consistent (board->S626_dev, bsize, &pha);

  if (pdma->LogicalBase == NULL)
    {
      printk ("DMA Memory mapping error\n");
      //return ERR_ALLOC_MEMORY;
    }

  pdma->PhysicalBase = (void *) pha;
  board->S626_dma[board->dmaallocations]->LogicalBase = pdma->LogicalBase;
  board->S626_dma[board->dmaallocations]->PhysicalBase = pdma->PhysicalBase;
  board->dmasize[board->dmaallocations] = bsize;
  DBG("S626DRV_AllocDMAB(): Logical=0x%x, bsize=%d, Physical=0x%x\n",
       (DWORD) pdma->LogicalBase, bsize, (DWORD) pdma->PhysicalBase);
  board->dmaallocations++;

}

static void S626DRV_CloseDMAB (cardinfo * board, DMABUF * pdma)
{
  size_t bsize;
  void *vbptr, *vpptr;
  int i;

  DBG ("Entering S626DRV_CloseDMAB():\n");
  if (pdma == NULL)
    return;
  //find the matching allocation from the board struct
  for(i=0;i<DMA_MAX;i++){
    if(board->S626_dma[i]->LogicalBase == pdma->LogicalBase)
      break;
  }
  if(i==DMA_MAX){
    printk("Can not deallocate\n");
    return;
  }
  bsize = board->dmasize[i];
  vbptr = board->S626_dma[i]->LogicalBase;
  vpptr = board->S626_dma[i]->PhysicalBase;
  //if (vbptr)
  if ((vbptr) && (0!=bsize))
  {
      pci_free_consistent (board->S626_dev, bsize, vbptr,
			  (int) vpptr);
      pdma->LogicalBase = 0;
      pdma->PhysicalBase = 0;
      board->S626_dma[i]->LogicalBase = 0;
      board->S626_dma[i]->PhysicalBase = 0;
      board->dmasize[i] = 0;
      DBG ("S626DRV_CloseDMAB(): Logical=0x%x, bsize=%d, Physical=0x%x\n",
	   (DWORD) vbptr, bsize, (DWORD) vpptr);
  }

}

static DWORD S626DRV_RegRead (int addr)
{
	#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
	return readl( addr );
	#else
	return readl( (volatile void __iomem *) addr );
	#endif
}

static void S626DRV_RegWrite (int addr, int data)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
	writel( data, addr );
#else
	writel( data, (volatile void __iomem *) addr );
#endif
}

///////////////////////////////////////////////////////////////////////////////////
// Interrupt handler.
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
static void handleallinterrupts( int irq, void *dev_id, struct pt_regs *regs)
#else
irqreturn_t handleallinterrupts( int irq, void *dev_id, struct pt_regs *regs )
#endif
{
  int i;
  DWORD irqstatus;
  cardinfo *info = dev_id;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#else
  int handled = 1;
#endif
  
  if (dev_id == 0)
  {
      DBGI("Bad Boy: Got interrupt # %d with a NULL dev_id\n", irq);
      goto end;
  }
  for (i = 0; i < MAX_626_BOARDS; i++)
  {
      if ((unsigned long) &board[i] == (unsigned long) dev_id)
	{
	  //The first thing to do is to determine if this S626 created this interrupt.
	  #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
	  irqstatus = readl( info->base + P_PSR );
	  #else
	  irqstatus = readl( (volatile void __iomem *) (info->base + P_PSR) );
	  #endif
	  if ((irqstatus & IRQ_MASK) == 0)
	    {
	      // this is not our interrupt
	      DBGI ("irqstatus 0x%x board base 0x%x board number%d\n",
		   irqstatus & IRQ_MASK, info->base, i);
	      goto end;
	    }
	  //This is our interrupt so clear the board's master interrupt enable bit.
	  DBGI("getting interrupt# 0x%x missed 0x%x\n", info->interruptcounter, info->interruptslost);
	  #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
	  writel( 0, info->base+P_IER );
	  #else
	  writel( 0, (volatile void __iomem *) (info->base+P_IER) );
	  #endif
	  #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
	  writel( IRQ_MASK, info->base+P_ISR );
	  #else
	  writel( IRQ_MASK, (volatile void __iomem *) (info->base+P_ISR) );
	  #endif
	  //increment the counters
	  info->interruptcounter++;
	  info->interruptslost++;
	  //now unblock the interrupt handler thread in user space
	  info->wait_condition = GOTINTERRUPT;
	  wake_up_interruptible (&info->wait_queue);
	  DBGI("Set board[%d] condition to 0x%x because of interrupt # %d\n",
	       i, info->wait_condition, irq);
	  goto end;
	}
  }
end:
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
  return;
#else
  return IRQ_RETVAL( handled );
#endif
}

static int S626DRV_RequestIRQ (cardinfo * board)
{
  unsigned int irq = board->S626_dev->irq;
  void *dev_id = board;		//pointer to the boards information
  /* FMP */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
  unsigned long flags = SA_SHIRQ;
#else
  unsigned long flags = IRQF_SHARED;
#endif
  int request = -1;
  //disable the interrupt on this board before we turn on the handler
  #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
  writel( 0, board->base+P_IER );
  #else
  writel( 0, (volatile void __iomem *) (board->base+P_IER) );
  #endif
  #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
  writel( IRQ_MASK, board->base+P_ISR );
  #else
  writel( IRQ_MASK, (volatile void __iomem *) (board->base+P_ISR) );
  #endif
  request = request_irq (irq, handleallinterrupts, flags, board->name, dev_id);
  if (request)
  {
      DBG ("did not get interrupt %d\n", irq);	//failed
  } else {
      DBG ("successfully requested handler for irq %d, id=0x%x\n", irq, dev_id);
      printk ("ioctl(): S626DRV_RequestIRQ: successfully requested handler for irq %d, id=0x%lx\n", irq, (long unsigned int) dev_id);
  }

  return request;

}

module_init (S626_init);
module_exit (S626_cleanup);
