/*
 * twim.c
 *
 * Created: 3/9/2016 12:57:21 AM
 *  Author: Josh
 */ 

//#include "twi.h"
#include "drivers/twi.h"
#include "drivers/twim.h"
#include "utils/status_codes.h"
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/atomic.h>


/* Master Transfer Descriptor */
static struct
{
	TWI_t *         bus;            // Bus register interface
	twi_package_t * pkg;            // Bus message descriptor
	int             addr_count;     // Bus transfer address data counter
	unsigned int    data_count;     // Bus transfer payload data counter
	bool            read;           // Bus transfer direction
	bool            locked;         // Bus busy or unavailable
	volatile status_code_t status;  // Transfer status

} transfer;


/**
 * \internal
 *
 * \brief TWI Master Interrupt Vectors
 *
 * The TWI master interrupt request entry points are conditionally compiled
 * for the TWI interfaces supported by the XMEGA MCU variant.  All of these
 * entry points call a common service function, twim_interrupt_handler(),
 * to handle bus events.  This handler uses the bus interface and message
 * parameters specified in the global \c transfer structure.
 */
static void twim_interrupt_handler(void);

#ifdef TWIC
ISR(TWIC_TWIM_vect) { twim_interrupt_handler(); }
#endif
#ifdef TWID
ISR(TWID_TWIM_vect) { twim_interrupt_handler(); }
#endif
#ifdef TWIE
ISR(TWIE_TWIM_vect) { twim_interrupt_handler(); }
#endif
#ifdef TWIF
ISR(TWIF_TWIM_vect) { twim_interrupt_handler(); }
#endif

/**
 * \internal
 *
 * \brief Test for an idle bus state.
 *
 *  Software can determine the TWI master bus state (unknown, idle, owner, or
 *  busy) by reading the bus master status register:
 *
 *          TWI_MASTER_BUSSTATE_UNKNOWN_gc Bus state is unknown.
 *          TWI_MASTER_BUSSTATE_IDLE_gc    Bus state is idle.
 *          TWI_MASTER_BUSSTATE_OWNER_gc   Bus state is owned by the master.
 *          TWI_MASTER_BUSSTATE_BUSY_gc    Bus state is busy.
 *
 * \param   twi     Base address of the TWI (i.e. &TWI_t).
 *
 * \retval  true    The bus is currently idle.
 * \retval  false   The bus is currently busy.
 */
static inline bool twim_idle (const TWI_t * twi)
{

	return ((twi->MASTER.STATUS & TWI_MASTER_BUSSTATE_gm)
			== TWI_MASTER_BUSSTATE_IDLE_gc);
}

typedef uint8_t irqflags_t;

/**
 * \internal
 *
 * \brief Get exclusive access to global TWI resources.
 *
 * Wait to acquire bus hardware interface and ISR variables.
 *
 * \param no_wait  Set \c true to return instead of doing busy-wait (spin-lock).
 *
 * \return STATUS_OK if the bus is acquired, else ERR_BUSY.
 */
static inline status_code_t twim_acquire(bool no_wait)
{
	/*while (transfer.locked) {

		if (no_wait) { return ERR_BUSY; }
	}*/
	bool twi_bus_busy = true;
	bool proceed = false;
	
	/* if user wants this to be a non-blocking operation */
	while (proceed == false) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
		{
			/* if transfer bus is not locked down. */
			if (transfer.locked == false) {
				twi_bus_busy = false;
			
				/* change the struct fields */
				transfer.locked = true;
				transfer.status = OPERATION_IN_PROGRESS;
				
				/* able to proceed */
				proceed = true;
			
			/* transfer bus is busy */
			} else {
				twi_bus_busy = true;
				
				/* able to proceed */
				proceed = true;
			}
			
		}
		
		/* if user requested non-blocking op, proceed with current bus state even if bus is busy.
		This is done outside of the atomic block in case user does request actually a blocking operation,
		the only way the transfer.locked bit can change is if a TWI interrupt occurs that allows some
		transmission to finish... since our twi code is interrupt driven. */
		if (no_wait) {
			proceed = true;
		}
	}
		
			
	/* if bus *was* not busy, then everything when ok. Nothing to report really. */
	if (!twi_bus_busy) {
		return STATUS_OK;
		
	/* bus *was* busy. report this to the calling user. */
	} else {
		return ERR_BUSY;
	}
}

/**
 * \internal
 *
 * \brief Release exclusive access to global TWI resources.
 *
 * Release bus hardware interface and ISR variables previously locked by
 * a call to \ref twim_acquire().  This function will busy-wait for
 * pending driver operations to complete.
 *
 * \return  status_code_t
 *      - STATUS_OK if the transfer completes
 *      - ERR_BUSY to indicate an unavailable bus
 *      - ERR_IO_ERROR to indicate a bus transaction error
 *      - ERR_NO_MEMORY to indicate buffer errors
 *      - ERR_PROTOCOL to indicate an unexpected bus state
 */
static inline status_code_t twim_release(void)
{
	/* First wait for the driver event handler to indicate something
	 * other than a transfer in-progress, then test the bus interface
	 * for an Idle bus state.
	 */
	while (OPERATION_IN_PROGRESS == transfer.status);

	while (! twim_idle(transfer.bus)) { _MemoryBarrier(); }

	status_code_t const status = transfer.status;

	transfer.locked = false;

	return status;
}

/**
 * \internal
 *
 * \brief TWI master write interrupt handler.
 *
 *  Handles TWI transactions (master write) and responses to (N)ACK.
 */
static inline void twim_write_handler(void)
{
	TWI_t * const         bus = transfer.bus;
	twi_package_t * const pkg = transfer.pkg;
	

	if (transfer.addr_count < pkg->addr_length) {

		const uint8_t * const data = pkg->addr;
		bus->MASTER.DATA = data[transfer.addr_count++];

	} else if (transfer.data_count < pkg->length) {

		if (transfer.read) {

			/* Send repeated START condition (Address|R/W=1). */

			bus->MASTER.ADDR |= 0x01;

		} else {
			const uint8_t * const data = pkg->buffer;
			bus->MASTER.DATA = data[transfer.data_count++];
		}

	} else {

		/* Send STOP condition to complete the transaction. */

		bus->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		transfer.status = STATUS_OK;
	}
}

/**
 * \internal
 *
 * \brief TWI master read interrupt handler.
 *
 *  This is the master read interrupt handler that takes care of
 *  reading bytes from the TWI slave.
 */
static inline void twim_read_handler(void)
{
	TWI_t * const         bus = transfer.bus;
	twi_package_t * const pkg = transfer.pkg;

	if (transfer.data_count < pkg->length) {

		uint8_t * const data = pkg->buffer;
		data[transfer.data_count++] = bus->MASTER.DATA;

		/* If there is more to read, issue ACK and start a byte read.
		 * Otherwise, issue NACK and STOP to complete the transaction.
		 */
		if (transfer.data_count < pkg->length) {

			bus->MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;

		} else {

			bus->MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
			transfer.status = STATUS_OK;
		}

	} else {

		/* Issue STOP and buffer overflow condition. */

		bus->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		transfer.status = ERR_NO_MEMORY;
	}
}

/**
 * \internal
 *
 * \brief Common TWI master interrupt service routine.
 *
 *  Check current status and calls the appropriate handler.
 */
static void twim_interrupt_handler(void)
{
	uint8_t const master_status = transfer.bus->MASTER.STATUS;

	if (master_status & TWI_MASTER_ARBLOST_bm) {

		transfer.bus->MASTER.STATUS = master_status | TWI_MASTER_ARBLOST_bm;
		transfer.bus->MASTER.CTRLC  = TWI_MASTER_CMD_STOP_gc;
		transfer.status = ERR_BUSY;

	} else if ((master_status & TWI_MASTER_BUSERR_bm) ||
		(master_status & TWI_MASTER_RXACK_bm)) {

		transfer.bus->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		transfer.status = ERR_IO_ERROR;

	} else if (master_status & TWI_MASTER_WIF_bm) {

		twim_write_handler();

	} else if (master_status & TWI_MASTER_RIF_bm) {

		twim_read_handler();

	} else {

		transfer.status = ERR_PROTOCOL;
	}
}

/**
 * \brief Initialize the twi master module
 *
 * \param twi       Base address of the TWI (i.e. &TWIC).
 * \param *opt      Options for initializing the twi module
 *                  (see \ref twi_options_t)
 * \retval STATUS_OK        Transaction is successful
 * \retval ERR_INVALID_ARG  Invalid arguments in \c opt.
 */
status_code_t twi_master_init(TWI_t *twi, const twi_options_t *opt)
{
	uint8_t const ctrla = TWI_MASTER_INTLVL_MED_gc | TWI_MASTER_RIEN_bm |
		TWI_MASTER_WIEN_bm | TWI_MASTER_ENABLE_bm;

	twi->MASTER.BAUD   = opt->speed_reg;
	twi->MASTER.CTRLA  = ctrla;
	twi->MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;

	transfer.locked    = false;
	transfer.status    = STATUS_OK;

	/* Enable configured PMIC interrupt level. */

	PMIC.CTRL |= PMIC_MEDLVLEN_bm;

	sei();

	return STATUS_OK;
}

/**
 * \brief Perform a TWI master write or read transfer.
 *
 * This function is a TWI Master write or read transaction.
 *
 * \param twi       Base address of the TWI (i.e. &TWI_t).
 * \param package   Package information and data
 *                  (see \ref twi_package_t)
 * \param read      Selects the transfer direction
 *
 * \return  status_code_t
 *      - STATUS_OK if the transfer completes
 *      - ERR_BUSY to indicate an unavailable bus
 *      - ERR_IO_ERROR to indicate a bus transaction error
 *      - ERR_NO_MEMORY to indicate buffer errors
 *      - ERR_PROTOCOL to indicate an unexpected bus state
 *      - ERR_INVALID_ARG to indicate invalid arguments.
 */
status_code_t twi_master_transfer(TWI_t *twi,
		const twi_package_t *package, bool read)
{
	/* Do a sanity check on the arguments. */

	if ((twi == NULL) || (package == NULL)) {
		return ERR_INVALID_ARG;
	}

	/* Initiate a transaction when the bus is ready. */
	status_code_t status = twim_acquire(package->no_wait);

	/* If twim module was successfully able to acquire control of the bus */
	if (STATUS_OK == status) {
		
		/* initialize a couple of parameters faciliting the transmission */
		transfer.bus         = (TWI_t *) twi;
		transfer.pkg         = (twi_package_t *) package;
		transfer.addr_count  = 0;
		transfer.data_count  = 0;
		transfer.read        = read;

		/* bit shift to allow for a R/W bit */
		uint8_t const chip = (package->chip) << 1;

		/* if user has specified addr_length or user want to write */
		if (package->addr_length || (false == read)) {
			transfer.bus->MASTER.ADDR = chip;
			
		/* user wants to write */
		} else if (read) {
			transfer.bus->MASTER.ADDR = chip | 0x01;
		}

		status = twim_release(); // this can be a blocking operation
	}

	return status;
}
