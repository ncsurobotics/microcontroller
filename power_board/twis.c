/**
 * \file *********************************************************************
 *
 * \brief
 *      XMEGA TWI slave driver source file.
 *
 *      This file contains the function implementations the XMEGA TWI slave
 *      driver.
 *
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. The driver is intended for
 *      rapid prototyping and documentation purposes for getting started with
 *      the XMEGA TWI slave module.
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 *      Several functions use the following construct:
 *          "some_register = ... | (some_parameter ? SOME_BIT_bm : 0) | ..."
 *      Although the use of the ternary operator ( if ? then : else ) is
 *      discouraged, in some occasions the operator makes it possible to write
 *      pretty clean and neat code. In this driver, the construct is used to
 *      set or not set a configuration bit based on a boolean input parameter,
 *      such as the "some_parameter" in the example above.
 *
 * \par Application note:
 *      AVR1308: Using the XMEGA TWI
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 *      Atmel Corporation: http://www.atmel.com \n
 *
 * $Revision: 2660 $
 * $Date: 2009-08-11 12:28:58 +0200 (Tue, 11 Aug 2009) $  \n
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 *****************************************************************************/
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "twi.h"
#include <util/delay.h>

/*! \brief Initalizes TWI slave driver structure.
 *
 *  Initialize the instance of the TWI Slave and set the appropriate values.
 *
 *  \param twi                  The TWI_Slave_t struct instance.
 *  \param module               Pointer to the TWI module.
 *  \param processDataFunction  Pointer to the function that handles incoming data.
 */
void TWI_SlaveInitializeDriver(TWI_Slave_t *twi,
                               TWI_t *module,
                               void (*processDataFunction) (void))
{
	twi->interface = module;
	twi->Process_Data = processDataFunction;
	twi->bytesReceived = 0;
	twi->bytesSent = 0;
	twi->status = TWIS_STATUS_READY;
	twi->result = TWIS_RESULT_UNKNOWN;
	twi->abort = false;
}


/*! \brief Initialize the TWI module.
 *
 *  Enables interrupts on address recognition and data available.
 *  Remember to enable interrupts globally from the main application.
 *
 *  \param twi        The TWI_Slave_t struct instance.
 *  \param address    Slave address for this module.
 *  \param intLevel   Interrupt level for the TWI slave interrupt handler.
 */
void TWI_SlaveInitializeModule(TWI_Slave_t *twi,
                               uint8_t address,
                               TWI_SLAVE_INTLVL_t intLevel)
{
	twi->interface->SLAVE.CTRLA = intLevel |
	                              TWI_SLAVE_DIEN_bm |
	                              TWI_SLAVE_APIEN_bm |
	                              TWI_SLAVE_ENABLE_bm;
	twi->interface->SLAVE.ADDR = (address<<1);
}

/*! \brief Common TWI slave interrupt service routine.
 *
 *  Handles all TWI transactions and responses to address match, data reception,
 *  data transmission, bus error and data collision.
 *
 *  \param twi The TWI_Slave_t struct instance.
 */
void TWI_SlaveInterruptHandler(TWI_Slave_t *twi)
{
	uint8_t currentStatus = twi->interface->SLAVE.STATUS;

	/* If bus error. */
	if (currentStatus & TWI_SLAVE_BUSERR_bm) {
		twi->bytesReceived = 0;
		twi->bytesSent = 0;
		twi->result = TWIS_RESULT_BUS_ERROR;
		twi->status = TWIS_STATUS_READY;
	}

	/* If transmit collision. */
	else if (currentStatus & TWI_SLAVE_COLL_bm) {
		twi->bytesReceived = 0;
		twi->bytesSent = 0;
		twi->result = TWIS_RESULT_TRANSMIT_COLLISION;
		twi->status = TWIS_STATUS_READY;
	}

	/* If address match. */
	else if ((currentStatus & TWI_SLAVE_APIF_bm) &&
	        (currentStatus & TWI_SLAVE_AP_bm)) {

		TWI_SlaveAddressMatchHandler(twi);
	}

	/* If stop (only enabled through slave read transaction). */
	else if (currentStatus & TWI_SLAVE_APIF_bm) {
		TWI_SlaveStopHandler(twi);
	}

	/* If data interrupt. */
	else if (currentStatus & TWI_SLAVE_DIF_bm) {
		TWI_SlaveDataHandler(twi);
	}

	/* If unexpected state. */
	else {
		TWI_SlaveTransactionFinished(twi, TWIS_RESULT_FAIL);
	}
}

/*! \brief TWI address match interrupt handler.
 *
 *  Prepares TWI module for transaction when an address match occurs.
 *
 *  \param twi The TWI_Slave_t struct instance.
 */
void TWI_SlaveAddressMatchHandler(TWI_Slave_t *twi)
{
	/* If application signalling need to abort (error occured). */
	if (twi->abort) {
		twi->interface->SLAVE.CTRLB = TWI_SLAVE_CMD_COMPTRANS_gc;
		TWI_SlaveTransactionFinished(twi, TWIS_RESULT_ABORTED);
		twi->abort = false;
	} else {
		twi->status = TWIS_STATUS_BUSY;
		twi->result = TWIS_RESULT_UNKNOWN;

		/* Disable stop interrupt (the stop interrupt will be enabled again once some data has been received) */
		uint8_t currentCtrlA = twi->interface->SLAVE.CTRLA;
		twi->interface->SLAVE.CTRLA = currentCtrlA & ~TWI_SLAVE_PIEN_bm;

		/* wipe byte counters clean */
		twi->bytesReceived = 0;
		twi->bytesSent = 0;

		/* if master cmds write operation Send ACK, wait for data interrupt. */
		twi->interface->SLAVE.CTRLB = TWI_SLAVE_CMD_RESPONSE_gc;
		
		/* BACKGROUND: a data interrupt is signaled by DIF in the TWI Slave Status Register.
		If master calls for a write operation (master writes data to slave), the slave will 
		acknowledge master, and then the DIF flag will set and trigger the DataInterruptHandler 
		*AFTER* master writes the byte of data it wanted to write to the slave.
		
		If master calls for a read operation (master wants to read data from the slave), then
		the DIF flag will set and trigger the DataInterruptHandler as soon as this slave device
		finishes acknowledging the master instead (as well as when this interrupt finishes such that
		the hardware interrupt handler is free to handle a new interrupt). This is because the slave is 
		soley responsible for putting data on the I2C bus.
		
		The DataInterruptHandler function is capable of automatically handling both of these situations.
		*/
	}
}


/*! \brief TWI stop condition interrupt handler.
 *
 *  \param twi The TWI_Slave_t struct instance.
 */
void TWI_SlaveStopHandler(TWI_Slave_t *twi)
{
	/* Disable stop interrupt. */
	uint8_t currentCtrlA = twi->interface->SLAVE.CTRLA;
	twi->interface->SLAVE.CTRLA = currentCtrlA & ~TWI_SLAVE_PIEN_bm;

	/* Clear APIF, according to flowchart don't ACK or NACK */
	uint8_t currentStatus = twi->interface->SLAVE.STATUS;
	twi->interface->SLAVE.STATUS = currentStatus | TWI_SLAVE_APIF_bm;

	TWI_SlaveTransactionFinished(twi, TWIS_RESULT_OK);

}


/*! \brief TWI data interrupt handler.
 *
 *  Calls the appropriate slave read or write handler.
 *
 *  \param twi The TWI_Slave_t struct instance.
 */
void TWI_SlaveDataHandler(TWI_Slave_t *twi)
{
	/* If master calls for a read operation (master wants to read data from slave) */
	if (twi->interface->SLAVE.STATUS & TWI_SLAVE_DIR_bm) {
		TWI_SlaveWriteHandler(twi);
	/* master called for a write operation (master has written data to the slave) */
	} else {
		TWI_SlaveReadHandler(twi);
	}
}


/*! \brief TWI slave read interrupt handler.
 *
 *  Handles TWI slave read transactions and responses.
 *
 *  \param twi The TWI_Slave_t struct instance.
 */
void TWI_SlaveReadHandler(TWI_Slave_t *twi)
{
	/* Enable stop interrupt. */
	uint8_t currentCtrlA = twi->interface->SLAVE.CTRLA;
	twi->interface->SLAVE.CTRLA = currentCtrlA | TWI_SLAVE_PIEN_bm;

	/* If free space in buffer. */
	if (twi->bytesReceived < TWIS_RECEIVE_BUFFER_SIZE) {
		/* Fetch data */
		uint8_t data = twi->interface->SLAVE.DATA;
		twi->receivedData[twi->bytesReceived] = data;

		/* Process data. */
		twi->Process_Data();

		twi->bytesReceived++;

		/* If application signalling need to abort (error occured),
		 * complete transaction and wait for next START. Otherwise
		 * send ACK and wait for data interrupt.
		 */
		if (twi->abort) {
			twi->interface->SLAVE.CTRLB = TWI_SLAVE_CMD_COMPTRANS_gc;
			TWI_SlaveTransactionFinished(twi, TWIS_RESULT_ABORTED);
			twi->abort = false;
		} else {
			twi->interface->SLAVE.CTRLB = TWI_SLAVE_CMD_RESPONSE_gc;
		}
	}
	/* If buffer overflow, send NACK and wait for next START. Set
	 * result buffer overflow.
	 */
	else {
		twi->interface->SLAVE.CTRLB = TWI_SLAVE_ACKACT_bm |
		                              TWI_SLAVE_CMD_COMPTRANS_gc;
		TWI_SlaveTransactionFinished(twi, TWIS_RESULT_BUFFER_OVERFLOW);
	}
}


/*! \brief TWI slave write interrupt handler.
 *
 *  Handles TWI slave write transactions and responses.
 *
 *  \param twi The TWI_Slave_t struct instance.
 */
void TWI_SlaveWriteHandler(TWI_Slave_t *twi)
{
	/* If NACK, slave write transaction finished. */
	if ((twi->bytesSent > 0) && (twi->interface->SLAVE.STATUS &
	                             TWI_SLAVE_RXACK_bm)) {

		twi->interface->SLAVE.CTRLB = TWI_SLAVE_CMD_COMPTRANS_gc;
		TWI_SlaveTransactionFinished(twi, TWIS_RESULT_OK);
	}
	/* If ACK, master expects more data. */
	else {
		if (twi->bytesSent < TWIS_SEND_BUFFER_SIZE) {
			/* load data from send buffer (assuming it's ready) */
			uint8_t data = twi->sendData[twi->bytesSent];
			twi->interface->SLAVE.DATA = data;
			twi->bytesSent++;

			/* Send data, wait for data interrupt. */
			twi->interface->SLAVE.CTRLB = TWI_SLAVE_CMD_RESPONSE_gc;
		}
		/* If buffer overflow. */
		else {
			twi->interface->SLAVE.CTRLB = TWI_SLAVE_CMD_COMPTRANS_gc;
			TWI_SlaveTransactionFinished(twi, TWIS_RESULT_BUFFER_OVERFLOW);
		}
	}
	
	/* PROTIP: next interrupt will be signaled with the raising of the DIF flag,
	*AND* possibly another flag as soon as master ACKs (or NACKs)  what the slave
	just sent over the I2C bus. 
	
	If master NACK'd what the slave just put on the bus, that is like a request to
	stop sending data. This handler may possibly run one more time, but it will notice
	that a NACK was received. Upon this slave will (locally) release the I2C bus, and
	expect master to do the same after master sends a stop bit... unless master sends
	another start bit. This is a repeated start bit, upon which the slave may recognize
	(if it was addressed in the sort of new transaction) and jump in on the still *BUSY*
	I2C bus. */
}


/*! \brief TWI transaction finished function.
 *
 *  Prepares module for new transaction.
 *
 *  \param twi    The TWI_Slave_t struct instance.
 *  \param result The result of the transaction.
 */
void TWI_SlaveTransactionFinished(TWI_Slave_t *twi, uint8_t result)
{
	twi->result = result;
	twi->status = TWIS_STATUS_READY;
}
