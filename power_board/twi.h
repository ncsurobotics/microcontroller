/*
 * twim.h
 *
 * Created: 3/9/2016 1:01:18 AM
 *  Author: Josh
 */ 


#ifndef TWIM_H_
#define TWIM_H_

#include "status_codes.h"
#include <avr/io.h>
#include <stdlib.h>
#include <stdbool.h>

/*! Baud register setting calculation. Formula described in datasheet. */
#define TWI_BAUD(F_SYS, F_TWI) ((F_SYS / (2 * F_TWI)) - 5)

/*!
 * \brief Input parameters when initializing the twi module mode
 */
typedef struct {
  unsigned long speed;     //! The baudrate of the TWI bus.
  unsigned long speed_reg; //! The baudrate register value of the TWI bus.
  char chip;			   //! The desired address.
} twi_options_t;



/*!
 * \brief Information concerning the data transmission
 */
typedef struct {
  char		chip;			//! TWI chip address to communicate with.
  uint8_t	addr[3];		//! TWI address/commands to issue to the other chip (node).
  int		addr_length;	//! Length of the TWI data address segment (1-3 bytes).
  void		*buffer;		//! Where to find the data to be written.
  unsigned int length;	//! How many bytes do we want to write.
  bool		no_wait;		//! Whether to wait if bus is busy (false) or return immediately (true)
} twi_package_t;


/* \brief read and write data from an I2C stream */
typedef struct TWI_Sock {
	char bufr[8];
	int r_count;
	char bufw[8];
	int w_count;
} twi_sock_t;

/* *****************************************
** Application level functions *************
********************************************
****************************************** */

#ifdef MASTER_DEVICE
//#ifndef SLAVE_DEVICE
/*! \brief Perform a TWI master write or read transfer.
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
status_code_t twi_master_transfer(TWI_t *twi, const twi_package_t *package,
		bool read);
		
		
// ------------------------------------------------------------------


/*! \brief Read multiple bytes from a TWI compatible slave device
 *
 * \param twi       Base address of the TWI (i.e. &TWI_t).
 * \param package   Package information and data
 *                  (see \ref twi_package_t)
 * \return STATUS_OK   If all bytes were read, error code otherwise
 */
static inline status_code_t twi_master_read(TWI_t *twi,
		const twi_package_t *package)
{
	return twi_master_transfer (twi, package, true);
}

// ------------------------------------------------------------------

/*! \brief Write multiple bytes to a TWI compatible slave device
 *
 * \param twi       Base address of the TWI (i.e. &TWI_t).
 * \param package   Package information and data
 *                  (see \ref twi_package_t)
 * \return STATUS_OK   If all bytes were written, error code otherwise
 */
static inline status_code_t twi_master_write(TWI_t *twi,
		const twi_package_t *package)
{
	return twi_master_transfer (twi, package, false);
}


// ------------------------------------------------------------------


/*! \brief Enable Master Mode of the TWI.
 * \param twi       Base address of the TWI instance.*/
static inline void twi_master_enable(TWI_t *twi) {
  twi->MASTER.CTRLA |= TWI_MASTER_ENABLE_bm;
}


// ------------------------------------------------------------------


/*! \brief Disable Master Mode of the TWI.
 *
 * \param twi       Base address of the TWI instance.
 */
static inline void twi_master_disable(TWI_t *twi)
{
  twi->MASTER.CTRLA &= (~TWI_MASTER_ENABLE_bm);
}

// ------------------------------------------------------------------

/*! \brief Initialize the twi master module
 *
 * \param twi       Base address of the TWI (i.e. &TWIC).
 * \param *opt      Options for initializing the twi module
 *                  (see \ref twi_options_t)
 * \retval STATUS_OK        Transaction is successful
 * \retval ERR_INVALID_ARG  Invalid arguments in \c opt.
 */
status_code_t twi_master_init(TWI_t *twi, const twi_options_t *opt);


// ------------------------------------------------------------------

//#endif
#endif



void init_twi(void);
void twi_query(twi_sock_t* sock, uint8_t addr, uint8_t n_out,  uint8_t n_in);
void twi_send(twi_sock_t* sock, uint8_t addr, uint8_t n_out);

//#ifdef SLAVE_DEVICE
//#ifndef MASTER_DEVICE

/**
 * \defgroup group_xmega_drivers_twi_twis TWI Slave
 *
 * \ingroup group_xmega_drivers_twi
 *
 * \{
 */

/* Transaction status defines.*/
#define TWIS_STATUS_READY                0
#define TWIS_STATUS_BUSY                 1

/* Transaction result enumeration */
typedef enum TWIS_RESULT_enum {
	TWIS_RESULT_UNKNOWN            = (0x00<<0),
	TWIS_RESULT_OK                 = (0x01<<0),
	TWIS_RESULT_BUFFER_OVERFLOW    = (0x02<<0),
	TWIS_RESULT_TRANSMIT_COLLISION = (0x03<<0),
	TWIS_RESULT_BUS_ERROR          = (0x04<<0),
	TWIS_RESULT_FAIL               = (0x05<<0),
	TWIS_RESULT_ABORTED            = (0x06<<0),
} TWIS_RESULT_t;

/* Buffer size defines. */
#define TWIS_RECEIVE_BUFFER_SIZE         8
#define TWIS_SEND_BUFFER_SIZE            8



/*! \brief TWI slave driver struct.
 *
 *  TWI slave struct. Holds pointer to TWI module and data processing routine,
 *  buffers and necessary variables.
 */
typedef struct TWI_Slave {
	TWI_t *interface;                               /*!< Pointer to what interface to use*/
	void (*Process_Data) (void);                    /*!< Pointer to process data function*/
	register8_t receivedData[TWIS_RECEIVE_BUFFER_SIZE]; /*!< Read data*/
	register8_t sendData[TWIS_SEND_BUFFER_SIZE];        /*!< Data to write*/
	register8_t bytesReceived;                          /*!< Number of bytes received*/
	register8_t bytesSent;                              /*!< Number of bytes sent*/
	register8_t status;                                 /*!< Status of transaction*/
	register8_t result;                                 /*!< Result of transaction*/
	bool abort;                                     /*!< Strobe to abort*/
} TWI_Slave_t;


void TWI_SlaveInitializeDriver(TWI_Slave_t *twi,
                               TWI_t *module,
                               void (*processDataFunction) (void));

void TWI_SlaveInitializeModule(TWI_Slave_t *twi,
                               uint8_t address,
                               TWI_SLAVE_INTLVL_t intLevel);

void TWI_SlaveInterruptHandler(TWI_Slave_t *twi);
void TWI_SlaveAddressMatchHandler(TWI_Slave_t *twi);
void TWI_SlaveStopHandler(TWI_Slave_t *twi);
void TWI_SlaveDataHandler(TWI_Slave_t *twi);
void TWI_SlaveReadHandler(TWI_Slave_t *twi);
void TWI_SlaveWriteHandler(TWI_Slave_t *twi);
void TWI_SlaveTransactionFinished(TWI_Slave_t *twi, uint8_t result);


/*! \brief Enable Slave Mode of the TWI.
 *
 * \param twi         Base address of the TWI instance.
 */
static inline void twi_slave_enable(TWI_t *twi)
{
  twi->SLAVE.CTRLA |= TWI_SLAVE_ENABLE_bm;
}

/*! \brief Disable Slave Mode of the TWI.
 *
 * \param twi         Base address of the TWI instance.
 */
static inline void twi_slave_disable(TWI_t *twi)
{
  twi->SLAVE.CTRLA &= (~TWI_SLAVE_ENABLE_bm);
}

//#endif
//#endif

#endif /* TWIM_H_ */