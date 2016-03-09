/*
 * twim.h
 *
 * Created: 3/9/2016 1:01:18 AM
 *  Author: Josh
 */ 


#ifndef TWIM_H_
#define TWIM_H_

#include "utils/status_codes.h"

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





void init_twi(void);
void twi_query(twi_sock_t* sock, uint8_t addr, uint8_t n_out,  uint8_t n_in);
void twi_send(twi_sock_t* sock, uint8_t addr, uint8_t n_out);


#endif /* TWIM_H_ */