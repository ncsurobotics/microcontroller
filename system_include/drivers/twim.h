#ifndef TWIM_H_
#define TWIM_H_

#include "./utils/status_codes.h"
#include <avr/io.h>

/* ******************
***** MACROS ********
*********************/

/*! Baud register setting calculation. Formula described in datasheet. */
#define TWI_BAUD(F_SYS, F_TWI) ((F_SYS / (2 * F_TWI)) - 5)

/************************
***** DATA TYPES ********
*************************/

/* Input parameters used when initializing the twi module mode */
typedef struct {
  unsigned long speed;     //! The baudrate of the TWI bus.
  unsigned long speed_reg; //! The baudrate register value of the TWI bus.
  char chip;			   //! The desired address.
} twi_options_t;

/* type containing Information concerning the actual data transmission packet */
typedef struct {
  char		chip;			//! TWI chip address to communicate with.
  uint8_t	addr[3];		//! TWI address/commands to issue to the other chip (node).
  int		addr_length;	//! Length of the TWI data address segment (1-3 bytes).
  void		*buffer;		//! Where to find the data to be written.
  unsigned int length;	//! How many bytes do we want to write.
  bool		no_wait;		//! Whether to wait if bus is busy (false) or return immediately (true)
} twi_package_t;

/* **********************
***** FUNCTIONS ********
********************** */

status_code_t twi_master_init(TWI_t *twi, const twi_options_t *opt);
status_code_t twi_master_transfer(TWI_t *twi,const twi_package_t *package, bool read);

#endif