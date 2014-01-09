/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

#include "ch.h"
#include "hal.h"
#include "./nrf24l01/nrf24.h"
#include "chprintf.h"

#define uart0 ((BaseSequentialStream *)&SD1)

#define NRF24L01_TRANSFER_DATA_SIZE	4

/* change to FALSE for receiver */
#define NRF24L01_TRANSMITTER	TRUE
//#define NRF24L01_TRANSMITTER	FALSE

/*
 * SPI configuration
 */
static SPIConfig spicfg = {
  NULL,
  GPIOA,
  GPIOA_SPI0SEL,
  CR0_DSS8BIT | CR0_FSPIFF | CR0_CLOCKRATE(2),
  19
};

static EVENTSOURCE_DECL(es);

/*
 * LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 256);
static msg_t Thread1(void *arg) {
  (void)arg;
  EventListener el;

  chEvtRegisterMask(&es, &el, 1);

  while (TRUE) {
    if (!chEvtWaitOneTimeout(ALL_EVENTS, MS2ST(500))) {
#if NRF24L01_TRANSMITTER == TRUE
      palClearPad(GPIOF, GPIOF_LED_GREEN);
      palClearPad(GPIOF, GPIOF_LED_BLUE);
      palSetPad(GPIOF, GPIOF_LED_RED);
    } else {
      palClearPad(GPIOF, GPIOF_LED_RED);
      palClearPad(GPIOF, GPIOF_LED_GREEN);
      palSetPad(GPIOF, GPIOF_LED_BLUE);
    }
#else
      palClearPad(GPIOF, GPIOF_LED_GREEN);
      palClearPad(GPIOF, GPIOF_LED_BLUE);
      palSetPad(GPIOF, GPIOF_LED_RED);
    } else {
      palClearPad(GPIOF, GPIOF_LED_RED);
      palClearPad(GPIOF, GPIOF_LED_BLUE);
      palSetPad(GPIOF, GPIOF_LED_GREEN);
    }
#endif
  }
  return 0;
}

static WORKING_AREA(waThread2, 256);
static WORKING_AREA(waThread3, 256);

msg_t msg_cbuf[5];
msg_t msg_pbuf[5];
MAILBOX_DECL(nrf24_cmb, msg_cbuf, 5);
MAILBOX_DECL(nrf24_pmb, msg_pbuf, 5);
uint8_t nrf24_buf[NRF24L01_TRANSFER_DATA_SIZE * 5];

#if NRF24L01_TRANSMITTER == TRUE
static msg_t uart_receiver_thread(void *arg)
{
  (void) arg;
  msg_t buf;
  
  /*
   * wate for data from uart
   * if data is received send it as message to transmitter
   * thread
   */
  while (TRUE) {
    chMBFetch(&nrf24_cmb, &buf, TIME_INFINITE);
    sdReadTimeout(&SD1, (uint8_t *) buf, NRF24L01_TRANSFER_DATA_SIZE, TIME_INFINITE);
    chMBPost(&nrf24_pmb, buf, TIME_INFINITE);
  }
  return 0;
}

/*
 * Fill buffer from UART and send it to nrf24
 */
static msg_t nrf24_transmitter_thread(void *arg)
{
  (void) arg;
  while (TRUE) {
    msg_t buf;
    
    /*
     * if there is no data from UART send dummy data
     */
    if (RDY_TIMEOUT == chMBFetch(&nrf24_pmb, &buf, 50)) {
      chMBFetch(&nrf24_cmb, &buf, TIME_INFINITE);
      chMBPost(&nrf24_pmb, buf, TIME_INFINITE);
      chMBFetch(&nrf24_pmb, &buf, TIME_INFINITE);
    }
    
    nrf24_send((uint8_t *) buf);
    chMBPost(&nrf24_cmb, buf, TIME_INFINITE);
    
    while (nrf24_isSending())
      chThdSleepMilliseconds(5);

    if (nrf24_lastMessageStatus() == NRF24_MESSAGE_LOST)
      chprintf(uart0, "> Message is lost ...\r\n");
    else
      /* send event to led thread to indicate that line is alive */
      chEvtBroadcast(&es);
  }
  return 0;
}

#else

static msg_t uart_transmitter_thread(void *arg)
{
  (void) arg;
  msg_t buf;
  
  while (TRUE) {
    chMBFetch(&nrf24_pmb, &buf, TIME_INFINITE);
    sdWriteTimeout(&SD1, (uint8_t *) buf, NRF24L01_TRANSFER_DATA_SIZE, TIME_INFINITE);
    chMBPost(&nrf24_cmb, buf, TIME_INFINITE);
  }
  return 0;
}

static msg_t nrf24_receiver_thread(void *arg)
{
  (void) arg;
  
  while (TRUE) {
    msg_t buf;
    
    /*
     * poll for data from nrf24l01
     * if data is received send message to uart transmitter thread
     * send event to led thread to indicate that line is alive
     */
    if (nrf24_dataReady()) {
      chMBFetch(&nrf24_cmb, &buf, TIME_INFINITE);
      nrf24_getData((uint8_t *) buf);
      chMBPost(&nrf24_pmb, buf, TIME_INFINITE);
      chEvtBroadcast(&es);
    }
  }
  return 0;
}
#endif

#if NRF24L01_TRANSMITTER == TRUE
uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
#else
uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
#endif

/*
 * Entry point, note, the main() function is already a thread in the system
 * on entry.
 */
int main(int argc, char **argv) {

  (void)argc;
  (void)argv;
  size_t i;
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);           /* Default: 38400,8,N,1. */
  spiStart(&SPID1, &spicfg);

  /* Channel #2 , payload length: 4 */
  nrf24_config(2, NRF24L01_TRANSFER_DATA_SIZE);
  /* Set the device addresses */
  nrf24_tx_address(tx_address);
  nrf24_rx_address(rx_address);

  for (i = 0; i < 5; i++)
    chMBPost(&nrf24_cmb, (msg_t) &nrf24_buf[NRF24L01_TRANSFER_DATA_SIZE * i], TIME_INFINITE);	

  /*
   * Creates the RF status thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

#if NRF24L01_TRANSMITTER == TRUE
  /* nrf24 thread */
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, nrf24_transmitter_thread, NULL);
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, uart_receiver_thread, NULL);
#else
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, nrf24_receiver_thread, NULL);
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, uart_transmitter_thread, NULL);
#endif

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop
   */
  while (TRUE) {
          chThdSleepMilliseconds(500);
  }

  return 0;
}
