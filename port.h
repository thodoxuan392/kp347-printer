#ifndef KP347_PRINTER_PORT_H
#define KP347_PRINTER_PORT_H

#include "main.h"

#include "Hal/uart.h"
#include "Hal/timer.h"

#define KP347_SEND_BYTE(data)				UART_send_byte(UART_4, data)
#define KP347_IS_AVAILABLE()				UART_receive_available(UART_4)
#define KP347_RECEIVE()                     UART_receive_data(UART_4)
// Refer this function https://www.arduino.cc/reference/en/language/functions/communication/stream/streamread/
/**
 * Return -1 if not available
 */
#define KP347_STREAM_READ()                 UART_stream_read(UART_4)


#define micros()             TIMER_get_tick_us()		// Get tick in us
#define yield()             (void)(NULL)			// Do nothing


#endif // KP347_PRINTER_PORT_H
