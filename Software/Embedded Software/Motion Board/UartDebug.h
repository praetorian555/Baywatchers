
#ifndef _UART_DEBUG_H_
#define _UART_DEBUG_H_

#include "stm32f10x.h"
#include "stm32f10x_usart.h"

extern char string[32];
extern unsigned short iterator;

extern void UsartInit   ();
extern void UsartPut    (uint8_t ch);
extern void Send        (const char *buffer, unsigned count);
extern void Clear       ();
extern void SendString  ();

char UsartGet(void);

#endif
