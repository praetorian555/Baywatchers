#ifndef _PRINT_H_
#define _PRINT_H_

extern int printf(const char *format, ...);
extern char getc();
extern void putc(char c);
extern char* itoa(unsigned int i, char* a);

 //void ftoa(float Value, char* Buffer); //added by Vladimir

#endif /* _PRINT_H_ */
