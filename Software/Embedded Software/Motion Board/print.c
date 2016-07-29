#include "UartDebug.h"


//static int *UART_DATA 	= (int *) (UART_BASE + UART_DR_OFFSET);
//static int *UART_LSR	= (int *) (UART_BASE + UART_LSR_OFFSET);

void putc(char c)
{
//	while ( ((*UART_LSR) & LSR_THRE_MASK) == 0);
	UsartPut(c);
	if (c == '\n')
		UsartPut('\r');
}

static void printchar(char **str, int c)
{	
	if (str) 
	{
		**str = c;
		++(*str);
	}
	else 
		putc(c);
}

#define PAD_RIGHT 1
#define PAD_ZERO 2

static int prints(char **out, const char *string, int width, int pad)
{
	register int pc = 0, padchar = ' ';

	if (width > 0) 
	{
		register int len = 0;
		register const char *ptr;
		for (ptr = string; *ptr; ++ptr) ++len;
		if (len >= width) width = 0;
		else width -= len;
		if (pad & PAD_ZERO) padchar = '0';
	}
	if (!(pad & PAD_RIGHT)) 
	{
		for ( ; width > 0; --width) 
		{
			printchar (out, padchar);
			++pc;
		}
	}
	for ( ; *string ; ++string) 
	{
		printchar (out, *string);
		++pc;
	}
	for ( ; width > 0; --width) 
	{
		printchar (out, padchar);
		++pc;
	}

	return pc;
}

// the following should be enough for 32 bit int
#define PRINT_BUF_LEN 12

static int mod(int a, int b)
{
	while (a >= b)
		a = a - b;
	return a;
}

/*static*/ int div(int a, int b)
{
	unsigned int result = 0;
	while (a >= b)
	{
		a = a - b;
		result++;
	}
	return result;
}

static int printi(char **out, int i, int b, int sg, int width, int pad, int letbase)
{
	char print_buf[PRINT_BUF_LEN];
	register char *s;
	register int t, neg = 0, pc = 0;
	register unsigned int u = i;

	if (i == 0) 
	{
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return prints (out, print_buf, width, pad);
	}

	if (sg && b == 10 && i < 0) 
	{
		neg = 1;
		u = -i;
	}

	s = print_buf + PRINT_BUF_LEN-1;
	*s = '\0';

	while (u) 
	{
		t = u % b;  // t = mod(u, b); 		// t = u % b;
		if( t >= 10 )
			t += letbase - '0' - 10;
		*--s = t + '0';
		u /= b;     // u = div(u, b);		// u /= b;
	}

	if (neg) 
	{
		if( width && (pad & PAD_ZERO) ) 
		{
			printchar (out, '-');
			++pc;
			--width;
		}
		else 
		{
			*--s = '-';
		}
	}

	return pc + prints (out, s, width, pad);
}

static int print(char **out, int *varg)
{
	register int width, pad;
	register int pc = 0;
	register char *format = (char *)(*varg++);
	char scr[2];

	for (; *format != 0; ++format) 
	{
		if (*format == '%') 
		{
			++format;
			width = pad = 0;
			if (*format == '\0') break;
			if (*format == '%') goto out;
			if (*format == '-') 
			{
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') 
			{
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; *format >= '0' && *format <= '9'; ++format) 
			{
				width *= 10;
				width += *format - '0';
			}
			if( *format == 's' ) 
			{
				register char *s = *((char **)varg++);
				pc += prints (out, s?s:"(null)", width, pad);
				continue;
			}
			if( *format == 'd' ) 
			{
				pc += printi (out, *varg++, 10, 1, width, pad, 'a');
				continue;
			}
			if( *format == 'x' ) 
			{
				pc += printi (out, *varg++, 16, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'X' ) 
			{
				pc += printi (out, *varg++, 16, 0, width, pad, 'A');
				continue;
			}
			if( *format == 'u' ) 
			{
				pc += printi (out, *varg++, 10, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'c' ) 
			{
				// char are converted to int then pushed on the stack 
				scr[0] = *varg++;
				scr[1] = '\0';
				pc += prints (out, scr, width, pad);
				continue;
			}
		}
		else 
		{
		out:
			printchar (out, *format);
			++pc;
		}
	}
	if (out) 
		**out = '\0';
	return pc;
}

// assuming sizeof(void *) == sizeof(int) 

int printf(const char *format, ...)
{
	register int *varg = (int *)(&format);
	return print(0, varg);
}

int sprintf(char *out, const char *format, ...)
{
	register int *varg = (int *)(&format);
	return print(&out, varg);
}

char getc()
{
	char c;
	//while ( ((*UART_LSR) & LSR_DR_MASK) == 0);
	//c = (char)(*UART_DATA);
	c=UsartGet();
	putc(c);
	if (c == '\r')
	{
		//c = (char)(*UART_DATA);
		c=UsartGet();
		return '\n';
	}
	return c;
}



char* itoa(unsigned int i, char* arr)           // integer to array of characters
{
    int temp = i;
    int num = 0;    
    if(i == 0)
    {   
        arr[0] = '0';
        arr[1] = '\0';
        return arr;
    }
    while(temp != 0)
    {
        temp = temp / 10;
        num++;
    }
    arr[num] = '\0';
    while(num--)
    {
        arr[num] = '0' + i % 10;
        i = i / 10;
    }
    return arr;    
}


/**************************************************
 *
 *  ** ADDED BY VLADIMIR, copied from internet **
 *
 *    ftoa - converts float to string  
 *
 ***************************************************
 *
 *    This is a simple implemetation with rigid
 *    parameters:
 *            - Buffer must be 8 chars long
 *            - 3 digits precision max
 *            - absolute range is -524,287 to 524,287 
 *            - resolution (epsilon) is 0.125 and
 *              always rounds down
 **************************************************/
 /*void ftoa(float Value, char* Buffer)
 {

    sprintf(Buffer, "%f", Value);
 /*    union
     {
         float f;
     
         struct
         {
             unsigned int    mantissa_lo : 16;
             unsigned int    mantissa_hi : 7;    
             unsigned int     exponent : 8;
             unsigned int     sign : 1;
         };
     } helper;
     
     unsigned long mantissa;
     signed char exponent;
     unsigned int int_part;
     char frac_part[3];
     int i, count = 0;
     
     helper.f = Value;
     //mantissa is LS 23 bits
     mantissa = helper.mantissa_lo;
     mantissa += ((unsigned long) helper.mantissa_hi << 16);
     //add the 24th bit to get 1.mmmm^eeee format
     mantissa += 0x00800000;
     //exponent is biased by 127
     exponent = (signed char) helper.exponent - 127;
     
     //too big to shove into 8 chars
     if (exponent > 18)
     {
         Buffer[0] = 'I';
         Buffer[1] = 'n';
         Buffer[2] = 'f';
         Buffer[3] = '\0';
         return;
     }
     
     //too small to resolve (resolution of 1/8)
     if (exponent < -3)
     {
         Buffer[0] = '0';
         Buffer[1] = '\0';
         return;
     }
     
     count = 0;
     
     //add negative sign (if applicable)
     if (helper.sign)
     {
         Buffer[0] = '-';
         count++;
     }
     
     //get the integer part
     int_part = mantissa >> (23 - exponent);    
     //convert to string
     itoa(int_part, &Buffer[count]);
     
     //find the end of the integer
     for (i = 0; i < 8; i++)
         if (Buffer[i] == '\0')
         {
             count = i;
             break;
         }        
 
     //not enough room in the buffer for the frac part    
     if (count > 5)
         return;
     
     //add the decimal point    
     Buffer[count++] = '.';
     
     //use switch to resolve the fractional part
     switch (0x7 & (mantissa  >> (20 - exponent)))
     {
         case 0:
             frac_part[0] = '0';
             frac_part[1] = '0';
             frac_part[2] = '0';
             break;
         case 1:
             frac_part[0] = '1';
             frac_part[1] = '2';
             frac_part[2] = '5';            
             break;
         case 2:
             frac_part[0] = '2';
             frac_part[1] = '5';
             frac_part[2] = '0';            
             break;
         case 3:
             frac_part[0] = '3';
             frac_part[1] = '7';
             frac_part[2] = '5';            
             break;
         case 4:
             frac_part[0] = '5';
             frac_part[1] = '0';
             frac_part[2] = '0';            
             break;
         case 5:
             frac_part[0] = '6';
             frac_part[1] = '2';
             frac_part[2] = '5';            
             break;
         case 6:
             frac_part[0] = '7';
             frac_part[1] = '5';
             frac_part[2] = '0';            
             break;
         case 7:
             frac_part[0] = '8';
             frac_part[1] = '7';
             frac_part[2] = '5';                    
             break;
     }
     
     //add the fractional part to the output string
     for (i = 0; i < 3; i++)
         if (count < 7)
             Buffer[count++] = frac_part[i];
     
     //make sure the output is terminated
     Buffer[count] = '\0';*//*
 }

*/
