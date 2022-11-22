
#include <energia.h>
// Handles the file system of the FLASH, to store parameters
#include <SLFS.h>
#include <WiFi.h>
#include <stdarg.h>

#include "common.h"

#define MAX_PRECISION	(10)
static const double rounders[MAX_PRECISION + 1] =
{
	0.5,				// 0
	0.05,				// 1
	0.005,				// 2
	0.0005,				// 3
	0.00005,			// 4
	0.000005,			// 5
	0.0000005,			// 6
	0.00000005,			// 7
	0.000000005,		// 8
	0.0000000005,		// 9
	0.00000000005		// 10
};


//////////////////////////////////////////////////////////////////
// Retrieves an IP address in a serial stream formatted as
// xxx.xxx.xxx.xxx
void ParseIP(char *TheString, IPAddress *TheIP)
{
  unsigned char i;
  unsigned char StrIndex = 0;
  unsigned char LocalStrIndex = 0;
  char TempString[20];
  uint8_t tempIP[IPV4_SIZE];

  for(i = 0 ; i < IPV4_SIZE ; i++)
  {
    while(TheString[StrIndex] != '.' && TheString[StrIndex] != '\0')
    {
      TempString[LocalStrIndex] = TheString[StrIndex];
      StrIndex++;
      LocalStrIndex++;
    }

    TempString[LocalStrIndex] = '\0';
    StrIndex++;	// skips the dot for next iteration
    tempIP[i] = atoi(TempString);
    //pprintf("found %u\n", tempIP[i]);
    LocalStrIndex = 0;
  }
  *TheIP = IPAddress(tempIP[0], tempIP[1], tempIP[2], tempIP[3]);
}

//////////////////////////////////////////////////////////////////
// Look for the '=' sign then tries to find a value
unsigned char SkipToValue(char *StringBuffer)
{
  unsigned char i = 0;
  while(StringBuffer[i] != '=')
  {
    if(i >= MAX_SERIAL)
    {
      pprintf("Syntax error - '=' sign is missing\n");
      //pprintf("%s\n",StringBuffer);
      return(0);
    }
    i++;
  }
  i++;
  return(i);
}

//////////////////////////////////////////////////////////////////
// Look for the next value in a list (separator = space ' '
/////////////////////////////////////////////////////
// Look for the ',' sign in a list of values
unsigned char SkipToNextValue(char *StringBuffer, unsigned char StartIndex)
{
  unsigned char i = StartIndex;
  while((StringBuffer[i] != ' ') && (StringBuffer[i] != '\0'))
  {
    if(i >= MAX_STRING_LEN)
    {
      pprintf("Syntax error - missing separator\n");
      //pprintf("%s\n",StringBuffer);
      return(0);
    }
    i++;
  }
  i++;
  return(i);
}

/////////////////////////////////////////////////
// Grabs a line in the config text file
unsigned char GrabLine(char* StringBuffer)
{
  unsigned char i;
  char c = 'a';
  char *pF;

  i = 0;

  SerFlash.readBytes(&c, 1);
  
  while((c != '\n') && (c > 0))
  {  
    if(c >=32)  // stores only regular chars
    {
      StringBuffer[i] = c;
      i++;
    }

    if(i >= MAX_STRING_LEN)
    {
      Serial.print("Line too long, skipping...\n");
      return(0);
    }
    SerFlash.readBytes(&c, 1);
  }
  StringBuffer[i] = '\0';
  return(i);
}

unsigned int StringLength(char* StringBuffer)
{
  unsigned int i = 0;
  
  
  while((StringBuffer[i] != '\n') && (i < MAX_STRING_LEN))
  {
    i++;
  }
  i++;  // keeps the \n in the string length
  return(i); 
}


/*
 *  description  : convert double to string
 *
 */
char * ftoa(double f, char * buf, int precision)
{
	char * ptr = buf;
	char * p = ptr;
	char * p1;
	char c;
	long intPart;

	// check precision bounds
	if (precision > MAX_PRECISION)
		precision = MAX_PRECISION;

	// sign stuff
	if (f < 0)
	{
		f = -f;
		*ptr++ = '-';
	}

	if (precision < 0)  // negative precision == automatic precision guess
	{
		if (f < 1.0) precision = 6;
		else if (f < 10.0) precision = 5;
		else if (f < 100.0) precision = 4;
		else if (f < 1000.0) precision = 3;
		else if (f < 10000.0) precision = 2;
		else if (f < 100000.0) precision = 1;
		else precision = 0;
	}

	// round value according the precision
	if (precision)
		f += rounders[precision];

	// integer part...
	intPart = f;
	f -= intPart;

	if (!intPart)
		*ptr++ = '0';
	else
	{
		// save start pointer
		p = ptr;

		// convert (reverse order)
		while (intPart)
		{
			*p++ = '0' + intPart % 10;
			intPart /= 10;
		}

		// save end pos
		p1 = p;

		// reverse result
		while (p > ptr)
		{
			c = *--p;
			*p = *ptr;
			*ptr++ = c;
		}

		// restore end pos
		ptr = p1;
	}

	// decimal part
	if (precision)
	{
		// place decimal point
		*ptr++ = '.';

		// convert
		while (precision--)
		{
			f *= 10.0;
			c = f;
			*ptr++ = '0' + c;
			f -= c;
		}
	}

	// terminating zero
	*ptr = 0;

	return buf;
}

void SetLedColor(boolean red, boolean green, boolean blue)
{
 digitalWrite(LED_RED, !red);
 digitalWrite(LED_GREEN, !green);
 digitalWrite(LED_BLUE, !blue);
}

#define PRINTF_BUF 80 // define the tmp buffer size (change if desired)
void pprintf(const char *format, ...) {
  char buf[PRINTF_BUF];
  va_list ap;
  va_start(ap, format);
  vsnprintf(buf, sizeof(buf), format, ap);
  for(char *p = &buf[0]; *p; p++) // emulate cooked mode for newlines
  {
    if(*p == '\n')
        Serial.write('\r');
    Serial.write(*p);
  }
  va_end(ap);
}
