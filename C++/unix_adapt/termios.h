#ifndef _TERMIOS_H 
#define _TERMIOS_H

/*
This file intended to serve as a drop-in replacement for termios.h on Windows.
Please add/remove functionality as neeeded.
*/

// Not sure what is that...
#ifndef __USE_MISC 
#define __USE_MISC
#endif // __USE_MISC

/* Get the system-dependent definitions of `struct termios', `tcflag_t',
   `cc_t', `speed_t', and all the macros specifying the flag bits.  */
#include <bits/termios.h>

#ifdef __USE_MISC
/* Compare a character C to a value VAL from the `c_cc' array in a
   `struct termios'.  If VAL is _POSIX_VDISABLE, no character can match it.  */
# define CCEQ(val, c)	((c) == (val) && (val) != _POSIX_VDISABLE)
#endif

// Prevent Winsock.h from being included by the Windows.h header.
// This must be done if we plan to include Winsock2.h in other files.
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif // WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <errno.h>

//#include "NtPathFromHandle.h"

inline UINT _linuxbaudrate2windows(speed_t BaudRate)
{
	switch (BaudRate)
	{
	case B110:
		return CBR_110;
	case B300:
		return CBR_300;
	case B600:
		return CBR_600;
	case B1200:
		return CBR_1200;
	case B4800:
		return CBR_4800;
	case B2400:
		return CBR_2400;
	case B9600:
		return CBR_9600;
	case B19200:
		return CBR_19200;
	case B38400:
		return CBR_38400;
	case B57600:
		return CBR_57600;
	case B115200:
		return CBR_115200;
	default:
		return 0;
	}
}

inline speed_t _windowsbaudrate2linux(UINT BaudRate)
{
	switch (BaudRate)
	{
	case CBR_110:
		return B110;
	case CBR_300:
		return B300;
	case CBR_600:
		return B600;
	case CBR_1200:
		return B1200;
	case CBR_4800:
		return B4800;
	case CBR_2400:
		return B2400;
	case CBR_9600:
		return B9600;
	case CBR_19200:
		return B19200;
	case CBR_38400:
		return B38400;
	case CBR_57600:
		return B57600;
	case CBR_115200:
		return B115200;
	default:
		return 0;
	}
}

__inline speed_t cfgetospeed(const struct termios *__termios_p) 
{
	return __termios_p->c_ospeed;
}

__inline speed_t cfgetispeed (const struct termios *__termios_p) 
{
	return __termios_p->c_ispeed;
}

__inline int cfsetospeed (struct termios *__termios_p, speed_t __speed) 
{
	__termios_p->c_ospeed = __speed;
	return 0;
}

__inline int cfsetispeed (struct termios *__termios_p, speed_t __speed) 
{
	__termios_p->c_ispeed = __speed;
	return 0;
}

#ifdef	__USE_MISC
__inline int cfsetspeed (struct termios *__termios_p, speed_t __speed) 
{
	__termios_p->c_ispeed = __speed;
	__termios_p->c_ospeed = __speed;
	return 0;
}
#endif

/* Put the state of FD into *TERMIOS_P.  */
__inline int tcgetattr (int __fd, struct termios *__termios_p) 
{
	HANDLE hDev = (HANDLE)__fd;
	COMMTIMEOUTS timeouts;
	DCB dcb;
	speed_t speed = 0;

//#pragma region TIMEOUTS
	memset(&timeouts, 0, sizeof(COMMTIMEOUTS));

	if (!GetCommTimeouts(hDev, &timeouts))
	{
		errno = EIO;
		return -1;
	}

	if ((timeouts.ReadIntervalTimeout == MAXDWORD)&&(timeouts.ReadTotalTimeoutConstant == 0)&&(timeouts.ReadTotalTimeoutMultiplier == 0))
	{
		// Polling read.
		__termios_p->c_cc[VMIN] = 0; // Minimum number of characters to read. 
		__termios_p->c_cc[VTIME] = 0; // Time to wait for every character read in tenths of seconds.
	}
	else if ((timeouts.ReadIntervalTimeout == 0)&&(timeouts.ReadTotalTimeoutConstant == 0)&&(timeouts.ReadTotalTimeoutMultiplier == 0))
	{
		// Blocking read.
		__termios_p->c_cc[VMIN] = 1; // Minimum number of characters to read. 
		__termios_p->c_cc[VTIME] = 0; // Time to wait for every character read in tenths of seconds.
	}
	else if ((timeouts.ReadIntervalTimeout == MAXDWORD)&&(timeouts.ReadTotalTimeoutMultiplier == MAXDWORD))
	{
		// Read with timeout.
		if (timeouts.ReadTotalTimeoutConstant/100 > 255)
		{
			errno = EINVAL;
			return -1;
		}
		__termios_p->c_cc[VMIN] = 0; // Minimum number of characters to read. 
		__termios_p->c_cc[VTIME] = (cc_t)(timeouts.ReadTotalTimeoutConstant/100); // Time to wait for every character read in tenths of seconds.
	}
	else if ((timeouts.ReadTotalTimeoutConstant == 0)&&(timeouts.ReadTotalTimeoutMultiplier == 0))
	{
		// Should be approximately equivalent...

		// Read with interbyte timeout.
		if (timeouts.ReadIntervalTimeout/100 > 255)
		{
			errno = EINVAL;
			return -1;
		}
		__termios_p->c_cc[VMIN] = 255; // Minimum number of characters to read. 
		__termios_p->c_cc[VTIME] = (cc_t)(timeouts.ReadIntervalTimeout/100); // Time to wait for every character read in tenths of seconds.
	}
	else
	{
		// Not sure what to do...

		// Read with timeout.
		if (timeouts.ReadTotalTimeoutConstant/100 > 255)
		{
			errno = EINVAL;
			return -1;
		}
		__termios_p->c_cc[VMIN] = 0; // Minimum number of characters to read. 
		__termios_p->c_cc[VTIME] = (cc_t)(timeouts.ReadTotalTimeoutConstant/100); // Time to wait for every character read in tenths of seconds.
	}
//#pragma endregion

//#pragma region DCB
	memset(&dcb, 0, sizeof(DCB));

	if (!GetCommState(hDev, &dcb))
	{
		errno = EIO;
		return -1;
	}

	speed = _windowsbaudrate2linux(dcb.BaudRate);
	__termios_p->c_ispeed = speed;
	__termios_p->c_ospeed = speed;

	switch (dcb.Parity)
	{
	case NOPARITY :
		__termios_p->c_cflag &= ~CMSPAR;
		__termios_p->c_cflag &= ~PARENB;
		__termios_p->c_cflag &= ~PARODD;
		break;
	case MARKPARITY :  
		__termios_p->c_cflag |= CMSPAR;
		__termios_p->c_cflag &= ~PARENB;
		__termios_p->c_cflag |= PARODD;
		break;
	case SPACEPARITY :  
		__termios_p->c_cflag |= CMSPAR;
		__termios_p->c_cflag &= ~PARENB;
		__termios_p->c_cflag &= ~PARODD;
		break;
	case ODDPARITY :    
		__termios_p->c_cflag &= ~CMSPAR;
		__termios_p->c_cflag |= PARENB;
		__termios_p->c_cflag |= PARODD;
		break;
	case EVENPARITY :  
		__termios_p->c_cflag &= ~CMSPAR;
		__termios_p->c_cflag |= PARENB;
		__termios_p->c_cflag &= ~PARODD;
		break;
	default :
		errno = EINVAL;
		return -1;
	}

	if (dcb.fParity)
	{
		__termios_p->c_iflag |= INPCK;
	}
	else
	{
		__termios_p->c_iflag &= ~INPCK;
	}

	if (!dcb.fErrorChar) 
	{
		__termios_p->c_iflag |= IGNPAR; // A character with a framing or parity error will be discarded.
		// This is only valid (at least for parity errors) if parity checking is enabled.
	}
	else
	{
		__termios_p->c_iflag &= ~IGNPAR;
	}

	if (dcb.ErrorChar == 0) 
	{
		__termios_p->c_iflag &= ~PARMRK; // Never mark a framing or parity error with prefix bytes.
	}
	else
	{
		__termios_p->c_iflag |= PARMRK;
	}

	__termios_p->c_cflag &= ~CSIZE; // Erase the previous flag for the number of data bits.

	switch (dcb.ByteSize)
	{
	case 8:
		__termios_p->c_cflag |= CS8;
		break;
	case 7:
		__termios_p->c_cflag |= CS7;
		break;
	case 6:
		__termios_p->c_cflag |= CS6;
		break;
	case 5:
		__termios_p->c_cflag |= CS5;
		break;
	default :
		errno = EINVAL;
		return -1;
	}

	switch (dcb.StopBits)
	{
	case ONESTOPBIT:
		__termios_p->c_cflag &= ~CSTOPB;
		break;
	case TWOSTOPBITS:
		__termios_p->c_cflag |= CSTOPB;
		break;
	default :
		errno = EINVAL;
		return -1;
	}

	// Binary mode.
	if (dcb.fBinary)
	{
		// The c_cflag member contains two options that should always be enabled,
		// CLOCAL and CREAD. These will ensure that your program does not become
		// the 'owner' of the port subject to sporatic job control and hangup signals, and
		// also that the serial interface driver will read incoming data bytes.
		__termios_p->c_cflag |= (CLOCAL | CREAD);

		// Raw input, no echo, no signals.
		__termios_p->c_lflag &= ~(ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHONL | ISIG);

		// Raw input.
		__termios_p->c_iflag &= ~(IGNBRK | BRKINT | INLCR | IGNCR | ICRNL);

		// Raw output.
		__termios_p->c_oflag &= ~(OPOST | ONLCR | ONOCR | ONLRET | OCRNL);
	}
	else
	{
		// What should be done...?

		// The c_cflag member contains two options that should always be enabled,
		// CLOCAL and CREAD. These will ensure that your program does not become
		// the 'owner' of the port subject to sporatic job control and hangup signals, and
		// also that the serial interface driver will read incoming data bytes.
		__termios_p->c_cflag |= (CLOCAL | CREAD);

		__termios_p->c_lflag |= ICANON;
	}

	// Software flow control.
	if (!(dcb.fOutX||dcb.fInX))
	{
		__termios_p->c_iflag &= ~(IXON | IXOFF | IXANY);
	}
	else
	{
		__termios_p->c_iflag |= (IXON | IXOFF);
	}

	__termios_p->c_cc[VSTART] = dcb.XonChar;
	__termios_p->c_cc[VSTOP] = dcb.XoffChar;
	__termios_p->c_cc[VEOF] = dcb.EofChar;

	// Hardware flow control.
	if (!dcb.fOutxCtsFlow)
	{
#ifdef CRTSCTS
		__termios_p->c_cflag &= ~CRTSCTS;
#endif // CRTSCTS
#ifdef CNEW_RTSCTS
		__termios_p->c_cflag &= ~CNEW_RTSCTS;
#endif // CNEW_RTSCTS
	}
	else
	{
#ifdef CRTSCTS
		__termios_p->c_cflag |= CRTSCTS;
#endif // CRTSCTS
#ifdef CNEW_RTSCTS
		__termios_p->c_cflag |= CNEW_RTSCTS;
#endif // CNEW_RTSCTS
	}
	
	// The most likely scenario where this flag is useful is if the communication channel is 
	// configured for parity and seven bits per character. In this case, the eigth bit on every 
	// received character is a parity bit, not part of the data payload. The user program does 
	// not need to know the value of the parity bit.
	__termios_p->c_iflag &= ~ISTRIP;
//#pragma endregion

	return 0;
}

/* Set the state of FD to *TERMIOS_P.
   Values for OPTIONAL_ACTIONS (TCSA*) are in <bits/termios.h>.  */
__inline int tcsetattr (int __fd, int __optional_actions, const struct termios *__termios_p) 
{
	HANDLE hDev = (HANDLE)__fd;
	COMMTIMEOUTS timeouts;
	DCB dcb;

	UNREFERENCED_PARAMETER(__optional_actions);

//#pragma region TIMEOUTS
	memset(&timeouts, 0, sizeof(COMMTIMEOUTS));

	if ((__termios_p->c_cc[VMIN] == 0)&&(__termios_p->c_cc[VTIME]) == 0)
	{
		// Polling read.
		timeouts.ReadIntervalTimeout = MAXDWORD; timeouts.ReadTotalTimeoutConstant = 0; timeouts.ReadTotalTimeoutMultiplier = 0;
		timeouts.WriteTotalTimeoutConstant = 1; // ???
	}
	else if ((__termios_p->c_cc[VMIN] == 1)&&(__termios_p->c_cc[VTIME]) == 0)
	{
		// Blocking read.
		timeouts.ReadIntervalTimeout = 0; timeouts.ReadTotalTimeoutConstant = 0; timeouts.ReadTotalTimeoutMultiplier = 0;
		timeouts.WriteTotalTimeoutConstant = 0; // ???
	}
	else if ((__termios_p->c_cc[VMIN] == 0)&&(__termios_p->c_cc[VTIME]) > 0)
	{
		// Read with timeout.
		if (__termios_p->c_cc[VTIME]*100 > MAXDWORD)
		{
			errno = EINVAL;
			return -1;
		}
		timeouts.ReadIntervalTimeout = MAXDWORD; timeouts.ReadTotalTimeoutConstant = __termios_p->c_cc[VTIME]*100; timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
		timeouts.WriteTotalTimeoutConstant = __termios_p->c_cc[VTIME]*100; // ???
	}
	else if ((__termios_p->c_cc[VMIN] == 255)&&(__termios_p->c_cc[VTIME]) > 0)
	{
		// Should be approximately equivalent...

		// Read with interbyte timeout.
		if (__termios_p->c_cc[VTIME]*100 > MAXDWORD)
		{
			errno = EINVAL;
			return -1;
		}
		timeouts.ReadIntervalTimeout = __termios_p->c_cc[VTIME]*100; timeouts.ReadTotalTimeoutConstant = 0; timeouts.ReadTotalTimeoutMultiplier = 0;
		timeouts.WriteTotalTimeoutConstant = 0; // ???
	}
	else
	{
		// Not sure what to do...

		// Read with interbyte timeout.
		if (__termios_p->c_cc[VTIME]*100 > MAXDWORD)
		{
			errno = EINVAL;
			return -1;
		}
		timeouts.ReadIntervalTimeout = __termios_p->c_cc[VTIME]*100; timeouts.ReadTotalTimeoutConstant = 0; timeouts.ReadTotalTimeoutMultiplier = 0;
		timeouts.WriteTotalTimeoutConstant = 0; // ???
	}

	if (!SetCommTimeouts(hDev, &timeouts))
	{
		errno = EIO;
		return -1;
	}
//#pragma endregion

//#pragma region DCB
	memset(&dcb, 0, sizeof(DCB));

	// Not possible to have different c_ispeed and c_ospeed on Windows...
	dcb.BaudRate = _linuxbaudrate2windows(__termios_p->c_ispeed);
	if (dcb.BaudRate == 0)
	{
		errno = EINVAL;
		return -1;
	}

	switch (__termios_p->c_cflag & (CMSPAR | PARENB | PARODD))
	{
	case 0:
		dcb.Parity = NOPARITY;
		break;
	case (CMSPAR | PARODD):
		dcb.Parity = MARKPARITY;
		break;
	case (CMSPAR):
		dcb.Parity = SPACEPARITY;
		break;
	case (PARENB | PARODD):
		dcb.Parity = ODDPARITY;
		break;
	case (PARENB):
		dcb.Parity = EVENPARITY;
		break;
	default:
		errno = EINVAL;
		return -1;
	}

	dcb.fParity = (__termios_p->c_iflag & INPCK)? TRUE: FALSE;

	if (__termios_p->c_iflag & IGNPAR)
	{
		dcb.fErrorChar = FALSE; // Indicates whether bytes received with parity errors are 
		// replaced with the character specified by the ErrorChar member. 
	}
	else
	{
		dcb.fErrorChar = TRUE;
	}

	if (__termios_p->c_iflag & PARMRK)
	{
		dcb.ErrorChar = 37; // ?
	}
	else
	{
		dcb.ErrorChar = 0;
	}
	
	switch (__termios_p->c_cflag & CSIZE)
	{
	case CS8:
		dcb.ByteSize = 8;
		break;
	case CS7:
		dcb.ByteSize = 7;
		break;
	case CS6:
		dcb.ByteSize = 6;
		break;
	case CS5:
		dcb.ByteSize = 5;
		break;
	default:
		errno = EINVAL;
		return -1;
	}

	dcb.StopBits = (__termios_p->c_cflag & CSTOPB)? TWOSTOPBITS: ONESTOPBIT;

	// Binary mode.
	if (__termios_p->c_lflag & ~ICANON)
	{
		dcb.fBinary = TRUE;
	}
	else
	{
		dcb.fBinary = FALSE;
	}

	// Software flow control.	
	if (!(__termios_p->c_iflag & IXON)||(__termios_p->c_iflag & IXOFF))
	{
		dcb.fOutX = FALSE;
		dcb.fInX = FALSE;
	}
	else
	{
		dcb.fOutX = TRUE;
		dcb.fInX = TRUE;
	}

	dcb.XonChar = __termios_p->c_cc[VSTART];
	dcb.XoffChar = __termios_p->c_cc[VSTOP];
	dcb.EofChar = __termios_p->c_cc[VEOF];

#ifdef CRTSCTS
	if (__termios_p->c_cflag & CRTSCTS)
	{
		dcb.fOutxCtsFlow = TRUE;
		dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
		dcb.fOutxDsrFlow = TRUE;
		//dcb.fDsrSensitivity = FALSE;		 
	}
	else
	{
		dcb.fOutxCtsFlow = FALSE;
		dcb.fRtsControl = RTS_CONTROL_DISABLE;
		dcb.fOutxDsrFlow = FALSE;
		//dcb.fDsrSensitivity = FALSE;		 
	}
#endif // CRTSCTS
#ifdef CNEW_RTSCTS
	if (__termios_p->c_cflag & CNEW_RTSCTS)
	{
		dcb.fOutxCtsFlow = TRUE;
		dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
		dcb.fOutxDsrFlow = TRUE;
		//dcb.fDsrSensitivity = FALSE;		 
	}
	else
	{
		dcb.fOutxCtsFlow = FALSE;
		dcb.fRtsControl = RTS_CONTROL_DISABLE;
		dcb.fOutxDsrFlow = FALSE;
		//dcb.fDsrSensitivity = FALSE;		 
	}
#endif // CNEW_RTSCTS

	// Miscellaneous default options.
	//dcb.fDtrControl = DTR_CONTROL_DISABLE;
	//dcb.fAbortOnError = FALSE;
	
	// The SetCommState() function reconfigures the communications resource, but it does not affect
	// the internal output and input buffers of the specified driver. The buffers are not flushed, 
	// and pending read and write operations are not terminated prematurely.
	if (!SetCommState(hDev, &dcb))
	{
		errno = EIO;
		return -1;
	}
//#pragma endregion

	return 0;
}

#ifdef	__USE_MISC
__inline void cfmakeraw(struct termios *__termios_p)
{
	__termios_p->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	__termios_p->c_oflag &= ~OPOST;
	__termios_p->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	__termios_p->c_cflag &= ~(CSIZE | PARENB);
	__termios_p->c_cflag |= CS8;
}
#endif

#endif // _TERMIOS_H
