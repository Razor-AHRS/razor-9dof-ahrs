#ifndef FCNCTL2_H 
#define FCNCTL2_H

// Prevent Winsock.h from being included by the Windows.h header.
// This must be done if we plan to include Winsock2.h in other files.
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif // WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <errno.h>

#include <fcntl.h>
#include <stdio.h>
#include <stdarg.h>

#ifdef _MSC_VER
// Disable some Visual Studio warnings.
#pragma warning(disable : 4996)
#endif // _MSC_VER

/* open/fcntl.  */
#ifndef O_NOCTTY
# define O_NOCTTY	   0400	/* Not fcntl.  */
#endif
//#ifndef O_TRUNC
//# define O_TRUNC	  01000	/* Not fcntl.  */
//#endif
//#ifndef O_APPEND
//# define O_APPEND	  02000
//#endif
#ifndef O_NONBLOCK
# define O_NONBLOCK	  04000
#endif
#ifndef O_NDELAY
# define O_NDELAY	O_NONBLOCK
#endif

#ifndef __O_DIRECTORY
# define __O_DIRECTORY  0200000
#endif
#ifndef __O_NOFOLLOW
# define __O_NOFOLLOW	0400000
#endif
#ifndef __O_CLOEXEC
# define __O_CLOEXEC   02000000
#endif
#ifndef __O_DIRECT
# define __O_DIRECT	 040000
#endif
#ifndef __O_NOATIME
# define __O_NOATIME   01000000
#endif
#ifndef __O_PATH
# define __O_PATH     010000000
#endif
#ifndef __O_TMPFILE
# define __O_TMPFILE   (020000000 | __O_DIRECTORY)
#endif

# define O_DIRECTORY	__O_DIRECTORY	/* Must be a directory.	 */
# define O_NOFOLLOW	__O_NOFOLLOW	/* Do not follow links.	 */
# define O_CLOEXEC	__O_CLOEXEC	/* Set close_on_exec.  */

# define O_DIRECT	__O_DIRECT	/* Direct disk access.	*/
# define O_NOATIME	__O_NOATIME	/* Do not set atime.  */
# define O_PATH		__O_PATH	/* Resolve pathname but do not open file.  */
# define O_TMPFILE	__O_TMPFILE	/* Atomically create nameless file.  */

/* Values for the second argument to `fcntl'.  */
#define F_DUPFD		0	/* Duplicate file descriptor.  */
#define F_GETFD		1	/* Get file descriptor flags.  */
#define F_SETFD		2	/* Set file descriptor flags.  */
#define F_GETFL		3	/* Get file status flags.  */
#define F_SETFL		4	/* Set file status flags.  */

#define MAX_PATH_LENGTH_OPEN_LINUX (128-8)

// Very limited implementation...
__inline int fcntl(int __fd, int __cmd, ...)
{
	HANDLE hDev = (HANDLE)(intptr_t)__fd;
	COMMTIMEOUTS timeouts;
	// Disable if XON/XOFF are necessary, enable if fcntl() is used together with O_NDELAY and timeouts...
#ifdef ENABLE_O_NDELAY_WORKAROUND
	DCB dcb;
	DWORD ReadIntervalTimeout = 0, ReadTotalTimeoutConstant = 0, ReadTotalTimeoutMultiplier = 0, WriteTotalTimeoutConstant = 0;
#endif // ENABLE_O_NDELAY_WORKAROUND
	va_list argptr;
	va_start(argptr, __cmd);

	if (__cmd == F_SETFL)
	{
		int flags = va_arg(argptr, int);

		memset(&timeouts, 0, sizeof(COMMTIMEOUTS));
		if (!GetCommTimeouts(hDev, &timeouts))
		{
			errno = EIO;
			return -1;
		}
		if (flags & O_NDELAY)
		{
			// Disable if XON/XOFF are necessary, enable if fcntl() is used together with O_NDELAY and timeouts...
#ifdef ENABLE_O_NDELAY_WORKAROUND
			// Need to save the original timeouts settings for next calls of fcntl()...

			memset(&dcb, 0, sizeof(DCB));
			if (!GetCommState(hDev, &dcb))
			{
				errno = EIO;
				return -1;
			}

			// Warning : XoffLim and XonLim are just used as placeholders to save the original timeouts settings, 
			// they are not normally related to timeouts...

			ReadIntervalTimeout = timeouts.ReadIntervalTimeout;
			ReadTotalTimeoutConstant = timeouts.ReadTotalTimeoutConstant;
			ReadTotalTimeoutMultiplier = timeouts.ReadTotalTimeoutMultiplier;
			WriteTotalTimeoutConstant = timeouts.WriteTotalTimeoutConstant;
			if (ReadIntervalTimeout == MAXDWORD) ReadIntervalTimeout = 255*100;
			else if (ReadIntervalTimeout/100 > 254) ReadIntervalTimeout = 254*100;
			if (ReadTotalTimeoutConstant == MAXDWORD) ReadTotalTimeoutConstant = 255*100;
			else if (ReadTotalTimeoutConstant/100 > 254) ReadTotalTimeoutConstant = 254*100;
			if (ReadTotalTimeoutMultiplier == MAXDWORD) ReadTotalTimeoutMultiplier = 255*100;
			else if (ReadTotalTimeoutMultiplier/100 > 254) ReadTotalTimeoutMultiplier = 254*100;
			if (WriteTotalTimeoutConstant == MAXDWORD) WriteTotalTimeoutConstant = 255*100;
			else if (WriteTotalTimeoutConstant/100 > 254) WriteTotalTimeoutConstant = 254*100;
			dcb.XoffLim = (WORD)(((ReadIntervalTimeout/100)<<8)|(ReadTotalTimeoutConstant/100));
			dcb.XonLim = (WORD)(((ReadTotalTimeoutMultiplier/100)<<8)|(WriteTotalTimeoutConstant/100));
			if (!SetCommState(hDev, &dcb))
			{
				errno = EIO;
				return -1;
			}
#endif // ENABLE_O_NDELAY_WORKAROUND
			// Change timeout settings.
			timeouts.ReadIntervalTimeout = MAXDWORD;
			timeouts.ReadTotalTimeoutConstant = 0;
			timeouts.ReadTotalTimeoutMultiplier = 0;
			timeouts.WriteTotalTimeoutConstant = 1; // ???
			timeouts.WriteTotalTimeoutMultiplier = 0;
			if (!SetCommTimeouts(hDev, &timeouts))
			{
				errno = EIO;
				return -1;
			}
		}
		else
		{
			// Disable if XON/XOFF are necessary, enable if fcntl() is used together with O_NDELAY and timeouts...
#ifdef ENABLE_O_NDELAY_WORKAROUND
			// Should restore original timeout values...

			memset(&dcb, 0, sizeof(DCB));
			if (!GetCommState(hDev, &dcb))
			{
				errno = EIO;
				return -1;
			}

			// Warning : XoffLim and XonLim are just used as placeholders to save the original timeouts settings, 
			// they are not normally related to timeouts...

			if ((dcb.XoffLim>>8) == 255) timeouts.ReadIntervalTimeout = MAXDWORD;
			else timeouts.ReadIntervalTimeout = (dcb.XoffLim>>8)*100;
			if ((dcb.XoffLim&0x00FF) == 255) timeouts.ReadTotalTimeoutConstant = MAXDWORD;
			else timeouts.ReadTotalTimeoutConstant = (dcb.XoffLim&0x00FF)*100;
			if ((dcb.XonLim>>8) == 255) timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
			else timeouts.ReadTotalTimeoutMultiplier = (dcb.XonLim>>8)*100;
			if ((dcb.XonLim&0x00FF) == 255) timeouts.WriteTotalTimeoutConstant = MAXDWORD;
			else timeouts.WriteTotalTimeoutConstant = (dcb.XonLim&0x00FF)*100;
			timeouts.WriteTotalTimeoutMultiplier = 0;
			// Set to full blocking in case the restored values were non-blocking.
			if ((timeouts.ReadIntervalTimeout == MAXDWORD)&&(timeouts.ReadTotalTimeoutConstant == 0)&&(timeouts.ReadTotalTimeoutMultiplier == 0))
			{
				timeouts.ReadIntervalTimeout = 0;
				timeouts.ReadTotalTimeoutConstant = 0;
				timeouts.ReadTotalTimeoutMultiplier = 0;
				timeouts.WriteTotalTimeoutConstant = 0;
				timeouts.WriteTotalTimeoutMultiplier = 0;
			}
#else
			timeouts.ReadIntervalTimeout = 0;
			timeouts.ReadTotalTimeoutConstant = 0;
			timeouts.ReadTotalTimeoutMultiplier = 0;
			timeouts.WriteTotalTimeoutConstant = 0;
			timeouts.WriteTotalTimeoutMultiplier = 0;
#endif // ENABLE_O_NDELAY_WORKAROUND
			if (!SetCommTimeouts(hDev, &timeouts))
			{
				errno = EIO;
				return -1;
			}
		}
	}
	else if (__cmd == F_GETFL)
	{
		int flags = 0;

		memset(&timeouts, 0, sizeof(COMMTIMEOUTS));
		if (!GetCommTimeouts(hDev, &timeouts))
		{
			errno = EIO;
			return -1;
		}
		if ((timeouts.ReadIntervalTimeout == MAXDWORD)&&(timeouts.ReadTotalTimeoutConstant == 0)&&(timeouts.ReadTotalTimeoutMultiplier == 0))
		{
			// Polling read.
			flags |= O_NDELAY;
		}
		else
		{
			// Blocking read.
			flags &= ~O_NDELAY;
		}
		return flags;
	}

	return 0;
}

__inline int open_linux(const char *__path, int __oflag, ...)
{
	// The optional arguments are ignored...

	int fd = -1;
	HANDLE hDev = INVALID_HANDLE_VALUE;
	char szDeviceTemp[2*(MAX_PATH_LENGTH_OPEN_LINUX+8)];
	TCHAR tstr[2*(MAX_PATH_LENGTH_OPEN_LINUX+8)];
	DWORD dwDesiredAccess = 0;
	DWORD dwCreationDisposition = 0;
	DWORD dwShareMode = 0;
	//SECURITY_ATTRIBUTES SecurityAttributes;
	DWORD dwFlagsAndAttributes = 0;

	// To be able to use COM10 and greater we need to add "\\.\" (that becomes "\\\\.\\" 
	// in C because the '\' is a special character).
	memset(szDeviceTemp, 0, sizeof(szDeviceTemp));
//#ifdef WINCE
	strcpy(szDeviceTemp, __path);
//#else
//	sprintf(szDeviceTemp, "\\\\.\\%s", __path);
//#endif // WINCE

#ifdef UNICODE
	mbstowcs(tstr, szDeviceTemp, sizeof(szDeviceTemp)/2);
#else
	memcpy(tstr, szDeviceTemp, sizeof(szDeviceTemp)/2);
#endif // UNICODE
	tstr[sizeof(tstr)-1] = 0;

	if (__oflag & O_RDONLY) dwDesiredAccess |= GENERIC_READ;
	if (__oflag & O_WRONLY) dwDesiredAccess |= GENERIC_WRITE;
	if (__oflag & O_RDWR) dwDesiredAccess |= GENERIC_READ|GENERIC_WRITE;

	if (__oflag & O_APPEND) dwCreationDisposition |= OPEN_EXISTING;
	if (__oflag & O_CREAT)
	{
		if (__oflag & O_EXCL) dwCreationDisposition |= CREATE_NEW;
		else dwCreationDisposition |= CREATE_ALWAYS;
	}
	if (__oflag & O_TRUNC) dwCreationDisposition |= TRUNCATE_EXISTING;

	if ((__oflag & O_NONBLOCK)||(__oflag&O_NDELAY)) dwFlagsAndAttributes |= 0; // ?

	if (__oflag & O_DIRECT) dwFlagsAndAttributes |= FILE_FLAG_NO_BUFFERING|FILE_FLAG_WRITE_THROUGH;

	if (__oflag & O_TMPFILE) dwFlagsAndAttributes |= FILE_ATTRIBUTE_TEMPORARY;

	hDev = CreateFile(tstr, dwDesiredAccess, dwShareMode, NULL, dwCreationDisposition, dwFlagsAndAttributes, NULL);

	if (hDev == INVALID_HANDLE_VALUE)
	{
		errno = EIO;
		return -1;
	}

	//#ifndef DISABLE_FORCE_CLEAR_DTR
	//if (!EscapeCommFunction(hDev, CLRDTR))
	//{
	//
	//}
	//#endif // DISABLE_FORCE_CLEAR_DTR

	fd = (int)(intptr_t)hDev;

	return fd;
}

#ifdef _MSC_VER
// Restore the Visual Studio warnings previously disabled.
#pragma warning(default : 4996)
#endif // _MSC_VER

#endif // FCNCTL2_H
