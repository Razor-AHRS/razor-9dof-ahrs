//#ifdef __MINGW32__
//
//#  include_next <unistd.h>
//
//#else

#ifndef UNISTD_H 
#define UNISTD_H

/* 
This file intended to serve as a drop-in replacement for unistd.h on Windows. 
Please add/remove functionality as neeeded.

See http://pubs.opengroup.org/onlinepubs/7908799/xsh/unistd.h.html.
*/ 

#include <io.h> 
#include <direct.h>
#include <process.h>
//#include <getopt.h> // getopt from: http://www.pwilson.net/sample.html.
// To get Sleep().
#include <winsock2.h>

// Prevent Winsock.h from being included by the Windows.h header.
// This must be done if we plan to include Winsock2.h in other files.
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif // WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <errno.h>

#include <sys/types.h>

//#define srandom srand 
//#define random rand 

//const W_OK = 2; 
//const R_OK = 4; 

//#define access _access 
//#define ftruncate _chsize 

#ifndef _SSIZE_T_
#ifndef ssize_t
#define ssize_t long 
#endif // ssize_t
#endif // _SSIZE_T_

__inline int usleep(unsigned int usec)
{
	Sleep(usec/1000);
	return 0;
}

__inline ssize_t read_linux(int __fd, void* __buf, size_t __nbytes)
{
	HANDLE hDev = (HANDLE)(intptr_t)__fd;
	DWORD readBytes = 0;

	if (!ReadFile(hDev, __buf, (DWORD)__nbytes, &readBytes, NULL))
	{
		// To be improved by checking GetLastError()...
		errno = EIO;
		return -1;
	}

	return (ssize_t)readBytes;
}

__inline ssize_t write_linux(int __fd, const void* __buf, size_t __nbytes)
{
	HANDLE hDev = (HANDLE)(intptr_t)__fd;
	DWORD writtenBytes = 0;

	if (!WriteFile(hDev, __buf, (DWORD)__nbytes, &writtenBytes, NULL))
	{
		// To be improved by checking GetLastError()...
		errno = EIO;
		return -1;
	}

	return (ssize_t)writtenBytes;
}

__inline int close_linux(int __fd)
{
	HANDLE hDev = (HANDLE)(intptr_t)__fd;

	if (!CloseHandle(hDev))
	{
		// To be improved by checking GetLastError()...
		errno = EIO;
		return -1;
	}

	return 0;
}

#endif // UNISTD_H

//#endif // __MINGW32__
