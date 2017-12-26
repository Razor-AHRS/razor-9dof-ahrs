#ifndef _SYS_TIME_H_
#define _SYS_TIME_H_
#include <time.h>

#ifdef _WIN32

#ifndef ENABLE_SYS_TIME_H_WIN32
#define ENABLE_SYS_TIME_H_WIN32
#endif // ENABLE_SYS_TIME_H_WIN32

// To get struct timeval.
#include <winsock2.h>
// To get struct _timeb.
//#include <sys/timeb.h>

//#define DELTA_EPOCH_IN_MICROSECS 11644473600000000ui64
//#define DELTA_EPOCH_IN_MICROSECS 11644473600000000ULL
#define DELTA_EPOCH_IN_MICROSECS_LOW 1216757760
#define DELTA_EPOCH_IN_MICROSECS_HIGH 2711190

#ifdef DISABLE_TIMEZONE_STRUCT_REDEFINITION
struct timezone2 
{
	int tz_minuteswest; // Minutes W of Greenwich.
	int tz_dsttime; // Type of DST correction.
};
#else
#ifdef _MSC_VER
// Disable Visual Studio warnings about timezone declaration.
#pragma warning(disable : 6244) 
#endif // _MSC_VER

// Local declaration of timezone hides previous declaration in time.h.
struct timezone 
{
	int tz_minuteswest; // Minutes W of Greenwich.
	int tz_dsttime; // Type of DST correction.
};

#ifdef _MSC_VER
// Restore the Visual Studio warnings previously disabled.
#pragma warning(default : 6244) 
#endif // _MSC_VER
#endif // DISABLE_TIMEZONE_STRUCT_REDEFINITION

/*
Obtain the current time, expressed as seconds and microseconds since the Epoch, 
and store it in the timeval structure pointed to by tv (the accuracy should be
around 15 ms).

struct timeval* tv : (INOUT) Valid pointer to the structure that will receive 
the current time.
struct timezone* tz : (INOUT) Usually set to NULL.

Return : EXIT_SUCCESS or EXIT_FAILURE if there is an error.
*/
#ifdef DISABLE_TIMEZONE_STRUCT_REDEFINITION
__inline int gettimeofday(struct timeval* tv, struct timezone2* tz)
#else
__inline int gettimeofday(struct timeval* tv, struct timezone* tz)
#endif // DISABLE_TIMEZONE_STRUCT_REDEFINITION
{
	FILETIME ft; // Will contain a 64-bit value representing the number of 100-nanosecond 
	// intervals since January 1, 1601 (UTC).
	ULONGLONG tmpres = 0;
	ULARGE_INTEGER li;
	ULARGE_INTEGER epoch;
#ifdef USE__TZSET
	static int tzflag;
#else
	TIME_ZONE_INFORMATION tz_winapi;
	int rez = 0; 
#endif // USE__TZSET

	if (tv)
	{
		GetSystemTimeAsFileTime(&ft);
		li.LowPart = ft.dwLowDateTime;
		li.HighPart = ft.dwHighDateTime;

		// Converting file time to UNIX Epoch.
		tmpres = li.QuadPart/(ULONGLONG)10; // Convert into microseconds.
		//tmpres -= DELTA_EPOCH_IN_MICROSECS;
		epoch.LowPart = DELTA_EPOCH_IN_MICROSECS_LOW;
		epoch.HighPart = DELTA_EPOCH_IN_MICROSECS_HIGH;
		tmpres -= epoch.QuadPart;

		tv->tv_sec = (long)(tmpres/(ULONGLONG)1000000);
		tv->tv_usec = (long)(tmpres%(ULONGLONG)1000000);
	}

#ifdef USE__TZSET
	if (tz)
	{
		if (!tzflag)
		{
			_tzset();
			tzflag++;
		}
		tz->tz_minuteswest = _timezone/60;
		tz->tz_dsttime = _daylight;
	}
#else
	if (tz)
	{
		// _tzset(), do not work properly, so we use GetTimeZoneInformation.   
		rez = GetTimeZoneInformation(&tz_winapi);     
		tz->tz_dsttime = (rez == 2)?TRUE:FALSE;     
		tz->tz_minuteswest = tz_winapi.Bias + ((rez == 2)?tz_winapi.DaylightBias:0); 
	}
#endif // USE__TZSET

	return 0;
}

//__inline int gettimeofday(struct timeval* tp, void* tz)
//{
//	struct _timeb timebuffer;
//
//	UNREFERENCED_PARAMETER(tz);
//
//	_ftime(&timebuffer);
//	tp->tv_sec = (long)timebuffer.time;
//	tp->tv_usec = timebuffer.millitm*1000;
//	return 0;
//}

//// From olsrd...
//__inline void gettimeofday(struct timeval *TVal, void *TZone __attribute__ ((unused)))
//{
//	SYSTEMTIME SysTime;
//	FILETIME FileTime;
//	unsigned __int64 Ticks;
//
//	GetSystemTime(&SysTime);
//	SystemTimeToFileTime(&SysTime, &FileTime);
//
//	Ticks = ((__int64) FileTime.dwHighDateTime << 32) | (__int64) FileTime.dwLowDateTime;
//
//	Ticks -= 116444736000000000LL;
//
//	TVal->tv_sec = (unsigned int)(Ticks / 10000000);
//	TVal->tv_usec = (unsigned int)(Ticks % 10000000) / 10;
//}

#endif  // _WIN32

#endif // _SYS_TIME_H_
