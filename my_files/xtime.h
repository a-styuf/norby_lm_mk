#ifndef TEST1_XTIME_H
#define TEST1_XTIME_H

typedef	unsigned short			u16_t;
typedef	unsigned int			u32_t;
typedef	signed short			s16_t;
typedef	signed int		    	s32_t;

// DEF: standard signed format
// UNDEF: non-standard unsigned format
// #define	_XT_SIGNED

typedef unsigned char BYTE;

#ifdef	_XT_SIGNED
typedef	s32_t                           xtime_t;
#else
typedef	u32_t                           xtime_t;

#endif

typedef struct
{       /* date and time components */
    BYTE	tm_sec;
    BYTE	tm_min;
    BYTE	tm_hour;
    BYTE	tm_mday;
    BYTE	tm_mon;
    u16_t	tm_year;
} type_tm;

void xttotm(type_tm *t, xtime_t secs);
xtime_t xtmtot(type_tm *t);

#endif //TEST1_XTIME_H
