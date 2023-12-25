#ifndef _NOR_API_H_
#define _NOR_API_H_

/** READ Functions **/
typedef int (*nor_read_fn_t)(int offset, char* buffer, size_t length);

int NOR_PublicRead(int offset, char* buffer, size_t length);
int NOR_PrivateRead(int offset, char* buffer, size_t length);
int NOR_BootflagRead(int offset, char* buffer, size_t length);
int NOR_OTPRead(int offset, char* buffer, size_t length);


/** WRITE Functions **/
typedef int (*nor_write_fn_t)(int offset, const char* buffer, size_t length);

int NOR_PublicWrite(int offset, const char* buffer, size_t length);
int NOR_PrivateWrite(int offset, const char* buffer, size_t length);
int NOR_BootflagWrite(int offset, const char* buffer, size_t length);
int NOR_OTPWrite(int offset, const char* buffer, size_t length);

#endif /* end of nor_api.h */
