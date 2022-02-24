#ifndef AT_PRINTF_H
#define AT_PRINTF_H

#ifdef __cplusplus
extern "C"
{
#endif

int printf(const char *format, ...);
int sprintf(char *out, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* AT_PRINTF_H */
