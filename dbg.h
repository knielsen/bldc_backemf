#include <stdint.h>

extern void serial_output_hexbyte(uint8_t byte);
extern void serial_output_str(const char *str);
extern void println_uint32(uint32_t val);
extern void float_to_str(char *buf, float f, uint32_t dig_before, uint32_t dig_after);
extern void println_float(float f, uint32_t dig_before, uint32_t dig_after);
extern void config_serial_dbg(void);
