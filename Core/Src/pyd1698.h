/***************************************************************************//**
 * @file
 * @brief pyd1698 PIR driver header file
 *****************************************************************************/
#ifndef PYD1698_H
#define PYD1698_H

int pir_init(void);
int read_pir_data(void);
int read_pir_val(void); // Valid function to read ADC PIR value, to use this function sensor should be configured for continuous reading
void write_pir_regval(unsigned long regval);
void pir_clear_int(void);
void pir_set_mode(uint8_t mode);

#endif
