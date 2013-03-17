/*
 * E-sky Hobby ET6I tx and rx.
 *
 * Licensed under AGPLv3.
 */

void rx_init(void);

/* Raw channels */
extern volatile uint16_t rx_ch[10];
/* Processed */
extern volatile uint8_t rx_no_signal;
/* Collective */
extern volatile uint8_t rx_co_throttle;
extern volatile uint8_t rx_co_right; /* aka. rudder */
/* Cyclic */
extern volatile uint8_t rx_cy_front; /* aka. elevator */
extern volatile uint8_t rx_cy_right; /* aka. ailerons */

/* ET6I custom controls */
extern volatile uint8_t rx_gyro_sw;
extern volatile uint8_t rx_right_pot;
extern volatile uint8_t rx_left_pot;
/* TGY9x custom controls, with ch5 as GEA & ID mixed */
extern volatile uint8_t rx_gear_sw;
extern volatile uint8_t rx_id_sw;
