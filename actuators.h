/*
 * Generates the signal that drives RC actuators (motors and servos).
 *
 * Licensed under AGPLv3.
 */

extern volatile uint16_t actuators[8];

void actuators_init(int devices);
void actuators_start(void);

static inline void actuator_set(uint8_t target, uint16_t value) {
	actuators[target] = value;
}
