#ifndef I2C_DEBUG_H
#define I2C_DEBUG_H

#include <linux/types.h>
#include <linux/time.h>

struct i2c_wtr_turnaround {
	spinlock_t lock;
	u64 kernel_time;
	u64 max_delta_t;
};

extern struct i2c_wtr_turnaround max_i2c_wtr_turnaround_0x55;

#endif
