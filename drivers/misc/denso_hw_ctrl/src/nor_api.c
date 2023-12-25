/* linux headers */
#include <linux/string.h>
#include <linux/mtd/mtd.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/crc32.h>

/* local headers */
#include "mtd_api.h"
#include "nor_attributes.h"
#include "nor_api.h"

static int array2int(char* source, size_t offset)
{
	int n = 0;
	n += ((int)source[offset + 0])<<24;
	n += ((int)source[offset + 1])<<16;
	n += ((int)source[offset + 2])<<8;
	n += ((int)source[offset + 3]);
	return n;
}

static void int2array(char* dest, size_t offset, int source)
{
	dest[offset + 0] = ((source>>24)&0x000000FF);
	dest[offset + 1] = ((source>>16)&0x000000FF);
	dest[offset + 2] = ((source>> 8)&0x000000FF);
	dest[offset + 3] = ((source    )&0x000000FF);
}

/* these functions arn't used if debugging
 *  so to eliminate warnings they are removed
 *  for debug builds.
 */
#ifndef DEBUG
static int read_version(char* data)
{
	int ret;
	int version;
	ret = sscanf(&data[public_nor.version.offset], "%d", &version);
	if (ret < 1) {
		version = -1;
	}
	return version;
}

static int read_update_number(char* data)
{
	return array2int(data, public_nor.update_number.offset);
}

static int read_crc32(char* data)
{
	return array2int(data, public_nor.checksum.offset);
}
#endif /* $ifndef DEBUG */

static void increase_update_number(char* data)
{
	int update = array2int(data, public_nor.update_number.offset);
	update += 1;
	int2array(data, public_nor.update_number.offset, update);
}

static void increase_error_count(char* data)
{
	int error = array2int(data, public_nor.error_count.offset);
	error += 1;
	int2array(data, public_nor.error_count.offset, error);
}

static void update_value(char* dest, size_t offset, const char* source, size_t length)
{
	memcpy(&dest[offset], source, length);
}

static int calculate_nor_entry_crc32(int seed, char* data, nor_entry_t entry)
{
	return crc32(seed, &data[entry.offset], min(strnlen(&data[entry.offset], entry.size) + 1, entry.size));
}

static int calculate_public_crc32(char* data)
{
	int crc = crc32(0, NULL, 0);
	crc = calculate_nor_entry_crc32(crc, data, public_nor.version);
	crc = calculate_nor_entry_crc32(crc, data, public_nor.update_number);
	crc = calculate_nor_entry_crc32(crc, data, public_nor.original_vin);
	crc = calculate_nor_entry_crc32(crc, data, public_nor.current_vin);
	return calculate_nor_entry_crc32(crc, data, public_nor.error_count);
}

static void update_public_crc32(char* data)
{
	int crc = calculate_public_crc32(data);
	int2array(data, public_nor.checksum.offset, crc);
}

static int calculate_private_crc32(char* data)
{
	int crc = crc32(0, NULL, 0);
	crc = calculate_nor_entry_crc32(crc, data, private_nor.version);
	crc = calculate_nor_entry_crc32(crc, data, private_nor.bluetooth_address);
	crc = calculate_nor_entry_crc32(crc, data, private_nor.oem_serial_number);
	crc = calculate_nor_entry_crc32(crc, data, private_nor.oem_part_number);
	crc = calculate_nor_entry_crc32(crc, data, private_nor.oem_manufacture_date);
	crc = calculate_nor_entry_crc32(crc, data, private_nor.denso_part_number);
	return calculate_nor_entry_crc32(crc, data, private_nor.denso_manufacture_date);
}

static void update_private_crc32(char *data)
{
	int crc = calculate_private_crc32(data);
	int2array(data, private_nor.checksum.offset, crc);
}

typedef enum public_partition_select
{
	SELECT_PRIMARY = 0,
	SELECT_SECONDARY,
	SELECT_BOTH,
	SELECT_NONE
} public_partition_select_t;

/* when in debug mode, always assume that both public partitions
 *  are valid; otherwise use the normal method.
 */
#ifdef DEBUG
static public_partition_select_t select_public_partition(char* p, char* s)
{
	return SELECT_BOTH;
}
#else /* #ifdef DEBUG */
static public_partition_select_t select_public_partition(char* p, char* s)
{
	public_partition_select_t selected_partition = SELECT_NONE;
	int primary_crc, secondary_crc;
	int primary_calculated_crc, secondary_calculated_crc;
	int primary_version, secondary_version;
	int primary_update_number, secondary_update_number;
	char unwritten_data[PUBLIC_NOR_SIZE];

	primary_crc = read_crc32(p);
	secondary_crc = read_crc32(s);
	primary_calculated_crc = calculate_public_crc32(p);
	secondary_calculated_crc = calculate_public_crc32(s);

	if (primary_crc == primary_calculated_crc &&
			 	secondary_crc == secondary_calculated_crc) {
		if (primary_crc == secondary_crc) {
			selected_partition = SELECT_BOTH;
		} else {
			primary_version = read_version(p);
			secondary_version = read_version(s);
			if (primary_version == secondary_version) {
				primary_update_number = read_update_number(p);
				secondary_update_number = read_update_number(s);
				if (primary_update_number == secondary_update_number) {
					selected_partition = SELECT_BOTH;
				} else if(primary_update_number > secondary_update_number) {
					selected_partition = SELECT_PRIMARY;
				} else {
					selected_partition = SELECT_SECONDARY;
				}
			} else if (primary_version > secondary_version) {
				selected_partition = SELECT_PRIMARY;
			} else {
				selected_partition = SELECT_SECONDARY;
			}
		}
	} else if (primary_crc == primary_calculated_crc) {
		selected_partition = SELECT_PRIMARY;
	} else if (secondary_crc == secondary_calculated_crc) {
		selected_partition = SELECT_SECONDARY;
	} else {
		/* if the public partition is totally unprogrammed
		 *   allow it to be programmed
		 */
		memset(unwritten_data, 0xFF, sizeof(unwritten_data));
		if (memcmp(unwritten_data, p, sizeof(unwritten_data)) == 0 &&
				memcmp(unwritten_data, s, sizeof(unwritten_data)) == 0) {
			selected_partition = SELECT_BOTH;
		} else {
			pr_crit("NOR API %s: everything is broken\n", __func__);
		}
	}
	return selected_partition;
}
#endif /* #ifdef DEBUG */

/********************/
/*** NOR Read API ***/
/********************/
/* honestly this still needs work, but no time left */
int NOR_PublicRead(int offset, char* buffer, size_t length)
{
	int plen, slen, ret = -1;
	char* primary = (char*)NULL;
	char* secondary = (char*)NULL;
	public_partition_select_t selected_partition;

	primary = (char*)kmalloc(PUBLIC_NOR_SIZE, GFP_KERNEL);
	secondary = (char*)kmalloc(PUBLIC_NOR_SIZE, GFP_KERNEL);
	if (primary && secondary) {
		plen = read_nor(PRIMARY, 0, primary, PUBLIC_NOR_SIZE);
		slen = read_nor(SECONDARY, 0, secondary, PUBLIC_NOR_SIZE);
		if (plen > 0 && slen > 0) {
			selected_partition = select_public_partition(primary, secondary);
			switch (selected_partition) {
			case SELECT_BOTH:
				ret = read_nor(PRIMARY, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
				if (ret < 0) {
					pr_err("NOR API %s: failed to read primary partition (%d)\n", __func__, ret);
				} else {
					ret = read_nor(SECONDARY, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
					if (ret < 0) {
						pr_err("NOR API %s: failed to read both partitions (%d)\n", __func__, ret);
					}
				}
				break;
			case SELECT_PRIMARY:
				ret = read_nor(PRIMARY, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
				if (ret < 0) {
					pr_err("NOR API %s: failed to read secondary partition (%d)\n", __func__, ret);
				}
				break;
			case SELECT_SECONDARY:
				ret = read_nor(SECONDARY, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
				if (ret < 0) {
					pr_err("NOR API %s: failed to read secondary partition (%d)\n", __func__, ret);
				}
				break;
			case SELECT_NONE:
			default:
				pr_err("NOR API %s: failed to select any partition\n", __func__);
				break;
			}
		} else if (plen > 0) {
			ret = read_nor(PRIMARY, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
			if (ret < 0) {
				pr_err("NOR API %s: failed to read primary partition (%d)\n", __func__, ret);
			}
		} else if (slen > 0) {
			ret = read_nor(SECONDARY, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
			if (ret < 0) {
				pr_err("NOR API %s: failed to read secondary partition (%d)\n", __func__, ret);
			}
		} else {
			pr_err("NOR API %s: failed to read any partition (%d) (%d)\n", __func__, plen, slen);
		}
	} else if (primary) {
		ret = read_nor(PRIMARY, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
		if (ret < 0) {
			pr_err("NOR API %s: failed to read primary partition (%d)\n", __func__, ret);
		}
	} else if (secondary) {
		ret = read_nor(SECONDARY, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
		if (ret < 0) {
			pr_err("NOR API %s: failed to read secondary partition (%d)\n", __func__, ret);
		}
	} else {
		pr_err("NOR API %s: no data access possible. incorrect device tree or partition data\n", __func__);
		ret = -1;
	}
	if (primary) kfree(primary);
	if (secondary) kfree(secondary);
	return ret;
}

int NOR_PrivateRead(int offset, char* buffer, size_t length)
{
	return read_nor(PRIVATE, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
}
EXPORT_SYMBOL(NOR_PrivateRead);

int NOR_BootflagRead(int offset, char* buffer, size_t length)
{
	return read_nor(BOOTFLAG, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
}

int NOR_OTPRead(int offset, char* buffer, size_t length)
{
	return read_otp(offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
}


/*********************/
/*** NOR Write API ***/
/*********************/
int NOR_PublicWrite(int offset, const char* buffer, size_t length)
{
	int plen, slen, ret;
	char* primary = (char*)NULL;
	char* secondary = (char*)NULL;
	public_partition_select_t selected_partition = SELECT_NONE;

	primary = (char*)kmalloc(PUBLIC_NOR_SIZE, GFP_KERNEL);
	secondary = (char*)kmalloc(PUBLIC_NOR_SIZE, GFP_KERNEL);
	if (primary && secondary) {
		plen = read_nor(PRIMARY, 0, primary, PUBLIC_NOR_SIZE);
		slen = read_nor(SECONDARY, 0, secondary, PUBLIC_NOR_SIZE);
		if (plen > 0 && slen > 0) {
			selected_partition = select_public_partition(primary, secondary);
			switch(selected_partition) {
			case SELECT_PRIMARY:
				increase_update_number(primary);
				increase_error_count(primary);
				update_value(primary, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
				update_public_crc32(primary);
				erase_nor(PRIMARY);
				ret = write_nor(PRIMARY, 0, primary, PUBLIC_NOR_SIZE);
				if (ret < 0) {
					pr_err("NOR API %s: failed to update primary partition (%d)\n", __func__, ret);
				}
				/* attempt o recover secondary partition */
				erase_nor(SECONDARY);
				slen = write_nor(SECONDARY, 0, primary, PUBLIC_NOR_SIZE);
				if (slen < 0) {
					pr_err("NOR API %s: failed to recover secondary partition (%d)\n", __func__, slen);
				}
				break;
			case SELECT_SECONDARY:
				increase_update_number(secondary);
				increase_error_count(secondary);
				update_value(secondary, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
				update_public_crc32(secondary);
				erase_nor(SECONDARY);
				ret = write_nor(SECONDARY, 0, secondary, PUBLIC_NOR_SIZE);
				if (ret < 0) {
					pr_err("NOR API %s: failed to update primary partition (%d)\n", __func__, ret);
				}
				/* attempt o recover primary partition */
				erase_nor(PRIMARY);
				plen = write_nor(PRIMARY, 0, secondary, PUBLIC_NOR_SIZE);
				if (plen < 0) {
					pr_err("NOR API %s: failed to recover secondary partition (%d)\n", __func__, plen);
				}
				break;
			case SELECT_BOTH:
				increase_update_number(primary);
				update_value(primary, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
				update_public_crc32(primary);
				erase_nor(PRIMARY);
				ret = write_nor(PRIMARY, 0, primary, PUBLIC_NOR_SIZE);
				if (ret < 0) {
					pr_err("NOR API %s: failed to update primary partition (%d)\n", __func__, ret);
				}
				erase_nor(SECONDARY);
				ret = write_nor(SECONDARY, 0, primary, PUBLIC_NOR_SIZE);
				if (ret < 0) {
					pr_err("NOR API %s: failed to update secondary partition (%d)\n", __func__, ret);
				}
				break;
			case SELECT_NONE:
			default:
				pr_crit("NOR API %s: no partition suitable for use\n", __func__);
				ret = -1;
				break;
			}
		} else if (plen > 0) {
			increase_update_number(primary);
			increase_error_count(primary);
			update_value(primary, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
			update_public_crc32(primary);
			erase_nor(PRIMARY);
			ret = write_nor(PRIMARY, 0, primary, PUBLIC_NOR_SIZE);
			if (ret < 0) {
				pr_err("NOR API %s: failed to update primary partition (%d)\n", __func__, ret);
			}
			/* attempt o recover secondary partition */
			erase_nor(SECONDARY);
			slen = write_nor(SECONDARY, 0, primary, PUBLIC_NOR_SIZE);
			if (slen < 0) {
				pr_err("NOR API %s: failed to recover secondary partition (%d)\n", __func__, slen);
			}
		} else if (slen > 0) {
			increase_update_number(secondary);
			increase_error_count(secondary);
			update_value(secondary, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
			update_public_crc32(secondary);
			erase_nor(SECONDARY);
			ret = write_nor(SECONDARY, 0, secondary, PUBLIC_NOR_SIZE);
			if (ret < 0) {
				pr_err("NOR API %s: failed to update primary partition (%d)\n", __func__, ret);
			}
			/* attempt o recover primary partition */
			erase_nor(PRIMARY);
			plen = write_nor(PRIMARY, 0, secondary, PUBLIC_NOR_SIZE);
			if (plen < 0) {
				pr_err("NOR API %s: failed to recover secondary partition (%d)\n", __func__, plen);
			}
		} else {
			pr_crit("NOR API %s: coudln't read anything (%d) (%d)\n", __func__, plen, slen);
			ret = -2;
		}
	} else if(primary) { /* failed to allocate memory for secondary partition */
		ret = read_nor(PRIMARY, 0, primary, PUBLIC_NOR_SIZE);
		if (ret < 0) {
			pr_err("NOR API %s: failed to read primary partition (%d)\n", __func__, ret);
		} else {
			increase_update_number(primary);
			increase_error_count(primary);
			update_value(primary, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
			update_public_crc32(primary);
			erase_nor(PRIMARY);
			ret = write_nor(PRIMARY, 0, primary, PUBLIC_NOR_SIZE);
			if (ret < 0) {
				pr_err("NOR API %s: failed to update primary partition (%d)\n", __func__, ret);
			}
			erase_nor(SECONDARY);
			slen = write_nor(SECONDARY, 0, primary, PUBLIC_NOR_SIZE);
			if (slen < 0) {
				pr_err("NOR API %s: failed to recover secondary partition (%d)\n", __func__, slen);
			}
		}
	} else if(secondary) { /* failed to allocate memory for primary partition */
		ret = read_nor(SECONDARY, 0, secondary, PUBLIC_NOR_SIZE);
		if (ret < 0) {
			pr_err("NOR API %s: failed to read secondary partition (%d)\n", __func__, ret);
		} else {
			increase_update_number(secondary);
			increase_error_count(secondary);
			update_value(secondary, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
			update_public_crc32(secondary);
			erase_nor(SECONDARY);
			ret = write_nor(SECONDARY, 0, secondary, PUBLIC_NOR_SIZE);
			if (ret < 0) {
				pr_err("NOR API %s: failed to update primary partition (%d)\n", __func__, ret);
			}
			erase_nor(PRIMARY);
			plen = write_nor(PRIMARY, 0, secondary, PUBLIC_NOR_SIZE);
			if (plen < 0) {
				pr_err("NOR API %s: failed to recover secondary partition (%d)\n", __func__, plen);
			}
		}
	} else { /* failed to allocate any memory */
		pr_crit("NOR API %s: failed to allocate memory 2 x (%d)\n", __func__, PUBLIC_NOR_SIZE);
		ret = -3;
	}
	if (primary) kfree(primary);
	if (secondary) kfree(secondary);
	return ret;
}

int NOR_PrivateWrite(int offset, const char* buffer, size_t length)
{
	int ret;
	char* private = (char*)NULL;
	private	= (char*)kmalloc(PRIVATE_NOR_SIZE, GFP_KERNEL);
	if (!private) {
		pr_crit("NOR API %s: failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	ret = read_nor(PRIVATE, 0, private, PRIVATE_NOR_SIZE);
	if (ret < 0) {
		pr_err("NOR API %s: failed to read private data (%d)\n", __func__, ret);
		ret = -1;
	} else {
		update_value(private, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
		update_private_crc32(private);
		erase_nor(PRIVATE);
		ret = write_nor(PRIVATE, 0, private, PRIVATE_NOR_SIZE);
		if (ret < 0) {
			pr_err("NOR API %s: failed to write private data (%d)\n", __func__, ret);
		}
	}
	if (private) kfree(private);
	return ret;
}

int NOR_BootflagWrite(int offset, const char* buffer, size_t length)
{
	int ret;
	char* bootflag = (char*)NULL;
	bootflag = (char*)kmalloc(NOR_PARTITION_SIZE, GFP_KERNEL);
	if (!bootflag) {
		pr_crit("NOR API %s: failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	ret = read_nor(BOOTFLAG, 0, bootflag, BOOTFLAG_NOR_SIZE);
	if (ret < 0) {
		pr_err("NOR API %s: failed to read bootflag data (%d)\n", __func__, ret);
		ret = -1;
	} else {
		update_value(bootflag, offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
		erase_nor(BOOTFLAG);
		ret = write_nor(BOOTFLAG, 0, bootflag, BOOTFLAG_NOR_SIZE);
		if (ret < 0) {
			pr_err("NOR API %s: failed to write bootflag data (%d)\n", __func__, ret);
		}
	}
	if (bootflag) kfree(bootflag);
	return ret;
}

int NOR_OTPWrite(int offset, const char* buffer, size_t length)
{
	/* can literally only do this once, no erasing! */
	return write_otp(offset, buffer, min((size_t)NOR_MAX_DATASIZE, length));
}
