/*
 * The purpose of the driver is to abstract the DENSO specific hardware 
 * attributes for access to the User Space.
 */
/* system headers */
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/printk.h>

/* mtd */
#include <linux/mtd/mtd.h>
#include <linux/err.h>

/* local headers */
#include "nor_attributes.h"
#include "nor_api.h"

#define MODULE_NAME "denso_hw_ctrl"

MODULE_ALIAS(MODULE_NAME);

/************************************/
/** Generic Show / Store Functions **/
/************************************/
/* constant for invalid data */
const static char INVALID[] = "INVALID";
const static char INVALID_ASCII[] = "????????????????????????????????";

/* show_ascii
 * designed to be used with an attribute function.
 * copies input buffer to output buffer as a string.
 * no safety checks are made, on purpose! If this fails the coder has
 *   done something very wrong.
 * 
 * @destination: destination buffer.
 * @source: source buffer
 * @source_length: length of the source buffer
 * 
 * @return: see return value for snprintf
 */
static ssize_t show_ascii( char* destination, char* source, size_t source_length )
{
	pr_err("%s: data (%s) length(%d)\n", __func__, source, source_length);
	/* convert invalid ASCII characters to valid (but nonsense) ASCII */
	if ((unsigned char)source[0] == 0xFF) { /* if any of it is invalid, it all is ~RP */
		return snprintf(destination, source_length, "%s", INVALID_ASCII);
	} else {
		return snprintf(destination, source_length, "%s", source);
	}
}

/* show_hex
 * designed to be used with an attribute function.
 * copies input buffer to output buffer as hexidecimal digits in ascii.
 * 		EX: 0xA9 -> "A9"
 * similar to show_ascii, no safty checks are performed; intentionally.
 * This function requires an destination buffer 2x the size of the source.
 * 
 * @destination: destination buffer.
 * @source: source buffer
 * @source_length: length of source buffer.
 * 
 * @return: length of string written to output buffer.
 */
static ssize_t show_hex( char* destination, char* source, size_t source_length )
{
	const char hex[] = "0123456789ABCDEF";
	int destination_index, source_index;
	
	pr_err("%s: ", __func__);
	for(source_index=0, destination_index=0; source_index < source_length; source_index++) {
		destination[destination_index++] = hex[(source[source_index]>>4) & 0xF];
		destination[destination_index++] = hex[(source[source_index]) & 0xF];
		pr_err("%c%c ", destination[destination_index-2], destination[destination_index-1]);
	}
	pr_err("\n");
	return destination_index;
}

/* store_ascii
 * designed to be used with an attribute function
 * copies source into destination as ascii text
 * no safety checks, intentionally.
 */
static ssize_t store_ascii(	char* destination, size_t destination_length,
														const char* source, size_t source_length)
{
	size_t i;
	size_t written;

	for (i = 0; i < destination_length-1 && i < source_length; i++)
		destination[i] = source[i];
	written = i + 1;
	for ( ; i < destination_length; i++)
		destination[i] = '\0';

	return written;
}

/* store_hex
 * designed to be used with an attribute function
 * copies source ascii into desintation as binary
 * no safety checks, intentionally.
 * --this turned out more complicated than desired--
 *  1. validate input: check to make sure all input is [0-9][a-f][A-F]
 * 			just throw out invalid strings, don't have time for that
 *	2. compenstate for odd number of input characters.
 * 			this prevents "12345" -> 0x12 0x34 0x05 (correct: 0x01 0x23 0x45)
 * @destination: pointer to destination array
 * @destination_length: length in bytes of destination_array
 * @source: pointer to source array
 * @source_length: length in bytes of source array
 * 
 * @return: length in bytes of characters copied
 */
static ssize_t store_hex( char* destination, size_t destination_length,
													const char* source, size_t source_length)
{
  int source_index;
  int destination_index;
  
  /* 2. compensate for odd number of input characters */
	if(source_length%2) { /* odd length, zero pad */
		destination[0] = '0';
		destination_index = 1;
	} else {
		destination_index = 0;
	}
	
	/* 1. validate input, quit if invalid character found */
	for (source_index = 0;
				(source_index < source_length) && (destination_index < destination_length);
				source_index++)
	{
		if (	(source[source_index] < '0') ||
					(source[source_index] > '9' && source[source_index]<'A') ||
					(source[source_index] > 'F' && source[source_index]<'a') ||
					(source[source_index] > 'f') )
		{
			pr_warning(MODULE_NAME " invalid input: (%c)\n", source[source_index]);
			return -1;
		} else {
			destination[destination_index] = source[source_index];
			destination_index++;
		}
	}
	return destination_index;
}

/*********************************/
/**					NOR Store						**/
/*********************************/
/* nor_store
 * uses the write_fn to write data into nor
 * @write_fn: function to write data to the nor
 * @offset: offset into the partition ( see nor_attributes.h )
 * @input: pointer to array to write
 * @input_length: length in bytes of the input array
 * 
 * @return: returns the amount of data written to the nor
 */
static ssize_t nor_store( const nor_write_fn_t write_fn, int offset, const char* input, size_t input_length)
{
	int writelen = 0;
	
	if (input == NULL) {
		pr_err("%s: error invalid buffer\n", __func__);
		return 0;
	}
	
	if (write_fn == NULL) {
		pr_err("%s: error invalid write function\n", __func__);
		return 0;
	}
	pr_err("%s: writing... data(%s) length(%d)\n", __func__, input, input_length);
	writelen = write_fn(offset, input, input_length);
	pr_err("%s: wrote %d bytes\n", __func__, writelen);
	if (writelen < 0) {
		pr_err("%s: error writing to nor: %d\n", __func__, writelen);
	}
	
	return writelen;
}

/* nor_ascii_store
 * processes data as ascii text, then stores it in nor
 * 
 * @offset: bytes into the parition to store the attribute data
 * @input: pointer to array
 * @input_length: length in bytes of input array
 * 
 * @return: size of data written into nor
 */
static ssize_t nor_ascii_store(const nor_write_fn_t write_fn, int offset, size_t length, const char* input)
{
	int writelen;
	char writedata[NOR_MAX_DATASIZE];

	if (length >= NOR_MAX_DATASIZE){
		length = NOR_MAX_DATASIZE - 1;
	}

	if (input == NULL) {
		pr_err("%s %s: invalid input pointer\n", MODULE_NAME, __func__);
		return 0;
	}
	
	memset(writedata, '\0', sizeof(writedata));
	writelen = store_ascii( writedata, sizeof(writedata), input, length);
	if (writelen < 0) {
		pr_err("%s: error converting to ascii: %d\n", __func__, writelen);
		return writelen;
	}
	pr_err("%s: writing %s (%d)\n", __func__, writedata, writelen);
	return nor_store( write_fn, offset, writedata, writelen);
} 

/* nor_hex_store
 * converts input to hexidecimal (binary), then stores it in nor
 * 
 * @offset:	bytes into the partition to store the attribute data
 * @input: pointer to array
 * @input_length: length in bytes of input array
 * 
 * @return: size of data written into nor
 */
static ssize_t nor_hex_store(const nor_write_fn_t write_fn, int offset, size_t length, const char* input)
{
	int writelen;
	char writedata[NOR_MAX_DATASIZE];

	if (input == NULL) {
		pr_err("%s %s: invalid input pointer\n", MODULE_NAME, __func__);
		return 0;
	}
	
	writelen = store_hex(writedata, sizeof(writedata), input, length);
	if (writelen < 0) {
		pr_err("%s: error converting to hex: %d\n", __func__, writelen);
		return writelen;
	}
	pr_err("%s: %s (%d)\n", __func__, writedata, writelen);
	return nor_store(write_fn, offset, writedata, writelen);
}

/****** PUBLIC NOR Attribute Store ******/
/* stores public nor data as ascii */
static ssize_t public_nor_ascii_store( int offset, size_t length, const char* input)
{
	return nor_ascii_store( NOR_PublicWrite, offset, length, input );
}

/* stores public nor data as hexidecimal */
static ssize_t public_nor_hex_store(int offset, size_t length, const char* input)
{
	return nor_hex_store( NOR_PublicWrite, offset, length, input );
}

/** All of these functions return count, because the kernel will send any remaining bytes forever
 * even if there is no intention or possibility of storing them.
 **/
/* write public version number to nor as ascii */
static ssize_t public_version_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = public_nor_ascii_store(public_nor.version.offset, min(public_nor.version.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/* write public update number to nor as hexidecimal */
static ssize_t public_update_number_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = public_nor_hex_store(public_nor.update_number.offset, min(public_nor.update_number.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/* write public current vin to nor as ascii */
static ssize_t current_vin_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = public_nor_ascii_store(public_nor.current_vin.offset, min(public_nor.current_vin.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/* write public original vin to nor as ascii */
static ssize_t original_vin_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = public_nor_ascii_store(public_nor.original_vin.offset, min(public_nor.original_vin.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/* write public error count to nor as hexidecimal */
static ssize_t public_error_count_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = public_nor_hex_store(public_nor.error_count.offset, min(public_nor.error_count.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/* write public checksum to nor as hexidecimal */
static ssize_t public_checksum_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = public_nor_hex_store(public_nor.checksum.offset, min(public_nor.checksum.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/****** PRIVATE NOR Attribute Store ******/
/* store private nor data as ascii */
static ssize_t private_nor_ascii_store(int offset, size_t length, const char* input)
{
	return nor_ascii_store( NOR_PrivateWrite, offset, length, input );
}

/* store private nor data as hexidecimal */
static ssize_t private_nor_hex_store(int offset, size_t length, const char* input)
{
	return nor_hex_store( NOR_PrivateWrite, offset, length, input );
}

/** All of these functions return count, because the kernel will send any remaining bytes forever
 * even if there is no intention or possibility of storing them.
 **/
/* write private version to nor as hexidecimal */
static ssize_t private_version_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = private_nor_ascii_store(private_nor.version.offset, min(private_nor.version.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/* write private bluetooth address to nor as ascii */
static ssize_t bluetooth_address_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = private_nor_ascii_store(private_nor.bluetooth_address.offset, min(private_nor.bluetooth_address.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/* write private OEM serial number to nor as ascii */
static ssize_t serial_number_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = private_nor_ascii_store(private_nor.oem_serial_number.offset, min(private_nor.oem_serial_number.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/* write private HERE ID to nor as ascii */
static ssize_t here_id_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = private_nor_ascii_store(private_nor.oem_here_id.offset, min(private_nor.oem_here_id.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/* write private OEM part number to nor as ascii */
static ssize_t part_number_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = private_nor_ascii_store(private_nor.oem_part_number.offset, min(private_nor.oem_part_number.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/* write private OEM manufacture date to nor as ascii */
static ssize_t manufacture_date_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = private_nor_ascii_store(private_nor.oem_manufacture_date.offset, min(private_nor.oem_manufacture_date.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/* write private DENSO part number to nor as ascii */
static ssize_t denso_part_number_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = private_nor_ascii_store(private_nor.denso_part_number.offset, min(private_nor.denso_part_number.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/* write private DENSO manufacture date to nor as ascii */
static ssize_t denso_manufacture_date_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = private_nor_ascii_store(private_nor.denso_manufacture_date.offset, min(private_nor.denso_manufacture_date.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/* write private checksum to nor as hexidecimal */
static ssize_t private_checksum_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = private_nor_hex_store(private_nor.checksum.offset, min(private_nor.checksum.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/****** BOOTFLAG NOR Attribute Store ******/
/* store private nor data as hexidecimal */
static ssize_t bootflag_nor_ascii_store(int offset, size_t length, const char* input)
{
	return nor_ascii_store( NOR_BootflagWrite, offset, length, input );
}

/* write data to bootflag */
static ssize_t bootflag_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = bootflag_nor_ascii_store(bootflag_nor.bootflag.offset, min(bootflag_nor.bootflag.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/****** PRODUCT NOR Attribute Store ******/
/* store private nor data as hexidecimal */
static ssize_t product_nor_ascii_store(int offset, size_t length, const char* input)
{
	return nor_ascii_store(NOR_BootflagWrite, offset, length, input );
}

/* write data to product */
static ssize_t product_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = product_nor_ascii_store(bootflag_nor.product.offset, min(bootflag_nor.product.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/****** PRODUCT_REVISION NOR Attribute Store ******/
/* store private nor data as hexidecimal */
static ssize_t product_revision_nor_ascii_store(int offset, size_t length, const char* input)
{
	return nor_ascii_store(NOR_BootflagWrite, offset, length, input );
}

/* write data to product_revision */
static ssize_t product_revision_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = product_nor_ascii_store(bootflag_nor.product_revision.offset, min(bootflag_nor.product_revision.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/****** MODEL NOR Attribute Store ******/
/* store private nor data as hexidecimal */
static ssize_t model_nor_ascii_store(int offset, size_t length, const char* input)
{
	return nor_ascii_store(NOR_BootflagWrite, offset, length, input );
}

/* write data to model */
static ssize_t model_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = model_nor_ascii_store(bootflag_nor.model.offset, min(bootflag_nor.model.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/****** OTP NOR Attribute Store ******/
/* store private nor data as hexidecimal */
static ssize_t otp_nor_ascii_store(int offset, size_t length, const char* input)
{
	return nor_ascii_store( NOR_OTPWrite, offset, length, input );
}

/* write garmin id to otp */
static ssize_t garmin_id_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int written = otp_nor_ascii_store(otp_nor.garmin_id.offset, min(otp_nor.garmin_id.size, count), buf);
	if (written < count) {
		pr_info("%s: only wrote %d bytes out of %d\n", __func__, written, count);
	}
	return count;
}

/*********************************/
/**					NOR Show 						**/
/*********************************/
/* read_from_nor
 * @read_fn: function of type nor_read_fn_t ( see nor_api.h )
 * 		This argument supplies the function used to read from nor.  Some
 *    partitions have different rules / algorithms and this allows
 *    the caller to specifiy which to use.
 * @offset: offset (in bytes) from the start of the partition.
 *    Each partition starts at 0x00.  Offset specifies where a parameter
 * 		starts with regard to the start of the partition.
 * @buffer: pointer to a string.
 * 		This is used to display the data retrieved.
 * @length: length of the supplied buffer.
 * 		Will only return a datasize up to the return length.
 * 
 * @return > 0: valid data returned, check *buffer for contents!
 * @return < 0: something went wrong, check stderr, and stdout.
 */
static ssize_t read_from_nor( nor_read_fn_t read_fn, int offset, char* output, size_t length)
{
	int readlen;
	
	if (output == NULL) {
		pr_err("%s: error invalid buffer\n", __func__);
		return -1;
	}
	
	if (read_fn == NULL) {
		pr_err("%s: error invalid read function\n", __func__);
		return -2;
	}
	
	memset(output, 0x00, length);
	readlen = read_fn(offset, output, length);
	if (readlen < 0) {
		pr_err("%s: error reading from nor: %d\n", __func__, readlen); 
		return readlen;
	}
	return readlen;
}

/* show_nor_ascii
 * this function reads a value from NOR using the supplied NOR read
 *   function.
 * The resulting data is then output as ascii text.
 * 	Ex: data{'H', 'I'} -> "HI"
 * 
 * @read_fn: nor read function. see nor_api.h for nor_read_fn_t def.
 * @offset: offset into the nor partition.  See nor_attributes.h.
 * @length: length of data requested.
 * @output: pointer to a string.
 * 
 * @return: number of bytes written to output
 */
static ssize_t show_nor_ascii( nor_read_fn_t read_fn, int offset, size_t length, char* output)
{
	int readlen;
	char readdata[NOR_MAX_DATASIZE];

	if (output == NULL) {
		pr_err("%s %s: invalud output pointer\n", MODULE_NAME, __func__);
		return 0;
	}
	
	memset(readdata, '\0', sizeof(readdata));
	readlen = read_from_nor( read_fn, offset, readdata, min(length, sizeof(readdata)) );
	if (readlen < 0) {
		pr_err("%s: error: %d\n", __func__, readlen);
		return snprintf(output, NOR_MAX_DATASIZE, "%s", INVALID);
	}
	pr_err("%s: data (%s) length(%d)\n", __func__, readdata, readlen);
	return show_ascii( output, readdata, readlen );
}

/* show_nor_hex
 * this function reads a value from NOR using the supplied NOR read
 *   function.
 * The resulting data is then output as ascii Hex.
 * 	Ex: data{0x32, 0x33} -> "3233"
 * 
 * @read_fn: nor read function. see nor_api.h for nor_read_fn_t def.
 * @offset: offset into the nor partition.  See nor_attributes.h.
 * @length: length of data requested.
 * @output: pointer to a string.
 * 
 * @return: number of bytes written to output
 */
static ssize_t show_nor_hex( nor_read_fn_t read_fn, int offset, size_t length, char* output)
{
	int readlen;
	char readdata[NOR_MAX_DATASIZE];

	if (output == NULL) {
		pr_err("%s %s: invalid output pointer\n", MODULE_NAME, __func__);
		return 0;
	}
	
	readlen = read_from_nor( read_fn, offset, readdata, min(length, sizeof(readdata)) );
	if (readlen < 0) {
		pr_err("%s: error: %d\n", __func__, readlen);
		return snprintf(output, NOR_MAX_DATASIZE, "%s", INVALID);
	}
	
	return show_hex( output, readdata, readlen );
}

/****** PUBLIC NOR Attribute Show ******/
/* read and show public nor data as ascii text */
static ssize_t public_nor_ascii_show( size_t offset, size_t length, char* output)
{
	return show_nor_ascii( NOR_PublicRead, offset, length, output );
}
/* read and show public nor data as hexidecimal */
static ssize_t public_nor_hex_show( size_t offset, size_t length, char* output)
{
	return show_nor_hex( NOR_PublicRead, offset, length, output);
}

/* show public nor version as hexidecimal text */
static ssize_t public_version_show(	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return public_nor_ascii_show(public_nor.version.offset, public_nor.version.size, buf);
}

/* show public update number as hexidecimal */
static ssize_t public_update_number_show(	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return public_nor_hex_show(public_nor.update_number.offset, public_nor.update_number.size, buf);
}

/* show public current vin as ascii */
static ssize_t current_vin_show(	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return public_nor_ascii_show(public_nor.current_vin.offset, public_nor.current_vin.size, buf);
}

/* show public original vin as ascii */
static ssize_t original_vin_show( struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return public_nor_ascii_show(public_nor.original_vin.offset, public_nor.original_vin.size, buf);
}

/* show public error count as hexidecimal */
static ssize_t public_error_count_show(	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return public_nor_hex_show(public_nor.error_count.offset, public_nor.error_count.size, buf);
}

/* show public checksum as hexidecimal */
static ssize_t public_checksum_show( struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return public_nor_hex_show(public_nor.checksum.offset, public_nor.checksum.size, buf);
}

/****** PRIVATE NOR Attribute Show ******/
/* read and show public nor data as ascii text */
static ssize_t private_nor_ascii_show( size_t offset, size_t length, char* output)
{
	return show_nor_ascii( NOR_PrivateRead, offset, length, output );
}
/* read and show public nor data as hexidecimal */
static ssize_t private_nor_hex_show( size_t offset, size_t length, char* output)
{
	return show_nor_hex( NOR_PrivateRead, offset, length, output);
}

/* show private nor version as hexidecimal */
static ssize_t private_version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return private_nor_ascii_show(private_nor.version.offset, private_nor.version.size, buf);
}

/* show private bluetooth address as ascii */
static ssize_t bluetooth_address_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return private_nor_ascii_show(private_nor.bluetooth_address.offset, private_nor.bluetooth_address.size, buf);
}

/* show private oem serial number as ascii */
static ssize_t serial_number_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return private_nor_ascii_show(private_nor.oem_serial_number.offset, private_nor.oem_serial_number.size, buf);
}

/* show private oem here id as ascii */
static ssize_t here_id_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return private_nor_ascii_show(private_nor.oem_here_id.offset, private_nor.oem_here_id.size, buf);
}

/* show private oem part number as ascii */
static ssize_t part_number_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return private_nor_ascii_show(private_nor.oem_part_number.offset, private_nor.oem_part_number.size, buf);
}

/* show private oem manufacture date as ascii */
static ssize_t manufacture_date_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return private_nor_ascii_show(private_nor.oem_manufacture_date.offset, private_nor.oem_manufacture_date.size, buf);
}

/* show private denso part number as ascii */
static ssize_t denso_part_number_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return private_nor_ascii_show(private_nor.denso_part_number.offset, private_nor.denso_part_number.size, buf);
}

/* show private denso manufacturing date as ascii */
static ssize_t denso_manufacture_date_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return private_nor_ascii_show(private_nor.denso_manufacture_date.offset, private_nor.denso_manufacture_date.size, buf);
}

/* show private checksum as hexidecimal */
static ssize_t private_checksum_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return private_nor_hex_show(private_nor.checksum.offset, private_nor.checksum.size, buf);
}

/****** BOOTFLAG NOR Attribute Show ******/
/* read and show bootflag nor data as hexidecimal */
static ssize_t bootflag_nor_ascii_show( size_t offset, size_t length, char* output)
{
	return show_nor_ascii( NOR_BootflagRead, offset, length, output);
}

static ssize_t bootflag_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return bootflag_nor_ascii_show(bootflag_nor.bootflag.offset, bootflag_nor.bootflag.size, buf);
}

/****** PRODUCT NOR Attribute Show ******/
/* read and show product nor data as hexidecimal */
static ssize_t product_nor_ascii_show( size_t offset, size_t length, char* output)
{
	return show_nor_ascii( NOR_BootflagRead, offset, length, output);
}

static ssize_t product_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return product_nor_ascii_show(bootflag_nor.product.offset, bootflag_nor.product.size, buf);
}

/****** PRODUCT_REVISION NOR Attribute Show ******/
/* read and show product_revision nor data as hexidecimal */
static ssize_t product_revision_nor_ascii_show( size_t offset, size_t length, char* output)
{
	return show_nor_ascii( NOR_BootflagRead, offset, length, output);
}

static ssize_t product_revision_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return product_revision_nor_ascii_show(bootflag_nor.product_revision.offset, bootflag_nor.product_revision.size, buf);
}

/****** MODEL NOR Attribute Show ******/
/* read and show bootflag nor data as hexidecimal */
static ssize_t model_nor_ascii_show( size_t offset, size_t length, char* output)
{
	return show_nor_ascii( NOR_BootflagRead, offset, length, output);
}

static ssize_t model_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return model_nor_ascii_show(bootflag_nor.model.offset, bootflag_nor.model.size, buf);
}


/****** OTP NOR Attribute Show ******/
/* read and show otp nor data as ascii */
static ssize_t otp_nor_ascii_show( size_t offset, size_t length, char* output)
{
	return show_nor_ascii( NOR_OTPRead, offset, length, output);
}

static ssize_t garmin_id_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return otp_nor_ascii_show(otp_nor.garmin_id.offset, otp_nor.garmin_id.size, buf);
}

/********************************/
/*** NOR Attribute Visibility ***/
/** These functions determine  **/
/**  the permissions of their  **/
/**  respective groups         **/
/********************************/
/* public_nor_is_visible
 * handles permission settings for public nor items.
 * current_vin - always read/write
 * original_vin - write once.  If the parameter reads as erased ( all 0xFF ).
 * 		The parameter is read/write.  
 * version / update number / error count / checksum - read only, unless in debug mode.
 * 
 * @return: read / write persmissions for the attribute set
 */
static
umode_t public_nor_is_visible(struct kobject *kobj, struct attribute *attribute, int index)
{
#ifdef DEBUG
	return (S_IWUSR | S_IRUGO);
#else /* #ifdef DEBUG */
	int read_length;
	char read_data[NOR_MAX_DATASIZE];
	char reset_data[NOR_MAX_DATASIZE];
	if (memcmp(attribute->name, __stringify2(PUBLIC_CURRENT_VIN), strlen(attribute->name)) == 0) {
		return (S_IWUSR | S_IRUGO);
	} else if (memcmp(attribute->name, __stringify2(PUBLIC_ORIGINAL_VIN), strlen(attribute->name)) == 0) {
		memset(read_data, '\0', sizeof(read_data));
		read_length = NOR_PublicRead(public_nor.original_vin.offset, read_data, sizeof(read_data));
		if (read_length < 0) {
			pr_err("error reading NOR: %d\n", read_length);
			return S_IRUGO;
		} else {
			pr_info("%s: length (%d)\n", __func__, read_length);
		}
		memset(reset_data, 0xFF, sizeof(reset_data));
		if (memcmp(read_data, reset_data, NOR_MAX_DATASIZE) == 0) { /* writable */
			return (S_IWUSR | S_IRUGO);
		} else { /* readonly */
			return S_IRUGO;
		}
	}
	return 0;
#endif /* #ifdef DEBUG */
}

/* private_nor_is_visible
 * handle permission settings for private nor items.
 * read only, unless built -DDEBUG
 */
static
umode_t private_nor_is_visible(struct kobject *kobj, struct attribute *attribute, int index)
{
#ifdef DEBUG
	return (S_IWUSR | S_IRUGO);
#else /* #ifdef DEBUG */
	if (	memcmp(attribute->name, __stringify2(PRIVATE_CHECKSUM), strlen(attribute->name)) == 0 ||
			 	memcmp(attribute->name, __stringify2(PRIVATE_VERSION), strlen(attribute->name)) == 0)
	{
		return 0;
	}
	return S_IRUGO;
#endif /* #ifdef DEBUG */
}

/* bootflag_nor_is_visible
 * handles permission settings for bootflag items.
 * always read / write.
 */
static
umode_t bootflag_nor_is_visible(struct kobject *kobj, struct attribute *attribute, int index)
{
	if ( memcmp(attribute->name, __stringify2(BOOTFLAG_BOOTFLAG), strlen(attribute->name)) == 0)
	{
		return (S_IWUSR | S_IRUGO);
	}
	else
	{
#ifdef DEBUG
	return (S_IWUSR | S_IRUGO);
#else /* #ifdef DEBUG */
	return S_IRUGO;
#endif /* #ifdef DEBUG */
	}
}

/* otp_nor_is_visible
 * handles permission settings for one time programable (otp) items.
 * always read-only, unless built with -DDEBUG
 */
static
umode_t otp_nor_is_visible(struct kobject *kobj, struct attribute *attribute, int index)
{
#ifdef DEBUG
	return (S_IWUSR | S_IRUGO);
#else /* #ifdef DEBUG */
	return S_IRUGO;
#endif
}

/*********************/
/*** KO Attributes ***/
/*********************/
/* All these attributes names are themselves stored in macros.
 * 	These __ATTR_X macros expand the easy to understand macros
 * 		into __ATTR with custom settings whose permissions can be upgraded
 * 		programatically.
 * 	While "slightly" harder to maintain, this approach allows
 * 		considerably more flexibility.
 */
#define __ATTR_EXPAND(_name) 	__ATTR(_name, 0, _name##_show, _name##_store)
#define __ATTR_UPGRADABLE(_name) __ATTR_EXPAND(_name)

/* Public NOR Attributes */
static struct kobj_attribute public_nor_version =
	__ATTR_UPGRADABLE(PUBLIC_VERSION);
static struct kobj_attribute public_nor_update_number =
	__ATTR_UPGRADABLE(PUBLIC_UPDATE_NUMBER);
static struct kobj_attribute public_nor_current_vin =
	__ATTR_UPGRADABLE(PUBLIC_CURRENT_VIN);
static struct kobj_attribute public_nor_original_vin =
	__ATTR_UPGRADABLE(PUBLIC_ORIGINAL_VIN);
static struct kobj_attribute public_nor_error_count =
	__ATTR_UPGRADABLE(PUBLIC_ERROR_COUNT);	
static struct kobj_attribute public_nor_checksum =
	__ATTR_UPGRADABLE(PUBLIC_CHECKSUM);	
	
/* Private NOR Attributes */
static struct kobj_attribute private_nor_version =
	__ATTR_UPGRADABLE(PRIVATE_VERSION);
static struct kobj_attribute private_nor_bluetooth_address =
	__ATTR_UPGRADABLE(PRIVATE_BLUETOOTH_ADDRESS);
static struct kobj_attribute private_nor_oem_serial =
	__ATTR_UPGRADABLE(PRIVATE_SERIAL_NUMBER);
static struct kobj_attribute private_nor_oem_part_number =
	__ATTR_UPGRADABLE(PRIVATE_PART_NUMBER);
static struct kobj_attribute private_nor_oem_manufacture_date =
	__ATTR_UPGRADABLE(PRIVATE_MANUFACTURE_DATE);
static struct kobj_attribute private_nor_denso_part_number =
	__ATTR_UPGRADABLE(PRIVATE_DENSO_PART_NUMBER);
static struct kobj_attribute private_nor_denso_manufacture_date =
	__ATTR_UPGRADABLE(PRIVATE_DENSO_MANUFACTURE_DATE);
static struct kobj_attribute private_nor_checksum =
	__ATTR_UPGRADABLE(PRIVATE_CHECKSUM);
static struct kobj_attribute private_nor_oem_here_id =
	__ATTR_UPGRADABLE(PRIVATE_HERE_ID);

/* Bootflag NOR Attributes */
static struct kobj_attribute bootflag_nor_bootflag =
	__ATTR_UPGRADABLE(BOOTFLAG_BOOTFLAG);
static struct kobj_attribute bootflag_nor_product =
	__ATTR_UPGRADABLE(BOOTFLAG_PRODUCT);
static struct kobj_attribute bootflag_nor_product_revision =
	__ATTR_UPGRADABLE(BOOTFLAG_PRODUCT_REVISION);
static struct kobj_attribute bootflag_nor_model =
	__ATTR_UPGRADABLE(BOOTFLAG_MODEL);

/* OTP NOR Attributes */
static struct kobj_attribute otp_garmin_id =
	__ATTR_UPGRADABLE(OTP_GARMIN_ID);


/**********************/
/** Attribute Groups **/
/**********************/
static struct attribute *public_nor_attrs[] = {
	&public_nor_version.attr,
	&public_nor_update_number.attr,
	&public_nor_current_vin.attr,
	&public_nor_original_vin.attr,
	&public_nor_error_count.attr,
	&public_nor_checksum.attr,
	NULL, /* sentinel */
};

static struct attribute *private_nor_attrs[] = {
	&private_nor_version.attr,
	&private_nor_bluetooth_address.attr,
	&private_nor_oem_serial.attr,
	&private_nor_oem_part_number.attr,
	&private_nor_oem_manufacture_date.attr,
	&private_nor_denso_part_number.attr,
	&private_nor_denso_manufacture_date.attr,
	&private_nor_checksum.attr,
	&private_nor_oem_here_id.attr,
	NULL, /* sentinel */
};

static struct attribute *bootflag_nor_attrs[] = {
	&bootflag_nor_bootflag.attr,
	&bootflag_nor_product.attr,
	&bootflag_nor_product_revision.attr,
	&bootflag_nor_model.attr,
	NULL, /* sentinel */
};

static struct attribute *otp_nor_attrs[] = {
	&otp_garmin_id.attr,
	NULL, /* sentinel */
};

static const struct attribute_group public_nor_attr_group = {
	.is_visible = public_nor_is_visible,
	.attrs = public_nor_attrs,
};

static const struct attribute_group private_nor_attr_group = {
	.is_visible = private_nor_is_visible,
	.attrs = private_nor_attrs,
};

static const struct attribute_group bootflag_nor_attr_group = {
	.is_visible = bootflag_nor_is_visible,
	.attrs = bootflag_nor_attrs,
};

static const struct attribute_group otp_nor_attr_group = {
	.is_visible = otp_nor_is_visible,
	.attrs = otp_nor_attrs,
};

static const struct attribute_group *attr_groups[] = {
	&public_nor_attr_group,
	&private_nor_attr_group,
	&bootflag_nor_attr_group,
	&otp_nor_attr_group,
	NULL,
};

static struct kobject *denso_hw_ctrl_kobj;

static int __init denso_hw_ctrl_init(void)
{
	int retval;

	/*
	 * Create a simple kobject with the name of MODULE_ALIAS,
	 * located under /sys/firmware/
	 *
	 * As this is a simple directory, no uevent will be sent to
	 * userspace.  That is why this function should not be used for
	 * any type of dynamic kobjects, where the name and number are
	 * not known ahead of time.
	 */
	denso_hw_ctrl_kobj = kobject_create_and_add(MODULE_NAME, firmware_kobj);
	if (!denso_hw_ctrl_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_groups(denso_hw_ctrl_kobj, attr_groups);
	if (retval)
	{
		kobject_put(denso_hw_ctrl_kobj);
	}
	return retval;
}

static void __exit denso_hw_ctrl_exit(void)
{
	kobject_put(denso_hw_ctrl_kobj);
}

late_initcall(denso_hw_ctrl_init);
module_exit(denso_hw_ctrl_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Robert Peters <robert_peters@denso-diam.com>");
MODULE_DESCRIPTION("Driver to control custom parts of DENSO J6 boards");
