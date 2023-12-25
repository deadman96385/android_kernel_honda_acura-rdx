#ifndef _NOR_ATTRIBUTES_H_
#define _NOR_ATTRIBUTES_H_

/* Maximum size in bytes for each attribute */
#define NOR_MAX_DATASIZE	(32)
#define NOR_INT_DATASIZE	(sizeof(int))

/* define some macros, just in case */
#define __expand(x) (x)
#define __stringify2(x)	(__stringify(x))

/* all the nor attributes, in one easy to change place */
#define PUBLIC_VERSION									public_version
#define PUBLIC_UPDATE_NUMBER						public_update_number
#define PUBLIC_CURRENT_VIN							current_vin
#define PUBLIC_ORIGINAL_VIN							original_vin
#define PUBLIC_ERROR_COUNT							public_error_count
#define PUBLIC_CHECKSUM									public_checksum

#define PRIVATE_VERSION									private_version
#define PRIVATE_BLUETOOTH_ADDRESS				bluetooth_address
#define PRIVATE_SERIAL_NUMBER						serial_number
#define PRIVATE_PART_NUMBER							part_number
#define PRIVATE_MANUFACTURE_DATE				manufacture_date
#define PRIVATE_DENSO_PART_NUMBER				denso_part_number
#define PRIVATE_DENSO_MANUFACTURE_DATE	denso_manufacture_date
#define PRIVATE_CHECKSUM								private_checksum
#define PRIVATE_HERE_ID						        here_id

#define BOOTFLAG_BOOTFLAG								bootflag
#define BOOTFLAG_PRODUCT								product
#define BOOTFLAG_PRODUCT_REVISION						product_revision
#define BOOTFLAG_MODEL									model

#define OTP_GARMIN_ID										garmin_id

/* nor attribute definition
 * @name: string naming the attribute
 * @offset: byte offset into the NOR partition
 * @size: byte size of the attribute
 */
typedef struct nor_entry{
	char* name;
	size_t offset;
	size_t size;
} nor_entry_t;

/* PUBLIC parameters data structure */
const static struct {
	nor_entry_t version;
	nor_entry_t update_number;
	nor_entry_t current_vin;
	nor_entry_t original_vin;
	nor_entry_t error_count;
	nor_entry_t checksum;
} public_nor =
{
	{__stringify2(PUBLIC_VERSION), 				0x00, NOR_MAX_DATASIZE},
	{__stringify2(PUBLIC_UPDATE_NUMBER),	0x20, NOR_INT_DATASIZE},
	{__stringify2(PUBLIC_CURRENT_VIN), 		0x60, NOR_MAX_DATASIZE},
	{__stringify2(PUBLIC_ORIGINAL_VIN),		0x80, NOR_MAX_DATASIZE},
	{__stringify2(PUBLIC_ERROR_COUNT),		0xA0, NOR_MAX_DATASIZE},
	{__stringify2(PUBLIC_CHECKSUM),				0xC0, NOR_INT_DATASIZE}
};
#define PUBLIC_NOR_SIZE (public_nor.checksum.offset + public_nor.checksum.size)

/* PRIVATE parameters data structure */
const static struct {
	nor_entry_t version;
	nor_entry_t bluetooth_address;
	nor_entry_t oem_serial_number;
	nor_entry_t oem_part_number;
	nor_entry_t oem_manufacture_date;
	nor_entry_t denso_part_number;
	nor_entry_t denso_manufacture_date;
	nor_entry_t checksum;
	nor_entry_t oem_here_id;
} private_nor =
{
	{__stringify2(PRIVATE_VERSION),									0x00, NOR_MAX_DATASIZE},
	{__stringify2(PRIVATE_BLUETOOTH_ADDRESS),				0x20, NOR_MAX_DATASIZE},
	{__stringify2(PRIVATE_SERIAL_NUMBER),						0x60, NOR_MAX_DATASIZE},
	{__stringify2(PRIVATE_PART_NUMBER),							0x80,	NOR_MAX_DATASIZE},
	{__stringify2(PRIVATE_MANUFACTURE_DATE),				0xA0, NOR_MAX_DATASIZE},
	{__stringify2(PRIVATE_DENSO_PART_NUMBER),				0xC0, NOR_MAX_DATASIZE},
	{__stringify2(PRIVATE_DENSO_MANUFACTURE_DATE),	0xD0, NOR_MAX_DATASIZE},
	{__stringify2(PRIVATE_CHECKSUM),								0xF0,	NOR_INT_DATASIZE},
	{__stringify2(PRIVATE_HERE_ID),						0x100,	NOR_MAX_DATASIZE}
};
#define PRIVATE_NOR_SIZE (private_nor.oem_here_id.offset + private_nor.oem_here_id.size)

/* BOOTFLAG parameters data structure */
const static struct {
	nor_entry_t bootflag;
	nor_entry_t product;
	nor_entry_t product_revision;
	nor_entry_t model;
} bootflag_nor = 
{
	{__stringify2(BOOTFLAG_BOOTFLAG), 0x00, NOR_MAX_DATASIZE},
	{__stringify2(BOOTFLAG_PRODUCT), 0x20, NOR_MAX_DATASIZE},
	{__stringify2(BOOTFLAG_PRODUCT_REVISION), 0x40, NOR_MAX_DATASIZE},
	{__stringify2(BOOTFLAG_MODEL), 0x60, NOR_MAX_DATASIZE}
};
#define BOOTFLAG_NOR_SIZE (bootflag_nor.model.offset + bootflag_nor.model.size)

/* OTP parameters data structure */
const static struct {
	nor_entry_t garmin_id;
} otp_nor =
{
	{__stringify2(OTP_GARMIN_ID), 0x00, NOR_MAX_DATASIZE}
};
#define OTP_NOR_SIZE (otp_nor.garmin_id.offset + otp_nor.garmin_id.size)

#endif /* #ifndef _NOR_ATTRIBUTES_H_ */
