#ifndef _MTD_API_H_
#define _MTD_API_H_

typedef enum nor_partition_id
{
  PRIMARY = 0,
  SECONDARY,
  PRIVATE,
  BOOTFLAG,
  OTP
}nor_partition_id_t;

#define NOR_PARTITION_SIZE			0x00010000

/* Erase Nor data on a MTD partitoin
 *  @partition: Primary, Secondary, Readonly. Other values are ignored.
 */
void erase_nor( nor_partition_id_t partition );

/* Write data to a MTD partition
 *  @partition: Primary, Secondary, Readonly. Other values will be ignored.
 *  @offset: byte offset into partition. All partitions start at 0.
 *  @data: pointer to an array containing data to be written
 *  @dataSize: size of the data array to be written
 *
 *  @return
 *    0: successful
 *   -1: data was NULL or invalid
 *   -X: other error
 */
int write_nor( nor_partition_id_t partition, size_t offset, const char *data, size_t dataSize );

/* Read data from a MTD partition
 *  @partition: Primary, Secondary, or Readonly. Other values will be ignored.
 *  @offset: byte offset into partition. All partitions start at 0.
 *  @data: pointer to an array to contain read data
 *  @dataSize: maximum size of data in bytes.
 *
 *  @return
 *  >0: length of data read
 *  -1: data was NULL or invalid
 *  -X: some error.
 */
int read_nor( nor_partition_id_t partition, size_t offset, char *data, size_t dataSize );

/* Write data to OTP sections
 *   Operation is only allowed when OTP is unlocked.
 * @offset: byte offset into OTP section.  will generate error if it exceeds OTP length
 * @data: pointer to the data to be written
 * @size: size of data to be written. Will only write up to OTP length - offset bytes.
 *
 * @return >0: success, returns bytes written.
 * @return <0: error
 */
int write_otp(size_t offset, const char *data, size_t size);

/* Read data from OTP sections
 * @offset: byte offset into OTP sector. will generate error if it exeeds the OTP length
 * @data: pointer to an array to contain read data
 * @size: maximum size of requested data. (*data) is assumed to be of sufficient size to hold returned data.
 * 		Will only read up to OTP length - offset bytes.
 *
 * @return >0: success, retruns bytes read
 * @return <0: error
 */
int read_otp(size_t offset, char *data, size_t size);

/* Lock OTP Sector
 * @offset: byte offset into OTP sectors. Will lock the sector which contains the offset
 *
 * @return 0: success
 * @return <0: error, see logs
 */
int lock_otp(size_t offset);

#endif /* end mtd api */
