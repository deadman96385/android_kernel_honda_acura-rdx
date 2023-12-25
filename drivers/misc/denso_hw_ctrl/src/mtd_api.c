/* linux headers */
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mtd/mtd.h>
#include <linux/printk.h>

/* local headers */
#include "mtd_api.h"

/*********************/
/*** MTD INTERFACE ***/
/*********************/

/* NOR Parition Structure
 *   @name: name of the parition; as it appears in the device tree configuration
 *   @number: number offset of the partition. SPL is 0. First Usable NOR partition is 2.
 *              this number should match what is in the device tree under &qspi
 * Note: Might be able to use MTD interface to generate this?
 */
typedef struct nor_partition
{
  char* name;
  int number;
}nor_partition_t;

const nor_partition_t NOR_PARTITIONS[] =
{
  { /* Primary Partition */
    "QSPI.denso-primary",
    2
  },
  { /* Secondary Partition */
    "QSPI.denso-secondary",
    3
  },
  { /* Private Partition */
    "QSPI.denso-readonly",
    4
  },
  { /* Bootflag Partition */
		"QSPI.denso-bootflags",
		5
	},
	{ /* OTP Partition */
		"QSPI.denso-otp",
		6
	}
};

/* Open MTD Partition
 *  Open a MTD partition correspoding to one of the predefined partitions
 * @partition: Primary, Secondary, or Readonly.
 * @mtd: pointer to a struct of type mtd_info
 *
 * @return
 *  0: Successful
 * -1: MTD device error
 * -2: No device available
 * -3: Wrong device ( maybe device tree was altered? )
 */
static int open_nor( nor_partition_id_t partition, struct mtd_info **mtd )
{
  (*mtd) = get_mtd_device( NULL, NOR_PARTITIONS[partition].number );

  if(IS_ERR( (*mtd) )) {
      pr_err("MTD %s: device error (%s)\n", __func__, NOR_PARTITIONS[partition].name);
      return -1;
  }

  if( (*mtd)->type == MTD_ABSENT ) {
      pr_err("MTD %s: device not present\n", __func__);
      put_mtd_device( (*mtd) );
      return -2;
  }

  if( strcmp( (*mtd)->name, NOR_PARTITIONS[partition].name ) != 0 )
  {
    pr_err("MTD %s: wrong device\n", __func__);
    put_mtd_device( (*mtd) );
    return -3;
  }

  return 0;
}

/* Close mtd partition
 * @mtd: pointer to a valid mtd_info structure
 */
static void close_nor( struct mtd_info *mtd )
{
  if( mtd != NULL )
  {
    put_mtd_device( mtd );
  }
}

/* Erase a NOR Sector */
static void erase_callback(struct erase_info *instr)
{
  wake_up((wait_queue_head_t *)instr->priv);
}

static void erase_sector( struct mtd_info* mtd, unsigned int offset, unsigned int len )
{
  int ret;
  struct erase_info ei = {0};
  wait_queue_head_t waitq;
  DECLARE_WAITQUEUE(wait, current);

  init_waitqueue_head(&waitq);
  ei.addr = offset;
  ei.len = len;
  ei.mtd = mtd;
  ei.callback = erase_callback;
  ei.priv = (unsigned long)&waitq;

  ret = mtd_erase( mtd, &ei );
  if( ret < 0 ) {
    pr_err("MTD %s: failed (%d)\n", __func__, ret);
  } else if(ret == 0) {
    set_current_state(TASK_UNINTERRUPTIBLE);
    add_wait_queue(&waitq, &wait);
    if (ei.state != MTD_ERASE_DONE && ei.state != MTD_ERASE_FAILED)
        schedule();
    remove_wait_queue(&waitq, &wait);
    set_current_state(TASK_RUNNING);
  }
}

/***************/
/*** MTD API ***/
/***************/

/* Erase Nor data on a MTD partitoin
 *  @partition: Primary, Secondary, Readonly. Other values are ignored.
 */
void erase_nor( nor_partition_id_t partition )
{
  int ret;
  unsigned int start;
  struct mtd_info *mtd = NULL;

  ret = open_nor( partition, &mtd );
  if( ret == 0 ) {
    for(  start = 0; start < mtd->size; start += mtd->erasesize )
    {
      erase_sector( mtd, start, mtd->erasesize );
    }
  } else {
    pr_err("MTD %s: failed - could not open device (%d)\n", __func__, ret);
  }

  if( mtd != NULL ) {
    close_nor( mtd );
  }
}

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
/** TODO figure out how to make this faster... **/
int write_nor( nor_partition_id_t partition, size_t offset, const char *data, size_t dataSize )
{
  int ret;
  struct mtd_info *mtd = NULL;
  size_t retlen;

  if( data == NULL ) {
    pr_err("MTD %s: invalid source\n", __func__);
    return -1;
  }

  ret = open_nor( partition, &mtd );
  if( ret == 0 ) {
    /* write all the data */
    ret = mtd_write( mtd, offset, dataSize, &retlen, data );
    if( ret < 0 ) {
      pr_err("MTD %s: failed - couldn't write data (%d)\n", __func__, ret);
    } else {
      ret = (int)retlen;
    }
  } else {
    pr_err("MTD %s: failed - could not open device (%d)\n", __func__, ret);
  }

  if( mtd != NULL )
  {
    close_nor( mtd );
  }

  return ret;
}

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
int read_nor( nor_partition_id_t partition, size_t offset, char *data, size_t dataSize )
{
  int ret;
  size_t retlen;
  struct mtd_info *mtd = NULL;

  if( data == NULL ) {
    pr_err("MTD %s: invalid destination\n", __func__);
    return -1;
  }

  ret = open_nor( partition, &mtd );
  if( ret == 0 ) {
    ret = mtd_read( mtd, offset, dataSize, &retlen, data );
    if( ret < 0 ) {
      pr_err("MTD %s: failed (%d)\n", __func__, ret);
    } else {
      ret = (int)retlen; /* TODO meh */
    }

    close_nor( mtd );
  } else {
    pr_err("MTD %s: Invalid device (%d)\n", __func__, ret);
  }

  return ret;
}

/* Write data to OTP sections
 *   Operation is only allowed when OTP is unlocked.
 * @offset: byte offset into OTP section.  will generate error if it exceeds OTP length
 * @data: pointer to the data to be written
 * @size: size of data to be written. Will only write up to OTP length - offset bytes.
 *
 * @return >0: success, returns bytes written.
 * @return <0: error
 */
int write_otp(size_t offset, const char *data, size_t size)
{
  int ret;
  struct mtd_info *mtd = NULL;
  struct otp_info otp;
  size_t lockmask;
  size_t retlen;

  if (data == NULL) {
    pr_err("MTD %s: invalid source\n", __func__);
    return -1;
  }

  ret = open_nor(OTP, &mtd); //TODO idk if this is right...
  if (ret == 0) {
    lockmask = 0;
    lockmask = 1 << (offset / 32); /* 32bytes for each writable otp block */
    printk(KERN_CRIT "%s lockmask (%X)\n", __func__, lockmask);
    ret = mtd_get_user_prot_info(mtd, &otp, sizeof(otp));
    if (ret < 0) {
      pr_err("MTD %s: could not retrieve otp info (%d)\n", __func__, ret);
    } else if (otp.locked & lockmask) {
      pr_err("MTD %s: otp is locked (%X)\n", __func__, otp.locked);
      ret = -2;
    } else if (offset > otp.length) {
      pr_err("MTD %s: cannot write data outside otp. max(%d), target(%d)\n", __func__, otp.length, offset);
      ret = -3;
    } else {
			pr_info("MTD %s: otp lock (%X)\n", __func__, otp.locked);
			ret = mtd_write_user_prot_reg(mtd, offset, min(otp.length - offset, size), &retlen, (char*)data);
			if (ret < 0) {
				pr_err("MTD %s: failed (%d)\n", __func__, ret);
			} else {
				ret = (int)retlen;
				if (ret != size) {
					pr_info("MTD %s: only wrote (%d) of (%d) bytes\n", __func__, ret, size);
				}
				lock_otp(offset);
			}
		}
		close_nor(mtd);
	} else {
    pr_err("MTD %s: Invalid device (%d)\n", __func__, ret);
	}
	return ret;
}

/* Read data from OTP sections
 * @offset: byte offset into OTP sector. will generate error if it exeeds the OTP length
 * @data: pointer to an array to contain read data
 * @size: maximum size of requested data. (*data) is assumed to be of sufficient size to hold returned data.
 *    Will only read up to OTP length - offset bytes.
 *
 * @return >0: success, retruns bytes read
 * @return <0: error
 */
int read_otp(size_t offset, char *data, size_t size)
{
  int ret;
  struct mtd_info *mtd = NULL;
  struct otp_info otp;
  size_t retlen;

  if (data == NULL) {
    pr_err("MTD %s: invalid destination\n", __func__);
    return -1;
  }

  ret = open_nor(OTP, &mtd); //TODO idk if this is right...
  if (ret == 0) {
    ret = mtd_get_user_prot_info(mtd, &otp, sizeof(otp));
    if (ret < 0) {
      pr_err("MTD %s: could not retrieve otp info (%d)\n", __func__, ret);
    } else if (offset > otp.length) {
      pr_err("MTD %s: cannot read data outside otp sector. max(%d), target(%d)\n", __func__, otp.length, offset);
      ret = -2;
    } else {
      ret = mtd_read_user_prot_reg(mtd, offset, min(otp.length - offset, size), &retlen, data);
      if (ret < 0) {
        pr_err("MTD %s: failed (%d)\n", __func__, ret);
      } else {
        ret = (int)retlen;
        if (ret != size) {
          pr_info("MTD %s: only read (%d) of requested (%d) bytes\n", __func__, ret, size);
        }
      }
    }
    close_nor(mtd);
  } else {
    pr_err("MTD %s: Invalid device (%d)\n", __func__, ret);
  }
  return ret;
}


/* Lock OTP Sector
 * @offset: byte offset into OTP sectors. Will lock the sector which contains the offset
 *
 * @return 0: success
 * @return <0: error, see logs
 */
int lock_otp(size_t offset)
{
  struct otp_info otp;
  size_t lockbits = 0;
  struct mtd_info *mtd = NULL;
  int ret = -1;

  ret = open_nor(OTP, &mtd);
  if (ret == 0) {
    lockbits = 0;
    lockbits = 1 << (offset / 32); /* 32 bytes per OTP sector */
    ret = mtd_get_user_prot_info(mtd, &otp, sizeof(otp));
    if (ret < 0) {
      pr_err("MTD %s: failed to read otp informaiton (%d)\n", __func__, ret);
    } else {
      lockbits |= otp.locked;
      ret = mtd_lock_user_prot_reg(mtd, 0x10, lockbits); /* byte 0x10 is start of 4 lockbit registers */
      if (ret < 0) {
        pr_err("MTD %s: failed to lock user protected register (%d)\n", __func__, ret);
      } else {
        ret = 0;
      }
    }
    close_nor(mtd);
  } else {
    pr_err("MTD %s: Invalid device (%d)\n", __func__, ret);
  }
  return ret;
}
