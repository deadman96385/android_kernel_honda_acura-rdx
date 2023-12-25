#ifndef _DENSO_MMC_HEALTH_H
#define _DENSO_MMC_HEALTH_H
/*
 * Declarations and constants for eMMC health
 * data (Micron CMD56)
 *
 * Copyright (c) 2015 DENSO INTERNATIONAL AMERICA, INC. ALL RIGHTS RESERVED.
 */

/* name of the debugfs file which reports eMMC block erasure statistics */
#define DENSO_HEALTH_DEBUGFS_ERASURES   "health_block_erasures"
#define DENSO_HEALTH_DEBUGFS_BAD_BLOCKS "health_bad_blocks"

struct denso_mmc_health {
    u16     initial_bad_block_count;
    u16     runtime_bad_block_count;
    u16     remaining_spare_block_count;
};

#ifndef TRUE
#define TRUE (1)
#endif
#ifndef FALSE
#define FALSE (0)
#endif

int denso_report_block_erasure_data(struct mmc_host *, char **, size_t *, size_t *);
int denso_get_bad_block_counters(struct mmc_host *, struct denso_mmc_health *);
int denso_log_bad_block_details(struct mmc_host *, u8, u8);
#endif /* _DENSO_MMC_HEALTH_H */
