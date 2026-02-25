/*
 * Copyright (c) Honor Technologies Co., Ltd. 2020. All rights reserved.
 * Description: hyperhold implement
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author:	He Biao <hebiao6.>
 *		Wang Cheng Ke <wangchengke2.>
 *		Wang Fa <fa.wang.>
 *
 * Create: 2020-4-16
 *
 */
#define pr_fmt(fmt) "[HYPERHOLD]" fmt

#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/atomic.h>
#include <linux/memcontrol.h>
#include <linux/version.h>
#include <securec.h>
#include "zram_drv.h"
#include "hyperhold.h"
#include "hyperhold_internal.h"

#ifdef CONFIG_HYPERHOLD_GKI
#include <linux/kernel_read_file.h>
#include <linux/string.h>
#include "hyperhold_gki_zswapd.h"
#endif

#ifdef CONFIG_HYPERHOLD_FILE_ARCHIVAL
#include <uapi/linux/loop.h>
#include <linux/fs.h>
#include <linux/mount.h>
#include <linux/file.h>
#include <linux/dcache.h>
#include <linux/types.h>
#include <linux/syscalls.h>
#include <linux/init_syscalls.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/namei.h>
#include <linux/string.h>
#endif

#define HYPERHOLD_WDT_EXPIRE_DEFAULT 4 * 3600
#define PRE_EOL_INFO_OVER_VAL 2
#define LIFE_TIME_EST_OVER_VAL 8
#define HYPERHOLD_BLOCK_DEV_NAME "/dev/block/by-name/hyperhold"

#ifdef CONFIG_HYPERHOLD_GKI
extern int get_ufs_health_info(u8 *pre_eol_info, u8 *life_time_est_a,
			       u8 *life_time_est_b);
#endif
struct hyperhold_cfg global_settings;

void *hyperhold_malloc(size_t size, bool fast, bool nofail)
{
	void *mem = NULL;

	if (likely(fast)) {
		mem = kzalloc(size, GFP_ATOMIC);
		if (likely(mem || !nofail))
			return mem;
	}

	mem = kzalloc(size, GFP_NOIO);

	return mem;
}

void hyperhold_free(const void *mem)
{
	kfree(mem);
}

static struct page *hyperhold_alloc_page_common(void *data, gfp_t gfp)
{
	struct page *page = NULL;
	struct zs_ext_para *ext_para = (struct zs_ext_para *)data;

	if (ext_para->pool) {
		spin_lock(&ext_para->pool->page_pool_lock);
		if (!list_empty(&ext_para->pool->page_pool_list)) {
			page = list_first_entry(&ext_para->pool->page_pool_list,
						struct page, lru);
			list_del(&page->lru);
		}
		spin_unlock(&ext_para->pool->page_pool_lock);
	}

	if (!page) {
		if (ext_para->fast) {
			page = alloc_page(GFP_ATOMIC);
			if (likely(page))
				goto out;
		}
		if (ext_para->nofail)
			page = alloc_page(GFP_NOIO);
		else
			page = alloc_page(gfp);
	}
out:
	return page;
}

#ifndef CONFIG_HYPERHOLD_GKI
static size_t hyperhold_zsmalloc_parse(void *data)
{
	struct zs_ext_para *ext_para = (struct zs_ext_para *)data;

	return ext_para->alloc_size;
}
#endif

unsigned long hyperhold_zsmalloc(struct zs_pool *zs_pool, size_t size,
				 struct hyperhold_page_pool *pool)
{
	gfp_t gfp = __GFP_DIRECT_RECLAIM | __GFP_KSWAPD_RECLAIM | __GFP_NOWARN |
		    __GFP_HIGHMEM | __GFP_MOVABLE;
#ifdef CONFIG_ZS_MALLOC_EXT
	struct zs_ext_para ext_para;
	unsigned long ret;

	ext_para.alloc_size = size;
	ext_para.pool = pool;
	ext_para.fast = true;
	ext_para.nofail = true;
	ret = zs_malloc(zs_pool, (size_t)(&ext_para), gfp);
	if (!ret)
		hh_print(HHLOG_ERR,
			 "alloc handle failed, size = %lu, gfp = %d\n", size,
			 gfp);

	return ret;
#else
	return zs_malloc(zs_pool, size, gfp);
#endif
}

unsigned long zram_zsmalloc(struct zs_pool *zs_pool, size_t size, gfp_t gfp)
{
#ifdef CONFIG_ZS_MALLOC_EXT
	unsigned long ret;
	struct zs_ext_para ext_para;

	if (!is_ext_pool(zs_pool))
		return zs_malloc(zs_pool, size, gfp);

	ext_para.alloc_size = size;
	ext_para.pool = NULL;
	ext_para.fast = false;
	ext_para.nofail = false;
	ret = zs_malloc(zs_pool, (size_t)(&ext_para), gfp);
	if (!ret && (gfp | GFP_NOIO) == GFP_NOIO)
		hh_print(HHLOG_ERR,
			 "alloc handle failed, size = %lu, gfp = %d\n", size,
			 gfp);

	return ret;
#else
	return zs_malloc(zs_pool, size, gfp);
#endif
}

struct page *hyperhold_alloc_page(struct hyperhold_page_pool *pool, gfp_t gfp,
				  bool fast, bool nofail)
{
	struct zs_ext_para ext_para;

	ext_para.pool = pool;
	ext_para.fast = fast;
	ext_para.nofail = nofail;

	return hyperhold_alloc_page_common((void *)&ext_para, gfp);
}

void hyperhold_page_recycle(struct page *page, struct hyperhold_page_pool *pool)
{
	if (pool) {
		spin_lock(&pool->page_pool_lock);
		list_add(&page->lru, &pool->page_pool_list);
		spin_unlock(&pool->page_pool_lock);
	} else {
		__free_page(page);
	}
}

int hyperhold_loglevel(void)
{
	return global_settings.log_level;
}

static void hyperhold_wdt_expire_set(unsigned long expire)
{
	global_settings.wdt_expire_s = expire;
}

static void hyperhold_wdt_set_enable(bool en)
{
	atomic_set(&global_settings.watchdog_protect, en ? 1 : 0);
}

static bool hyperhold_wdt_enable(void)
{
	return !!atomic_read(&global_settings.watchdog_protect);
}

bool hyperhold_reclaim_in_enable(void)
{
	return !!atomic_read(&global_settings.reclaim_in_enable);
}

static void hyperhold_set_reclaim_in_disable(void)
{
	atomic_set(&global_settings.reclaim_in_enable, false);
}

static void hyperhold_set_reclaim_in_enable(bool en)
{
	del_timer_sync(&global_settings.wdt_timer);
	atomic_set(&global_settings.reclaim_in_enable, en ? 1 : 0);
	if (en && hyperhold_wdt_enable())
		mod_timer(&global_settings.wdt_timer,
			  jiffies + msecs_to_jiffies(
					    global_settings.wdt_expire_s *
					    MSEC_PER_SEC));
}

bool hyperhold_enable(void)
{
	return !!atomic_read(&global_settings.enable);
}
EXPORT_SYMBOL(hyperhold_enable);

static void hyperhold_set_enable(bool en)
{
	hyperhold_set_reclaim_in_enable(en);

	if (!hyperhold_enable())
		atomic_set(&global_settings.enable, en ? 1 : 0);
}

bool hyperhold_crypto_enable(void)
{
	return !!atomic_read(&global_settings.crypto_enable);
}

void hyperhold_set_crypto_enable(bool en)
{
	if (!hyperhold_crypto_enable())
		atomic_set(&global_settings.crypto_enable, en ? 1 : 0);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 61)
static void hyperhold_wdt_timeout(struct timer_list *timer)
#else
static void hyperhold_wdt_timeout(unsigned long data)
#endif
{
	hh_print(HHLOG_ERR,
		 "hyperhold wdt is triggered! Hyperhold is disabled!\n");
	hyperhold_set_reclaim_in_disable();
}

void hyperhold_close_bdev(struct block_device *bdev, struct file *backing_dev)
{
	if (bdev)
		blkdev_put(bdev, FMODE_READ | FMODE_WRITE | FMODE_EXCL);

	if (backing_dev)
		filp_close(backing_dev, NULL);
}

static struct file *hyperhold_open_bdev(const char *file_name)
{
	struct file *backing_dev = NULL;

#ifdef CONFIG_HYPERHOLD_GKI
	backing_dev = filp_open_block(file_name, O_RDWR | O_LARGEFILE, 0);
#else
	backing_dev = filp_open(file_name, O_RDWR | O_LARGEFILE, 0);
#endif
	if (unlikely(IS_ERR(backing_dev))) {
		hh_print(HHLOG_ERR, "open the %s failed! eno = %ld\n",
			 file_name, PTR_ERR(backing_dev));
		backing_dev = NULL;
		return NULL;
	}

	if (unlikely(!S_ISBLK(backing_dev->f_mapping->host->i_mode))) {
		hh_print(HHLOG_ERR, "%s isn't a blk device\n", file_name);
		hyperhold_close_bdev(NULL, backing_dev);
		return NULL;
	}

	return backing_dev;
}

struct hyperhold_bind_info {
	struct file *backing_dev;
	unsigned long nr_pages;
	struct block_device *bdev;
	char bdev_name[HYPERHOLD_NAME_LEN];
};

static int hyperhold_bind(struct zram *zram, const char *file_name,
			  struct hyperhold_bind_info *bind_info)
{
	struct file *backing_dev = NULL;
	struct inode *inode = NULL;
	unsigned long nr_pages;
	struct block_device *bdev = NULL;
	int err;

	backing_dev = hyperhold_open_bdev(file_name);
	if (unlikely(!backing_dev))
		return -EINVAL;

	inode = backing_dev->f_mapping->host;

	bdev = blkdev_get_by_dev(inode->i_rdev,
				 FMODE_READ | FMODE_WRITE | FMODE_EXCL, zram);
	if (IS_ERR(bdev)) {
		err = PTR_ERR(bdev);
		hh_print(HHLOG_ERR, "%s blkdev_get_by_dev failed! eno = %d\n",
			 file_name, err);
		bdev = NULL;
		goto out;
	}
	nr_pages = (unsigned long)i_size_read(inode) >> PAGE_SHIFT;
	err = set_blocksize(bdev, PAGE_SIZE);
	if (unlikely(err)) {
		hh_print(HHLOG_ERR, "%s set blocksize failed! eno = %d\n",
			 file_name, err);
		goto out;
	}

	bind_info->bdev = bdev;
	bind_info->backing_dev = backing_dev;
	bind_info->nr_pages = nr_pages;
	snprintf(bind_info->bdev_name, HYPERHOLD_NAME_LEN, "%s", file_name);

	return 0;
out:
	hyperhold_close_bdev(bdev, backing_dev);

	return err;
}

#ifdef CONFIG_HYPERHOLD_FILE_ARCHIVAL
ssize_t hyperhold_space_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t len)
{
	bool ret;
	ret = hyperhold_space_split_param(buf, len);

	if (!ret)
		hh_print(HHLOG_ERR, "val error %s", buf);

	return len;
}

ssize_t hyperhold_space_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return snprintf(buf, PAGE_SIZE,
			"hyperhold space:%s\n"
			"hyperhold space size:%llu\n"
			"hyperhold partition size:%llu\n"
			"hyperhold partition threshold:%llu\n"
			"hyperhold file threshold:%llu\n",
			hyperhold_space_is_file_arch() ? "partition_file" :
							 "partition",
			hyperhold_space_get_total_sz(),
			hyperhold_space_get_partition_sz(),
			hyperhold_get_partition_threshold(),
			hyperhold_get_file_threshold());
}

static void hyperhold_backingbdev_init(u32 index)
{
	struct hyperhold_space *hp_bdev = NULL;

	if (index >= HP_SPACE_MAX_NR)
		return;
	hp_bdev = &global_settings.backing_bdev[index];

	hp_bdev->bdev = NULL;
	hp_bdev->backing_dev = NULL;
	memset(hp_bdev->file_name, 0, HYPERHOLD_NAME_LEN);
	memset(hp_bdev->bdev_name, 0, HYPERHOLD_NAME_LEN);
	hp_bdev->nr_pages = 0;
	hp_bdev->start_sector = 0;
	hp_bdev->end_sector = 0;
	hp_bdev->start = 0;
	hp_bdev->end = 0;
	hp_bdev->pddr_continuous = false;
}

static void hyperhold_backingbdev_deinit(u32 index)
{
	struct hyperhold_space *hp_bdev = NULL;

	if (index >= HP_SPACE_MAX_NR)
		return;

	hp_bdev = &global_settings.backing_bdev[index];
	if (hp_bdev->backing_dev)
		filp_close(hp_bdev->backing_dev, NULL);
	hp_bdev->backing_dev = NULL;

	if (hp_bdev->bdev && hp_bdev->pddr_continuous) {
		blkdev_put(hp_bdev->bdev, FMODE_READ | FMODE_WRITE);
	} else if (hp_bdev->bdev) {
		blkdev_put(hp_bdev->bdev,
			   FMODE_READ | FMODE_WRITE | FMODE_EXCL);
	}
	hp_bdev->bdev = NULL;

	memset(hp_bdev->file_name, 0, HYPERHOLD_NAME_LEN);
	memset(hp_bdev->bdev_name, 0, HYPERHOLD_NAME_LEN);
	hp_bdev->nr_pages = 0;
	hp_bdev->start_sector = 0;
	hp_bdev->end_sector = 0;
	hp_bdev->start = 0;
	hp_bdev->end = 0;
}

static int hyperhold_alloc_loopbdev(const char *loopbdev_name,
				    const char *file_name, u64 size,
				    struct hyperhold_space *hp_bdev)
{
	struct inode *inode = NULL;
	int ret = 0;
	unsigned long nr_pages;
	struct block_device *loop_bdev = NULL;
	struct file *backing_dev = NULL;

	backing_dev = filp_open_block(loopbdev_name, O_RDWR | O_LARGEFILE, 0);
	if (IS_ERR(backing_dev)) {
		hh_print(HHLOG_ERR, "open file %s failed! eno = %ld\n",
			 loopbdev_name, PTR_ERR(backing_dev));
		goto out;
	}
	inode = backing_dev->f_mapping->host;
	loop_bdev = blkdev_get_by_dev(inode->i_rdev,
				      FMODE_READ | FMODE_WRITE | FMODE_EXCL,
				      global_settings.zram);
	if (IS_ERR(loop_bdev)) {
		ret = PTR_ERR(loop_bdev);
		hh_print(HHLOG_ERR, "%s blkdev_get_by_dev failed! eno = %d\n",
			 loopbdev_name, ret);
		goto fileclose;
	}
	nr_pages = i_size_read(inode) >> PAGE_SHIFT;
	if (nr_pages != (size >> PAGE_SHIFT)) {
		hh_print(HHLOG_ERR, "%s size error, page %lu\n", loopbdev_name,
			 nr_pages);
		goto loopdevput;
	}

	hp_bdev->bdev = loop_bdev;
	hp_bdev->backing_dev = backing_dev;
	snprintf(hp_bdev->file_name, HYPERHOLD_NAME_LEN, "%s", file_name);
	snprintf(hp_bdev->bdev_name, HYPERHOLD_NAME_LEN, "%s", loopbdev_name);
	hp_bdev->nr_pages = nr_pages;
	hp_bdev->start_sector = 0;
	hp_bdev->end_sector = nr_pages * HYPERHOLD_PAGE_SIZE_SECTOR;
	hp_bdev->start = 0;
	hp_bdev->end = nr_pages;
	hp_bdev->pddr_continuous = false;
	goto out;

loopdevput:
	blkdev_put(loop_bdev, FMODE_READ | FMODE_WRITE | FMODE_EXCL);
fileclose:
	filp_close(backing_dev, NULL);
out:
	return ret;
}

static int do_hyperhold_space_alloc_loopbdev(const char *bdev_name,
					     const char *file_name, u32 index)
{
	int ret = -1;
	struct hyperhold_space *hp_bdev = &global_settings.backing_bdev[index];

	hh_print(HHLOG_DEBUG, "loopdev: %s file: %s, size: %llu, index:%d\n",
		 bdev_name, file_name, loopdev_info.size, index);

	ret = hyperhold_alloc_loopbdev(bdev_name, file_name, loopdev_info.size,
				       hp_bdev);
	if (unlikely(ret)) {
		hh_print(HHLOG_ERR, "hyperhold_alloc_loopbdev ret = %d\n", ret);
		goto out;
	}

	hh_print(
		HHLOG_DEBUG,
		"start_sector = %llu, end_sector = %llu, start = %lu, end = %lu, nr_pages %lu\n",
		hp_bdev->start_sector, hp_bdev->end_sector, hp_bdev->start,
		hp_bdev->end, hp_bdev->nr_pages);
out:
	return ret;
}

static int hyperhold_space_alloc_loopbdev(u32 index)
{
	int ret;

	if (!hyperhold_space_info_flag(HP_SPACE_INFO_LOOP_DEV_INITED)) {
		hh_print(HHLOG_ERR, "loopbdev info not inited");
		return -EINVAL;
	}

	if (index != loopdev_info.index) {
		hh_print(HHLOG_ERR, "alloc loopbdev error %u", index);
		return -EINVAL;
	}

	ret = do_hyperhold_space_alloc_loopbdev(loopdev_info.bdev_name,
						loopdev_info.file_name,
						loopdev_info.index);
	if (unlikely(ret)) {
		hh_print(HHLOG_ERR,
			 "get %llu non-continuous loopdev %s failed\n",
			 loopdev_info.index, loopdev_info.file_name);
		hyperhold_backingbdev_deinit(loopdev_info.index);
	} else {
		hh_print(HHLOG_DEBUG,
			 "get %llu non-continuous loopdev %s success\n",
			 loopdev_info.index, loopdev_info.file_name);
	}

	return ret;
}

#ifndef CONFIG_HYPERHOLD_GKI
static int hyperhold_get_bdev(const char *bdev_name, const char *file_name,
			      struct hyperhold_space *hp_bdev)
{
	struct file *backing_dev = NULL;
	struct block_device *bdev = NULL;
	struct inode *inode = NULL;
	int err;

	backing_dev = hyperhold_open_bdev(bdev_name);
	if (!backing_dev) {
		return -EINVAL;
	}

	inode = backing_dev->f_mapping->host;
	bdev = blkdev_get_by_dev(inode->i_rdev, FMODE_READ | FMODE_WRITE, NULL);
	if (IS_ERR(bdev)) {
		err = PTR_ERR(bdev);
		hh_print(HHLOG_ERR,
			 "%s blkdev_get_by_dev failed! eno = %d, holder %p\n",
			 file_name, err, global_settings.zram);
		bdev = NULL;
		goto out;
	}

	hp_bdev->bdev = bdev;
	hp_bdev->backing_dev = backing_dev;
	snprintf(hp_bdev->bdev_name, HYPERHOLD_NAME_LEN, "%s", bdev_name);

	return 0;
out:
	filp_close(backing_dev, NULL);
	return err;
}

static int hyperhold_get_continuous_bdev_info(struct hyperhold_space *hp_bdev)
{
	unsigned long start, end;
	unsigned long long size;

	start = dev_info.start;
	end = dev_info.end;
	size = dev_info.size;
	if (end - start != (size >> PAGE_SHIFT) - 1) {
		hh_print(
			HHLOG_ERR,
			"hyperhold file error start %lu, end %lu, should be %llu",
			start, end, (size >> PAGE_SHIFT) - 1);
		goto out;
	}
	hp_bdev->start_sector = start << BLOCK_SECTOR_SHIFT;
	hp_bdev->end_sector = end << BLOCK_SECTOR_SHIFT;
	hp_bdev->start = start;
	hp_bdev->end = end;
	hp_bdev->nr_pages = size >> PAGE_SHIFT;
	snprintf(hp_bdev->file_name, HYPERHOLD_NAME_LEN, "%s",
		 dev_info.file_name);
	hp_bdev->pddr_continuous = true;

	hh_print(HHLOG_DEBUG, "file %s", hp_bdev->file_name);
	hh_print(
		HHLOG_DEBUG,
		"start_sector = %llu, end_sector = %llu, start = %lu, end = %lu, nr_pages %lu",
		hp_bdev->start_sector, hp_bdev->end_sector, hp_bdev->start,
		hp_bdev->end, hp_bdev->nr_pages);

	return 0;
out:
	return -EINVAL;
}

static int do_hyperhold_space_alloc_bdev(const char *bdev_name,
					 const char *file_name, u32 index)
{
	struct hyperhold_space *hp_bdev;
	int ret;

	hp_bdev = &global_settings.backing_bdev[index];
	ret = hyperhold_get_bdev(bdev_name, file_name, hp_bdev);
	if (unlikely(ret)) {
		hh_print(HHLOG_ERR, "hyperhold_get_bdev fail");
		goto out;
	}

	ret = hyperhold_get_continuous_bdev_info(hp_bdev);
out:
	return ret;
}

int hyperhold_space_alloc_bdev(u32 index)
{
	int ret;
	if (!hyperhold_space_info_flag(HP_SPACE_INFO_DEV_INITED)) {
		hh_print(HHLOG_ERR, "bdev info not inited");
		return -EINVAL;
	}

	if (index != dev_info.index) {
		hh_print(HHLOG_ERR, "alloc bdev error %u", index);
		return -EINVAL;
	}

	ret = do_hyperhold_space_alloc_bdev(dev_info.bdev_name,
					    dev_info.file_name, dev_info.index);
	if (unlikely(ret)) {
		hh_print(HHLOG_ERR, "get %llu bdev file %s fail\n",
			 dev_info.index, dev_info.file_name);
		hyperhold_backingbdev_deinit(dev_info.index);
	} else {
		hh_print(HHLOG_DEBUG, "get %llu bdev file %s success\n",
			 dev_info.index, dev_info.file_name);
	}

	return ret;
}
#endif

static int hyperhold_space_file_alloc(void)
{
	int all_space_count, hp_space_count;
	int ret;

	all_space_count = hyperhold_get_space_count();
	if (all_space_count <= 0)
		return -1;

	hp_space_count = HP_SPACE_FIRST_FILE;
	while (hp_space_count < all_space_count) {
#ifndef CONFIG_HYPERHOLD_GKI
		ret = hyperhold_space_alloc_bdev(hp_space_count);
		if (ret) {
			ret = hyperhold_space_alloc_loopbdev(hp_space_count);
		}
#else
		ret = hyperhold_space_alloc_loopbdev(hp_space_count);
#endif

		if (ret) {
			hh_print(HHLOG_ERR, "hyperhold space file alloc error");
			return ret;
		}
		hp_space_count++;
	}

	global_settings.bdev_count = all_space_count;
	return 0;
}

static void hyperhold_space_partition_init(struct hyperhold_bind_info *ori_info)
{
	struct hyperhold_space *hp_bdev =
		&global_settings.backing_bdev[HP_SPACE_PARTITION];

	hp_bdev->bdev = ori_info->bdev;
	hp_bdev->backing_dev = ori_info->backing_dev;
	hp_bdev->nr_pages = ori_info->nr_pages;
	hp_bdev->start_sector = 0;
	hp_bdev->end_sector = ori_info->nr_pages * HYPERHOLD_PAGE_SIZE_SECTOR;
	hp_bdev->start = 0;
	hp_bdev->end = ori_info->nr_pages;
	hp_bdev->pddr_continuous = true;
	snprintf(hp_bdev->bdev_name, HYPERHOLD_NAME_LEN, "%s",
		 ori_info->bdev_name);
}

static void hyperhold_space_deinit(void)
{
	int index = HP_SPACE_FIRST_FILE;
	for (; index < HP_SPACE_MAX_NR; index++) {
		hyperhold_backingbdev_deinit(index);
	}
}

static void hyperhold_space_init(void)
{
	int index = 0;
	for (; index < HP_SPACE_MAX_NR; index++) {
		hyperhold_backingbdev_init(index);
	}

	global_settings.bdev_count = 0;
}

#endif

static void hyperhold_put_mapped_bdev(struct block_device *bdev)
{
	if (bdev)
		blkdev_put(bdev, FMODE_READ);
}

static struct block_device *hyperhold_get_mapped_bdev(const char *file_name)
{
	struct block_device *bdev = NULL;
	int err;

	bdev = blkdev_get_by_path(file_name, FMODE_READ, NULL);
	if (IS_ERR(bdev)) {
		err = PTR_ERR(bdev);
		hh_print(HHLOG_ERR,
			 "get_bdev %s blkdev_get_by_patch failed! eno = %d\n",
			 file_name, err);
		return NULL;
	}
	return bdev;
}

static void hyperhold_stat_init(struct hyperhold_stat *stat)
{
	int i;

	atomic64_set(&stat->reclaimin_cnt, 0);
	atomic64_set(&stat->reclaimin_bytes, 0);
	atomic64_set(&stat->reclaimin_pages, 0);
	atomic64_set(&stat->reclaimin_infight, 0);
	atomic64_set(&stat->batchout_cnt, 0);
	atomic64_set(&stat->batchout_bytes, 0);
	atomic64_set(&stat->batchout_pages, 0);
	atomic64_set(&stat->batchout_inflight, 0);
	atomic64_set(&stat->fault_cnt, 0);
	atomic64_set(&stat->hyperhold_fault_cnt, 0);
	atomic64_set(&stat->reout_pages, 0);
	atomic64_set(&stat->reout_bytes, 0);
	atomic64_set(&stat->zram_stored_pages, 0);
	atomic64_set(&stat->zram_stored_size, 0);
	atomic64_set(&stat->stored_pages, 0);
	atomic64_set(&stat->stored_size, 0);
	atomic64_set(&stat->notify_free, 0);
	atomic64_set(&stat->frag_cnt, 0);
	atomic64_set(&stat->mcg_cnt, 0);
	atomic64_set(&stat->ext_cnt, 0);
	atomic64_set(&stat->daily_ext_max, 0);
	atomic64_set(&stat->daily_ext_min, 0);
	atomic64_set(&stat->miss_free, 0);
	atomic64_set(&stat->mcgid_clear, 0);
	atomic64_set(&stat->swapin_time, 0);
	atomic64_set(&stat->swapout_time, 0);
	atomic64_set(&stat->bioin_time, 0);
	atomic64_set(&stat->bioout_time, 0);
#ifdef CONFIG_ZSWAPD_GKI
	atomic64_set(&stat->zswapd_wakeup, 0);
	atomic64_set(&stat->zswapd_refault, 0);
	atomic64_set(&stat->zswapd_swapout, 0);
	atomic64_set(&stat->zswapd_critical_press, 0);
	atomic64_set(&stat->zswapd_medium_press, 0);
	atomic64_set(&stat->zswapd_snapshot_times, 0);
	atomic64_set(&stat->zswapd_empty_round, 0);
	atomic64_set(&stat->zswapd_empty_round_skip_times, 0);
#endif

	for (i = 0; i < HYPERHOLD_SCENARIO_BUTT; ++i) {
		atomic64_set(&stat->io_fail_cnt[i], 0);
		atomic64_set(&stat->alloc_fail_cnt[i], 0);
		atomic64_set(&stat->lat[i].total_lat, 0);
		atomic64_set(&stat->lat[i].max_lat, 0);
	}

	stat->record.num = 0;
	spin_lock_init(&stat->record.lock);
}

static bool hyperhold_global_setting_init(struct zram *zram)
{
	if (unlikely(global_settings.stat))
		return false;

#ifdef CONFIG_HYPERHOLD_GKI_DEBUG
	global_settings.log_level = HHLOG_DEBUG;
#else
	global_settings.log_level = HHLOG_ERR;
#endif

	global_settings.zram = zram;
	hyperhold_wdt_set_enable(true);
	hyperhold_set_enable(false);
	hyperhold_set_crypto_enable(false);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 61)
	timer_setup(&global_settings.wdt_timer, hyperhold_wdt_timeout, 0);
#else
	setup_timer(&global_settings.wdt_timer, hyperhold_wdt_timeout, 0ul);
#endif
	hyperhold_wdt_expire_set(HYPERHOLD_WDT_EXPIRE_DEFAULT);
	global_settings.stat =
		hyperhold_malloc(sizeof(struct hyperhold_stat), false, true);
	if (unlikely(!global_settings.stat)) {
		hh_print(HHLOG_ERR, "global stat allocation failed!\n");

		return false;
	}

	hyperhold_stat_init(global_settings.stat);
	global_settings.reclaim_wq =
		alloc_workqueue("hyperhold_reclaim", WQ_CPU_INTENSIVE, 0);
	if (unlikely(!global_settings.reclaim_wq)) {
		hh_print(HHLOG_ERR, "reclaim workqueue allocation failed!\n");
		hyperhold_free(global_settings.stat);
		global_settings.stat = NULL;

		return false;
	}

	global_settings.shrink_wq =
		alloc_workqueue("hyperhold_shrink", WQ_CPU_INTENSIVE, 0);
	if (unlikely(!global_settings.shrink_wq)) {
		hh_print(HHLOG_ERR, "shrink workqueue allocation failed!\n");
		hyperhold_free(global_settings.stat);
		global_settings.stat = NULL;

		return false;
	}

#ifdef CONFIG_HYPERHOLD_FILE_ARCHIVAL
	hyperhold_space_init();
#endif
	return true;
}

static void hyperhold_global_setting_deinit(void)
{
	destroy_workqueue(global_settings.reclaim_wq);
	destroy_workqueue(global_settings.shrink_wq);
	hyperhold_free(global_settings.stat);
	global_settings.stat = NULL;
	global_settings.zram = NULL;
#ifdef CONFIG_HYPERHOLD_FILE_ARCHIVAL
	hyperhold_space_deinit();
#endif
}

struct workqueue_struct *hyperhold_get_reclaim_workqueue(void)
{
	return global_settings.reclaim_wq;
}

struct workqueue_struct *hyperhold_get_shrink_workqueue(void)
{
	return global_settings.shrink_wq;
}

/*
 * This interface will be called when user set the ZRAM size.
 * Hyperhold init here.
 */
void hyperhold_init(struct zram *zram)
{
	int ret = 0;
	const char *real_name = HYPERHOLD_BLOCK_DEV_NAME;
	const char *mapper_name = global_settings.mapper_name;
	struct block_device *real_bdev = NULL;
	struct hyperhold_bind_info bind_info;

	hh_print(HHLOG_ERR, "hyperhold init start");
	if (!hyperhold_global_setting_init(zram))
		return;

	if (mapper_name != NULL) {
		real_bdev = hyperhold_get_mapped_bdev(real_name);
		if (!real_bdev) {
			hh_print(HHLOG_ERR, "get storage device failed! %d\n",
				 ret);
			return;
		}
		ret = hyperhold_bind(zram, mapper_name, &bind_info);
	} else {
		ret = hyperhold_bind(zram, real_name, &bind_info);
	}
	if (unlikely(ret)) {
		hh_print(HHLOG_ERR, "bind storage device failed! %d\n", ret);
		goto err_out;
	}

#ifdef CONFIG_HYPERHOLD_FILE_ARCHIVAL
	hh_print(HHLOG_ERR, "hyperhold file space alloc start");
	if (!hyperhold_space_is_file_arch()) {
		hh_print(HHLOG_ERR, "hyperhold file space type is partition");
		goto finish_bind;
	}

	hyperhold_space_partition_init(&bind_info);
	ret = hyperhold_space_file_alloc();
	if (unlikely(ret)) {
		hh_print(HHLOG_ERR, "hyperhold file space alloc failed! %d",
			 ret);
		goto err_out;
	}
	hyperhold_space_manager_init();
	bind_info.nr_pages = hyperhold_space_get_total_sz() >> PAGE_SHIFT;
	hh_print(HHLOG_ERR, "hyperhold file space alloc succ");
	hyperhold_space_all_info();

finish_bind:
#endif

	down_write(&zram->init_lock);
	zram->bdev = bind_info.bdev;
	zram->backing_dev = bind_info.backing_dev;
	zram->nr_pages = bind_info.nr_pages;
	up_write(&zram->init_lock);
#ifdef CONFIG_HYPERHOLD_DEBUG
	global_settings.stat->nr_pages = zram->nr_pages;
#endif
	global_settings.real_bdev = real_bdev ?: zram->bdev;

#ifndef CONFIG_HYPERHOLD_GKI
	zs_pool_enable_ext(zram->mem_pool, true, hyperhold_zsmalloc_parse);
	zs_pool_ext_malloc_register(zram->mem_pool,
				    hyperhold_alloc_page_common);
#else
	memcg_ext_init();
	hhgki_zswapd_init();
#endif
	hh_print(HHLOG_ERR, "hyperhold space init succ");
	return;

err_out:
	hyperhold_put_mapped_bdev(real_bdev);
	hyperhold_global_setting_deinit();
	hh_print(HHLOG_ERR, "hyperhold init failed");
}

static int hyperhold_set_enable_init(bool en)
{
	int ret;

	if (hyperhold_enable() || !en)
		return 0;

	if (!global_settings.stat) {
		hh_print(HHLOG_ERR, "global_settings.stat is null!\n");

		return -EINVAL;
	}

	hh_print(HHLOG_ERR, "hyperhold init manager\n");
	ret = hyperhold_manager_init(global_settings.zram);
	if (unlikely(ret)) {
		hh_print(HHLOG_ERR, "init manager failed! %d\n", ret);

		return -EINVAL;
	}

	hh_print(HHLOG_ERR, "hyperhold init schedule\n");
	ret = hyperhold_schedule_init();
	if (unlikely(ret)) {
		hh_print(HHLOG_ERR, "init schedule failed! %d\n", ret);
		hyperhold_manager_deinit(global_settings.zram);

		return -EINVAL;
	}

	hh_print(HHLOG_ERR, "hyperhold enable init succ\n");
	return 0;
}

struct hyperhold_stat *hyperhold_get_stat_obj(void)
{
	return global_settings.stat;
}

#ifndef CONFIG_HYPERHOLD_GKI
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)
int __attribute__((weak))
blk_lld_health_query(struct block_device *bi_bdev, u8 *pre_eol_info,
		     u8 *life_time_est_a, u8 *life_time_est_b)
{
#ifndef UFS_IOCTL_QUERY
#define UFS_IOCTL_QUERY 0x5388
#define UPIU_QUERY_OPCODE_READ_DESC 0x1

#define QUERY_DESC_IDN_HEALTH 0x9
#define QUERY_DESC_HEALTH_DEF_SIZE 0x25

#define HEALTH_DESC_PARAM_EOL_INFO 0x2
#define HEALTH_DESC_PARAM_LIFE_TIME_EST_A 0x3
#define HEALTH_DESC_PARAM_LIFE_TIME_EST_B 0x4
#endif
	struct ufs_ioctl_query_data {
		__u32 opcode;
		__u8 idn;
		__u16 buf_size;
		__u8 buffer[QUERY_DESC_HEALTH_DEF_SIZE];
	} query = {0};
	int ret = -ENOTSUPP;
	struct block_device *ufs_dev = bi_bdev->bd_contains;
	if (ufs_dev) {
		mm_segment_t old_fs = get_fs();
		set_fs(KERNEL_DS);
		query.opcode = UPIU_QUERY_OPCODE_READ_DESC;
		query.idn = QUERY_DESC_IDN_HEALTH;
		query.buf_size = QUERY_DESC_HEALTH_DEF_SIZE;
		ret = blkdev_ioctl(ufs_dev, FMODE_READ, UFS_IOCTL_QUERY,
				   (unsigned long)&query);
		set_fs(old_fs);
		if (ret == 0) {
			*pre_eol_info =
				query.buffer[HEALTH_DESC_PARAM_EOL_INFO];
			*life_time_est_a =
				query.buffer[HEALTH_DESC_PARAM_LIFE_TIME_EST_A];
			*life_time_est_b =
				query.buffer[HEALTH_DESC_PARAM_LIFE_TIME_EST_B];
		}
	}
	return ret;
}
#else
int __attribute__((weak))
blk_lld_health_query(struct block_device *bi_bdev, u8 *pre_eol_info,
		     u8 *life_time_est_a, u8 *life_time_est_b)
{
	*pre_eol_info = 1;
	*life_time_est_a = 3;
	*life_time_est_b = 3;
	hh_print(HHLOG_ERR, "weak blk_lld_health_query\n");
	return 0;
}
#endif
#endif

static int hyperhold_health_check(void)
{
	int ret;
	u8 pre_eol_info = PRE_EOL_INFO_OVER_VAL;
	u8 life_time_est_a = LIFE_TIME_EST_OVER_VAL;
	u8 life_time_est_b = LIFE_TIME_EST_OVER_VAL;

	if (unlikely(!global_settings.real_bdev)) {
		hh_print(HHLOG_ERR, "bdev is null!\n");

		return -EFAULT;
	}

#ifdef CONFIG_HYPERHOLD_GKI
	ret = get_ufs_health_info(&pre_eol_info, &life_time_est_a,
				  &life_time_est_b);
#else
	ret = blk_lld_health_query(global_settings.real_bdev, &pre_eol_info,
				   &life_time_est_a, &life_time_est_b);
#endif
	if (ret) {
		hh_print(HHLOG_ERR, "query health err %d!\n", ret);

		return ret;
	}

	if ((pre_eol_info >= PRE_EOL_INFO_OVER_VAL) ||
	    (life_time_est_a >= LIFE_TIME_EST_OVER_VAL) ||
	    (life_time_est_b >= LIFE_TIME_EST_OVER_VAL)) {
		hh_print(HHLOG_ERR, "over life time uesd %u %u %u\n",
			 pre_eol_info, life_time_est_a, life_time_est_b);

		return -EPERM;
	}

	hh_print(HHLOG_DEBUG, "life time uesd %u %u %u\n", pre_eol_info,
		 life_time_est_a, life_time_est_b);

	return 0;
}

ssize_t hyperhold_enable_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t len)
{
	int ret;
	unsigned long val;
	struct zram *zram = (struct zram *)dev_to_disk(dev)->private_data;

	ret = kstrtoul(buf, 0, &val);
	if (unlikely(ret)) {
		hh_print(HHLOG_ERR, "val is error!\n");

		return -EINVAL;
	}

	/* hyperhold must be close when over 70% life time uesd */
	if (hyperhold_health_check())
		val = false;
	down_write(&zram->init_lock);
	if (hyperhold_set_enable_init(!!val)) {
		up_write(&zram->init_lock);
		return -EINVAL;
	}

	hyperhold_set_enable(!!val);
	up_write(&zram->init_lock);
	return len;
}

ssize_t hyperhold_enable_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	return snprintf(buf, PAGE_SIZE, "hyperhold %s reclaim_in %s\n",
			hyperhold_enable() ? "enable" : "disable",
			hyperhold_reclaim_in_enable() ? "enable" : "disable");
}

ssize_t hyperhold_mapper_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t len)
{
	if (global_settings.mapper_name)
		kfree(global_settings.mapper_name);
	global_settings.mapper_name = kstrdup(buf, GFP_KERNEL);
	return len;
}

ssize_t hyperhold_mapper_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
			global_settings.mapper_name ?: "");
}
#ifdef CONFIG_HYPERHOLD_CACHE
struct zram *hyperhold_get_global_zram(void)
{
	return global_settings.zram;
}
#endif
#ifndef CONFIG_HYPERHOLD_GKI
extern int hyperhold_get_keyring_key(u32 key_id, u8 *key_buffer,
				     u8 key_buffer_len, u32 *key_len);

ssize_t hyperhold_cryptokey_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	u8 ret;
	u32 val;

	ret = kstrtou32(buf, 0, &val);
	if (unlikely(ret)) {
		hh_print(HHLOG_ERR, "val is error %d\n", ret);
		return -EINVAL;
	}

	if (hyperhold_crypto_enable()) {
		hh_print(HHLOG_ERR, "hyperhold crypto has enabled\n");
		return len;
	}

	memset_s(hyperhold_io_key, HYPERHOLD_KEY_SIZE, 0, HYPERHOLD_KEY_SIZE);
	ret = hyperhold_get_keyring_key(val, hyperhold_io_key,
					HYPERHOLD_KEY_SIZE,
					&hyperhold_io_key_len);
	if (ret) {
		hh_print(HHLOG_ERR, "hyperhold_get_keyring_key fail %d!\n",
			 ret);
		return -EINVAL;
	}

	hh_print(HHLOG_ERR, "hyperhold_get_keyring_key success, len: %d\n",
		 hyperhold_io_key_len);
	return len;
}
#endif
ssize_t hyperhold_cryptokey_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "crypto %s\n",
			hyperhold_crypto_enable() ? "enable" : "disable");
}

#ifdef CONFIG_HYPERHOLD_GKI
void hyperhold_gki_zram_exit(void)
{
	struct zram *zram = global_settings.zram;

	if (!zram)
		return;
	global_settings.zram = NULL;
	down_write(&zram->init_lock);
	if (zram->bitmap)
		kfree(zram->bitmap);
	hyperhold_close_bdev(zram->bdev, zram->backing_dev);
	down_write(&zram->init_lock);
#ifdef CONFIG_ZSWAPD_GKI
	hhgki_zswapd_deinit();
#endif
}

#ifdef CONFIG_HYPERHOLD_GKI_DEBUG
static void hyperhold_gki_loglevel_set(int level)
{
	global_settings.log_level = level;
}

ssize_t hyperhold_gki_debug_level_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	unsigned long val;
	int ret;

	if (!hyperhold_enable())
		return len;

	ret = kstrtoul(buf, 0, &val);
	if (unlikely(ret))
		return -EINVAL;
	hyperhold_gki_loglevel_set((int)val);

	return len;
}

ssize_t hyperhold_gki_debug_level_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	ssize_t size = 0;
	size += scnprintf(buf + size, PAGE_SIZE - size,
			  "Hyperhold log level: %d\n", hyperhold_loglevel());
	return size;
}
#endif
#endif

#ifdef CONFIG_HYPERHOLD_DEBUG_FS
static void hyperhold_loglevel_set(int level)
{
	global_settings.log_level = level;
}

static bool ft_get_val(const char *buf, const char *token, unsigned long *val)
{
	int ret = -EINVAL;
	char *str = strstr(buf, token);

	if (str)
		ret = kstrtoul(str + strlen(token), 0, val);

	return ret == 0;
}

ssize_t hyperhold_ft_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t len)
{
	char *type_buf = NULL;
	unsigned long val;

	if (!hyperhold_enable())
		return len;

	type_buf = strstrip((char *)buf);
	if (ft_get_val(type_buf, "idle", &val)) {
		memcg_idle_count(global_settings.zram);
		goto out;
	}
	if (ft_get_val(type_buf, "loglevel=", &val)) {
		hyperhold_loglevel_set((int)val);
		goto out;
	}

	if (ft_get_val(type_buf, "watchdog=", &val)) {
		if (val)
			hyperhold_wdt_expire_set(val);
		hyperhold_wdt_set_enable(!!val);
	}
out:
	return len;
}

ssize_t hyperhold_ft_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	ssize_t size = 0;
	struct hyperhold_stat *stat = hyperhold_get_stat_obj();

	size += scnprintf(buf + size, PAGE_SIZE, "Hyperhold enable: %s\n",
			  hyperhold_enable() ? "Yes" : "No");
	size += scnprintf(buf + size, PAGE_SIZE - size,
			  "Hyperhold watchdog enable: %s\n",
			  hyperhold_wdt_enable() ? "Yes" : "No");
	size += scnprintf(buf + size, PAGE_SIZE - size,
			  "Hyperhold watchdog expire(s): %lu\n",
			  global_settings.wdt_expire_s);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			  "Hyperhold log level: %d\n", hyperhold_loglevel());
	if (stat)
		size += scnprintf(buf + size, PAGE_SIZE - size,
				  "Hyperhold mcgid clear: %lld\n",
				  atomic64_read(&stat->mcgid_clear));

	return size;
}

ssize_t hyperhold_swap_time_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct hyperhold_stat *stat = hyperhold_get_stat_obj();

	if (stat) {
		atomic64_set(&stat->swapin_time, 0);
		atomic64_set(&stat->swapout_time, 0);
	}

	return len;
}
ssize_t hyperhold_swap_time_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	ssize_t size = 0;
	struct hyperhold_stat *stat = hyperhold_get_stat_obj();

	if (stat) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
				  "Hyperhold swapin time: %lld\n",
				  atomic64_read(&stat->swapin_time));
		size += scnprintf(buf + size, PAGE_SIZE - size,
				  "Hyperhold swapout time: %lld\n",
				  atomic64_read(&stat->swapout_time));
	}

	return size;
}
ssize_t hyperhold_io_time_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t len)
{
	struct hyperhold_stat *stat = hyperhold_get_stat_obj();

	if (stat) {
		atomic64_set(&stat->bioin_time, 0);
		atomic64_set(&stat->bioout_time, 0);
	}

	return len;
}
ssize_t hyperhold_io_time_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	ssize_t size = 0;
	struct hyperhold_stat *stat = hyperhold_get_stat_obj();

	if (stat) {
		size += scnprintf(buf + size, PAGE_SIZE - size,
				  "Hyperhold swapin bio time: %lld\n",
				  atomic64_read(&stat->bioin_time));
		size += scnprintf(buf + size, PAGE_SIZE - size,
				  "Hyperhold swapout bio time: %lld\n",
				  atomic64_read(&stat->bioout_time));
	}

	return size;
}
#endif
#ifdef CONFIG_HYPERHOLD_CACHE
ssize_t hyperhold_cache_show(struct device *dev, struct device_attribute *attrr,
			     char *buf)
{
	ssize_t size = 0;

	hyperhold_cache_state(global_settings.zram, buf, &size);

	return size;
}
ssize_t hyperhold_cache_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t len)
{
	char *type_buf = NULL;
	unsigned long val;

	if (!hyperhold_enable())
		return len;

	type_buf = strstrip((char *)buf);
	if (ft_get_val(type_buf, "cachelevel=", &val)) {
		hyperhold_set_cache_level(global_settings.zram, (int)val);
		goto out;
	}
	if (ft_get_val(type_buf, "cache_high_wm=", &val)) {
		hyperhold_set_cache_wm(global_settings.zram, (int)val, 1);
		goto out;
	}
	if (ft_get_val(type_buf, "cache_low_wm=", &val)) {
		hyperhold_set_cache_wm(global_settings.zram, (int)val, 0);
		goto out;
	}

out:
	return len;
}
#endif
