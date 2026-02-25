/*
 * Copyright (c) Honor Co., Ltd. 2021-2022. All rights reserved.
 * Description: This file is for preallocating blocks in f2fs
 * Author: laixinyi wangzijie
 * Create: 2024-01-08
 */

#include <linux/stat.h>
#include <linux/buffer_head.h>
#include <linux/writeback.h>
#include <linux/blkdev.h>
#include <linux/falloc.h>
#include <linux/types.h>
#include <linux/compat.h>
#include <linux/uaccess.h>
#include <linux/mount.h>
#include <linux/pagevec.h>
#include <linux/uio.h>
#include <linux/uuid.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/f2fs_fs.h>

#include "f2fs.h"
#include "node.h"
#include "segment.h"

static inline void __curseg_meta_dup(struct curseg_info *dst,
				     struct curseg_info *src)
{
	dst->inited = src->inited;
	dst->sum_blk = src->sum_blk;
	dst->journal = src->journal;
	dst->next_segno = src->next_segno;
	dst->alloc_type = src->alloc_type;
	dst->segno = src->segno;
	dst->next_blkoff = src->next_blkoff;
	dst->zone = src->zone;
}

int __curseg_set_prealloc(struct f2fs_sb_info *sbi, int type, unsigned int segno,
		    struct curseg_info *mirror)
{
	struct curseg_info *curseg = CURSEG_I(sbi, type);
	struct summary_footer *summary_footer = NULL;
	unsigned int extra_flag;
    	unsigned short seg_type = curseg->seg_type;

	mutex_lock(&curseg->curseg_mutex);
	__curseg_meta_dup(mirror, curseg);

	curseg->inited = true;
	curseg->segno = segno;
	curseg->zone = GET_ZONE_FROM_SEG(sbi, curseg->segno);
	curseg->next_segno = NULL_SEGNO;
	curseg->next_blkoff = 0;
	curseg->alloc_type = PREALLOC;
	curseg->journal = NULL;
	curseg->sum_blk = NULL;

	for (extra_flag = 0; !(curseg->sum_blk && curseg->journal);
		extra_flag ? ({extra_flag = GFP_KERNEL;}) : ({extra_flag = __GFP_NOFAIL;})) {
		if (extra_flag == GFP_KERNEL) {
			if (curseg->sum_blk)
				kfree(curseg->sum_blk);
			if (curseg->journal)
				kfree(curseg->journal);
			mutex_unlock(&curseg->curseg_mutex);
			return -ENOMEM;
		}
		curseg->sum_blk = f2fs_kzalloc(sbi, PAGE_SIZE,
						GFP_KERNEL | extra_flag);
		curseg->journal = f2fs_kzalloc(sbi, sizeof(struct f2fs_journal),
						GFP_KERNEL | extra_flag);
	}

	summary_footer = &curseg->sum_blk->footer;
	memset(summary_footer, 0, sizeof(struct summary_footer));
	SET_SUM_TYPE(summary_footer, SUM_TYPE_DATA);
	__set_sit_entry_type(sbi, seg_type, curseg->segno, 1);

	mutex_unlock(&curseg->curseg_mutex);

	return 0;
}

inline void __curseg_restore(struct f2fs_sb_info *sbi, int type,
			 struct curseg_info *mirror)
{
	struct curseg_info *curseg = CURSEG_I(sbi, type);

	mutex_lock(&curseg->curseg_mutex);
	kfree(curseg->sum_blk);
	kfree(curseg->journal);
	__curseg_meta_dup(curseg, mirror);
	mutex_unlock(&curseg->curseg_mutex);
}

static unsigned int min_nr_sec_needed(struct f2fs_sb_info *sbi, int needed)
{
	int node_secs = get_blocktype_secs(sbi, F2FS_DIRTY_NODES);
	int imeta_secs = get_blocktype_secs(sbi, F2FS_DIRTY_IMETA);
	int dent_secs = get_blocktype_secs(sbi, F2FS_DIRTY_DENTS);

	return (node_secs + dent_secs + imeta_secs + dent_secs +
		reserved_sections(sbi) + needed);
}

unsigned int f2fs_find_consecutive_secs(struct f2fs_sb_info *sbi, int nr_sec)
{
	struct free_segmap_info *free_i = FREE_I(sbi);
	unsigned int secno = MAIN_SECS(sbi);
	unsigned int segno;
	unsigned int left = 0, right;

	spin_lock(&free_i->segmap_lock);
	if (free_sections(sbi) <= min_nr_sec_needed(sbi, nr_sec)) {
		printk(KERN_ERR "[HP]%s: free secnum = %u, needed = %u", __func__,
		       free_sections(sbi), min_nr_sec_needed(sbi, nr_sec));
		goto out;
	}
	while (left < MAIN_SECS(sbi)) {
		left = find_next_zero_bit(free_i->free_secmap, MAIN_SECS(sbi), left);
		right = find_next_bit(free_i->free_secmap, MAIN_SECS(sbi), left);
		if (right - left >= (unsigned int)nr_sec) {
			secno = left;
			right = left + nr_sec;
			break;
		} else {
			left = right;
		}
	}
	if (secno < MAIN_SECS(sbi)) {
		left = GET_SEG_FROM_SEC(sbi, left);
		right = GET_SEG_FROM_SEC(sbi, right);
		for (segno = left; segno < right; ++segno)
			__set_inuse(sbi, segno);
	}
out:
	spin_unlock(&free_i->segmap_lock);

	return secno;
}

void __init_curseg_prealloc(struct f2fs_sb_info *sbi, struct curseg_info *curseg)
{
	struct summary_footer *summary_footer = NULL;
   	unsigned short seg_type = curseg->seg_type;

	curseg->inited = true;
	write_sum_page(sbi, curseg->sum_blk, GET_SUM_BLOCK(sbi, curseg->segno));
	curseg->segno++;
	curseg->zone = GET_ZONE_FROM_SEG(sbi, curseg->segno);
	curseg->next_blkoff = 0;
	summary_footer = &curseg->sum_blk->footer;
	memset(summary_footer, 0, sizeof(struct summary_footer));
	SET_SUM_TYPE(summary_footer, SUM_TYPE_DATA);
	__set_sit_entry_type(sbi, seg_type, curseg->segno, 1);
}

static int f2fs_hyper_file_extend(struct inode *inode, unsigned long long len)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	struct f2fs_map_blocks map = {
		.m_next_pgofs = NULL,
		.m_next_extent = NULL,
		.m_seg_type = NO_CHECK_TYPE,
		.m_may_create = 1
	};
	pgoff_t size_in_blk;
	int err;
	struct curseg_info mirror;
	unsigned int start_segno, end_segno, start_secno;

	size_in_blk = len >> PAGE_SHIFT; /*lint !e644*/
	if (has_not_enough_free_secs(sbi, 0, size_in_blk / BLKS_PER_SEC(sbi))) {
			printk(KERN_ERR "[HP]No enough space! \r\n");
			return -ENOSPC;
	}

	err = inode_newsize_ok(inode, len); /*lint !e644*/
	if (err)
		return err;

	map.m_lblk = 0;
	map.m_len = size_in_blk;

	f2fs_down_write(&sbi->pin_sem);
	f2fs_down_write(&sbi->cp_global_sem);
	map.m_seg_type = CURSEG_COLD_DATA_PINNED;

	start_secno = f2fs_find_consecutive_secs(sbi, size_in_blk / BLKS_PER_SEC(sbi));
	if (start_secno >= MAIN_SECS(sbi)) {
		f2fs_up_write(&sbi->cp_global_sem);
		f2fs_up_write(&sbi->pin_sem);
		printk(KERN_ERR "[HP]No continuous space!");
		return -ENOSPC;
	}

	start_segno = GET_SEG_FROM_SEC(sbi, start_secno);
	end_segno = start_segno + (size_in_blk >> sbi->log_blocks_per_seg);

	printk(KERN_ERR "%s found consecutive room from %u to %lu, start_segno:%u, end_segno:%u",
		__func__, START_BLOCK(sbi, start_segno), START_BLOCK(sbi, start_segno) + size_in_blk, start_segno, end_segno);

	err = __curseg_set_prealloc(sbi, CURSEG_COLD_DATA_PINNED, start_segno, &mirror);
	if (err) {
		unsigned int i;
		for (i = start_segno; i < end_segno; i++)
			__set_test_and_free(sbi, i, true);
		f2fs_up_write(&sbi->cp_global_sem);
		f2fs_up_write(&sbi->pin_sem);
		return err;
	}
	err = f2fs_map_blocks(inode, &map, F2FS_GET_BLOCK_PRE_DIO);
	__curseg_restore(sbi, CURSEG_COLD_DATA_PINNED, &mirror);
	f2fs_up_write(&sbi->cp_global_sem);
	f2fs_up_write(&sbi->pin_sem);

	f2fs_i_size_write(inode, len); /*lint !e644*/

	return err;
}

#define HP_FILE_MAX (5UL * 1024UL * 1024UL * 1024UL)

static int f2fs_hyper_parameter_validate(struct inode *inode, unsigned long arg,
			       struct hyper_file_config *config)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	int err;

	err = -EISDIR;
	if (unlikely(!S_ISREG(inode->i_mode))) {
		printk(KERN_ERR "[HP]target is not a regular file! \r\n");
		goto err_out;
	}

	err = -EFAULT;
	if (unlikely(!(struct f2fs_hp_file_cfg __user *)arg))
		goto err_out;

	err = -EINVAL;
	if (unlikely(copy_from_user(config, (struct f2fs_hp_file_cfg __user *)arg,
				    sizeof(*config)))) {
		printk(KERN_ERR "[HP]copy from user error! \r\n");
		goto err_out;
	}

	if (((sbi->segs_per_sec << (sbi->log_blocks_per_seg + PAGE_SHIFT)) - 1) &
	    config->len) {
		printk(KERN_ERR "[HP]Not section align! \r\n");
		goto err_out;
	}

	if (config->len > HP_FILE_MAX) {
		printk(KERN_ERR "[HP]preallocate size too large! \r\n");
		goto err_out;
	}

	err = 0;
err_out:
	return err;
}

static int f2fs_hyper_result_verify(struct file *filp, struct f2fs_sb_info *sbi,
				struct hyper_file_config *config)
{
	struct address_space *mapping = filp->f_mapping;
	unsigned long end_blk;
	int err;

	err = -EINVAL;
	if (!mapping || !mapping->a_ops) {
		printk(KERN_ERR "[HP]No address mapping operations! \r\n");
		goto err_out;
	}

	config->addr = mapping->a_ops->bmap(mapping, 0);
	end_blk = mapping->a_ops->bmap(mapping, (config->len >> PAGE_SHIFT) - 1); /*lint !e644*/

	err = -EFAULT;
	if (end_blk > MAX_BLKADDR(sbi) || end_blk < config->addr) {
		printk(KERN_ERR "[HP]bmap error:start_blk = %lu, end_blk = %lu, maxblk = %u",
			config->addr, end_blk, MAX_BLKADDR(sbi));
		goto err_out;
	}

	if (((end_blk - config->addr + 1) << PAGE_SHIFT != config->len) ||
		(i_size_read(file_inode(filp)) != config->len)) {
		printk(KERN_ERR "[HP]alloc error:start_blk = %lu, end_blk = %lu, required_bytes = %llu file_bytes = %lld allocated_bytes = %lu",
			config->addr, end_blk, config->len, i_size_read(file_inode(filp)), (end_blk - config->addr + 1) << PAGE_SHIFT);
		goto err_out;
	}

	err = 0;
err_out:
	return err;
}

int f2fs_ioc_init_hyper_file(struct file *filp, unsigned long arg)
{
	int ret;
	struct hyper_file_config config;
	struct inode *inode = file_inode(filp);
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);

	printk(KERN_ERR "[HP]hp file ino = %lu", inode->i_ino);

	ret = f2fs_hyper_parameter_validate(inode, arg, &config);
	if (unlikely(ret))
		goto exit;

	printk(KERN_ERR "[HP]hp input file config: pin:%u len:%llu addr:%lu", config.pin, config.len, config.addr);
	ret = mnt_want_write_file(filp);
	if (unlikely(ret)) {
		printk(KERN_ERR "[HP]mnt_want_write_file failed! %d", ret);
		goto exit;
	}

	inode_lock(inode);

	ret = f2fs_convert_inline_inode(inode);
	if (unlikely(ret)) {
		printk(KERN_ERR "[HP]convert inline inode failed! %d", ret);
		goto unlock_exit;
	}

	set_inode_flag(inode, FI_PIN_FILE);
	ret = f2fs_hyper_file_extend(inode, config.len); /*lint !e644*/
	if (ret) {
		clear_inode_flag(inode, FI_PIN_FILE);
		goto unlock_exit;
	}

	inode->i_mtime = inode->i_ctime = current_time(inode);

	f2fs_update_time(F2FS_I_SB(inode), REQ_TIME);
	ret = f2fs_hyper_result_verify(filp, sbi, &config);
	printk(KERN_ERR "[HP]hp output file config: pin:%u len:%llu addr:%lu", config.pin, config.len, config.addr);
	if (unlikely(ret))
		goto unlock_exit;
unlock_exit:
	inode_unlock(inode);
	mnt_drop_write_file(filp);
exit:
	return ret;
}
