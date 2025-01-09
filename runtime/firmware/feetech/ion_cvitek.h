/*
 * Copyright (C) Cvitek Co., Ltd. 2019-2020. All rights reserved.
 *
 * File Name: ion_cvitek.h
 * Description:
 */

#ifndef __ION_CVITEK_H__
#define __ION_CVITEK_H__

#include <unistd.h>

struct ion_custom_data {
	unsigned int cmd;
	unsigned long arg;
};

#define ION_IOC_CUSTOM		_IOWR('I', 6, struct ion_custom_data)

struct cvitek_cache_range {
	void *start;
#ifdef __arm__
	uint32_t padding;
#endif
	uint32_t size;
	uint64_t paddr;
};

struct cvitek_heap_info {
	unsigned int id;
	unsigned long total_size;
	unsigned long avail_size;
};

#define ION_IOC_CVITEK_FLUSH_RANGE		1
#define ION_IOC_CVITEK_GET_HEAP_INFO		2
#define ION_IOC_CVITEK_INVALIDATE_RANGE		3
#define ION_IOC_CVITEK_FLUSH_PHY_RANGE		4
#define ION_IOC_CVITEK_INVALIDATE_PHY_RANGE	5

#endif /* __ION_CVITEK_H__ */
