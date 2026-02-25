#ifdef CONFIG_HYPERHOLD_FILE_ARCHIVAL
#include "hyperhold_internal.h"

/*must align 4*256k*/
static atomic64_t hp_space_total_sz =
	ATOMIC_LONG_INIT(2UL * 1024UL * 1024UL * 1024UL);
static atomic64_t hp_space_partition_sz =
	ATOMIC_LONG_INIT(2UL * 1024UL * 1024UL * 1024UL);
static atomic64_t hp_space_partition_threshold;
static atomic64_t hp_space_file_threshold;
#define HP_SPACE_ALLIN_SZ (1024 * 1024)

struct hyperhold_space_dev_info dev_info;
struct hyperhold_space_loopdev_info loopdev_info;
static atomic_t hp_space_info_inited = ATOMIC_INIT(0);

inline bool hp_str_eq(const char *buf, const char *target)
{
	if (!buf || !target) {
		hh_print(HHLOG_ERR, "param error!");
		return false;
	}
	return !strncmp(buf, target, strlen(target));
}

inline bool hp_str_get_u64(const char *buf, const char *target, u64 *v)
{
	if (!buf || !target || !v) {
		hh_print(HHLOG_ERR, "param error!");
		return false;
	}
	return !strncmp(buf, target, strlen(target)) &&
	       !kstrtou64(buf + strlen(target), 0, v);
}

inline bool hp_str_get_str(const char *buf, const char *target, char *v,
			   size_t size)
{
	int ret;
	if (!buf || !target || !v) {
		hh_print(HHLOG_ERR, "param error!");
		return false;
	}
	if (hp_str_eq(buf, target)) {
		ret = snprintf(v, size, "%s", buf + strlen(target));
		return ret > 0 && ret < size;
	}
	return false;
}

bool hyperhold_space_info_flag(int flag)
{
	return (atomic_read(&hp_space_info_inited) & flag);
}

void hyperhold_space_param_init(void)
{
	if (hyperhold_space_info_flag(HP_SPACE_INFO_INITED))
		return;
	dev_info.index = 0;
	memset(dev_info.file_name, 0, sizeof(dev_info.file_name));
	memset(dev_info.bdev_name, 0, sizeof(dev_info.bdev_name));
	dev_info.size = 0;
	dev_info.start = 0;
	dev_info.end = 0;

	loopdev_info.index = 0;
	memset(loopdev_info.file_name, 0, sizeof(loopdev_info.file_name));
	memset(loopdev_info.bdev_name, 0, sizeof(loopdev_info.bdev_name));
	loopdev_info.size = 0;
	atomic_set(&hp_space_info_inited, HP_SPACE_INFO_INITED);
}

int hyperhold_space_set_type(u64 value)
{
	if (value == HP_PARTITION_ARCHIVAL) {
		atomic_set(&global_settings.space_type, HP_PARTITION_ARCHIVAL);
	} else if (value == HP_PARTITION_FILE_ARCHIVAL) {
		atomic_set(&global_settings.space_type,
			   HP_PARTITION_FILE_ARCHIVAL);
	} else {
		hh_print(HHLOG_ERR, "hyperhold space val is error!");
		return -EINVAL;
	}
	return 0;
}

u64 hyperhold_space_get_total_sz(void)
{
	return atomic64_read(&hp_space_total_sz);
}

int hyperhold_space_set_total_sz(u64 size)
{
	/*must align 4*256k*/
	size = size * HP_SPACE_ALLIN_SZ;
	atomic64_set(&hp_space_total_sz, size);
	hh_print(HHLOG_ERR, "hyperhold space size: %llu",
		 atomic64_read(&hp_space_total_sz));
	return 0;
}

u64 hyperhold_space_get_partition_sz(void)
{
	return atomic64_read(&hp_space_partition_sz);
}

int hyperhold_space_set_partition_sz(u64 size)
{
	/*must align 4*256k*/
	size = size * HP_SPACE_ALLIN_SZ;
	atomic64_set(&hp_space_partition_sz, size);
	hh_print(HHLOG_DEBUG, "hyperhold space size: %llu",
		 atomic64_read(&hp_space_partition_sz));
	return 0;
}

void hyperhold_space_manager_init(void)
{
	u64 space_file_sz, threshold;

	space_file_sz = atomic64_read(&hp_space_total_sz) -
			atomic64_read(&hp_space_partition_sz);
	threshold = space_file_sz >> HYPERHOLD_SECTOR_SHIFT;
	if (atomic64_read(&hp_space_partition_sz) == 0)
		threshold += 1;

	atomic64_set(&hp_space_partition_threshold, threshold);
	atomic64_set(&hp_space_file_threshold, 0);
}

u64 hyperhold_get_partition_threshold(void)
{
	return atomic64_read(&hp_space_partition_threshold);
}

u64 hyperhold_get_file_threshold(void)
{
	return atomic64_read(&hp_space_file_threshold);
}

int hyperhold_get_space_count(void)
{
	return HP_SPACE_MAX_NR;
}

bool hyperhold_space_parse_loopdev(char *param[], int max_index)
{
	int index = 1;
	char *element = NULL;
	u64 value;
	char str_value[50];
	size_t temp;

	if (loopdev_info.size != 0)
		return false;

	while (index < max_index) {
		value = 0;
		memset(str_value, 0, sizeof(str_value));
		element = param[index];
		hh_print(HHLOG_ERR, "index %d element %s", index, element);
		if (hp_str_get_u64(element, "INDEX=", &value)) {
			loopdev_info.index = value;
		} else if (hp_str_get_str(element, "DEV=", str_value,
					  sizeof(str_value))) {
			temp = snprintf(loopdev_info.bdev_name,
					sizeof(dev_info.bdev_name), "%s",
					str_value);
			if (temp < 0 || temp > sizeof(dev_info.bdev_name))
				return false;
		} else if (hp_str_get_u64(element, "SIZE=", &value)) {
			loopdev_info.size = value * HP_SPACE_ALLIN_SZ;
		} else if (hp_str_get_str(element, "FILE=", str_value,
					  sizeof(str_value))) {
			temp = snprintf(loopdev_info.file_name,
					sizeof(loopdev_info.file_name), "%s",
					str_value);
			if (temp < 0 || temp > sizeof(loopdev_info.file_name))
				return false;
		}
		index++;
	}

	atomic_set(&hp_space_info_inited, HP_SPACE_INFO_LOOP_DEV_INITED);
	return true;
}

bool hyperhold_space_parse_dev(char *param[], int max_index)
{
	int index = 1;
	char *element = NULL;
	u64 value;
	char str_value[50];
	size_t temp;

	if (dev_info.size != 0)
		return false;

	while (index < max_index) {
		value = 0;
		memset(str_value, 0, sizeof(str_value));
		element = param[index];
		hh_print(HHLOG_ERR, "index %d element %s", index, element);
		if (hp_str_get_u64(element, "INDEX=", &value)) {
			dev_info.index = value;
		} else if (hp_str_get_str(element, "DEV=", str_value,
					  sizeof(str_value))) {
			temp = snprintf(dev_info.bdev_name,
					sizeof(dev_info.bdev_name), "%s",
					str_value);
			if (temp < 0 || temp > sizeof(dev_info.bdev_name))
				return false;
		} else if (hp_str_get_u64(element, "SIZE=", &value)) {
			dev_info.size = value * HP_SPACE_ALLIN_SZ;
		} else if (hp_str_get_str(element, "FILE=", str_value,
					  sizeof(str_value))) {
			temp = snprintf(dev_info.file_name,
					sizeof(dev_info.file_name), "%s",
					str_value);
			if (temp < 0 || temp > sizeof(dev_info.file_name))
				return false;
		} else if (hp_str_get_u64(element, "START=", &value)) {
			dev_info.start = value;
		} else if (hp_str_get_u64(element, "END=", &value)) {
			dev_info.end = value;
		}
		index++;
	}
	atomic_set(&hp_space_info_inited, HP_SPACE_INFO_DEV_INITED);
	return true;
}

bool hyperhold_space_parse_space_all(char *param[], int max_index)
{
	bool ret = true;
	int index = 1;
	char *element = NULL;
	u64 value;

	while (index < max_index) {
		value = 0;
		element = param[index];
		hh_print(HHLOG_ERR, "index %d element %s", index, element);
		if (hp_str_get_u64(element, "SPACE_TYPE=", &value)) {
			hyperhold_space_set_type(value);
		} else if (hp_str_get_u64(element, "TOTAL_SIZE=", &value)) {
			hyperhold_space_set_total_sz(value);
		} else if (hp_str_get_u64(element, "PARTITION_SIZE=", &value)) {
			hyperhold_space_set_partition_sz(value);
		}
		index++;
	}
	return ret;
}

bool hyperhold_space_parse(char *param[], int max_index)
{
	char *element;
	hyperhold_space_param_init();
	element = param[0];
	if (!element) {
		hh_print(HHLOG_ERR, "element NULL");
		return false;
	}
	hh_print(HHLOG_ERR, "index 0 element %s", element);
	if (hp_str_eq(element, "TYPE=SPACE_ALL")) {
		return hyperhold_space_parse_space_all(param, max_index);
	} else if (hp_str_eq(element, "TYPE=DEV")) {
		return hyperhold_space_parse_dev(param, max_index);
	} else if (hp_str_eq(element, "TYPE=LOOPDEV")) {
		return hyperhold_space_parse_loopdev(param, max_index);
	}
	return false;
}

bool hyperhold_space_split_param(const char *buf, size_t len)
{
	char delims[] = ";";
	char *token = NULL;
	int index = 0;
	int size;
	char *result[10];
	char *element;
	char *dumpbuf, **dumpbuf_ref;
	size_t temp_size;
	bool ret = true;

	dumpbuf = hyperhold_malloc(len + 1, true, true);
	if (!dumpbuf) {
		hh_print(HHLOG_ERR, "param buf error");
		return false;
	}
	size = snprintf(dumpbuf, len + 1, "%s", buf);
	if (size < 0 || size > len + 1) {
		ret = false;
		goto errout;
	}
	dumpbuf_ref = &dumpbuf;
	memset(result, 0, sizeof(result));
	token = strsep(dumpbuf_ref, delims);
	while (token != NULL && strlen(token) > 0 && index < 10) {
		hh_print(HHLOG_ERR, "hyperhold space index %d param %s", index,
			 token);
		temp_size = strlen(token) + 1;
		element = hyperhold_malloc(temp_size, true, true);
		size = snprintf(element, temp_size, "%s", token);
		if (size > 0 && size < temp_size) {
			result[index] = element;
		} else {
			hh_print(HHLOG_ERR, "param error %s", dumpbuf);
			ret = false;
			goto out;
		}

		index++;
		token = strsep(dumpbuf_ref, delims);
	}

	ret = hyperhold_space_parse(result, index);

out:
	while (index >= 0) {
		element = result[index];
		if (element)
			hyperhold_free(element);
		index--;
	}
errout:
	hyperhold_free(dumpbuf);
	return ret;
}

static void hyperhold_space_info(int index)
{
	struct hyperhold_space *space;
	if (index >= HP_SPACE_MAX_NR)
		return;
	space = &global_settings.backing_bdev[index];
	hh_print(HHLOG_ERR, "space index: %d", index);
	hh_print(HHLOG_ERR, "space file_name: %s", space->file_name);
	hh_print(HHLOG_ERR, "space bdev_name: %s", space->bdev_name);
	hh_print(HHLOG_ERR, "space nr_pages: %lu", space->nr_pages);
	hh_print(HHLOG_ERR, "space start_sector: %llu", space->start_sector);
	hh_print(HHLOG_ERR, "space end_sector: %llu", space->end_sector);
	hh_print(HHLOG_ERR, "space start: %lu", space->start);
	hh_print(HHLOG_ERR, "space end: %lu", space->end);
	hh_print(HHLOG_ERR, "space pddr_continuous: %d",
		 space->pddr_continuous);
	if (index == HP_SPACE_PARTITION)
		hh_print(HHLOG_ERR, "space threshold: %llu",
			 hyperhold_get_partition_threshold());
	else if (index == HP_SPACE_FIRST_FILE)
		hh_print(HHLOG_ERR, "space threshold: %llu",
			 hyperhold_get_file_threshold());
}

void hyperhold_space_all_info(void)
{
	int hp_space_count = 0;

	hh_print(HHLOG_ERR, "print space all info");
	while (hp_space_count < global_settings.bdev_count &&
	       hp_space_count < HP_SPACE_MAX_NR) {
		hyperhold_space_info(hp_space_count);
		hp_space_count++;
	}
}

bool hyperhold_space_is_file_arch(void)
{
	return atomic_read(&global_settings.space_type) !=
	       HP_PARTITION_ARCHIVAL;
}

static bool hyperhold_space_map_vaddr(sector_t vaddr, sector_t *addr)
{
	if (vaddr >= hyperhold_get_partition_threshold()) {
		*addr = vaddr - hyperhold_get_partition_threshold() +
			global_settings.backing_bdev[HP_SPACE_PARTITION]
				.start_sector;
	} else if (vaddr >= hyperhold_get_file_threshold()) {
		*addr = vaddr - hyperhold_get_file_threshold() +
			global_settings.backing_bdev[HP_SPACE_FIRST_FILE]
				.start_sector;
	} else {
		hh_print(HHLOG_ERR, "failed vaddr %llu ,addr %llu", vaddr,
			 *addr);
		return false;
	}

	return true;
}

static struct block_device *hyperhold_space_get_bdev_by_vaddr(sector_t vaddr)
{
	if (vaddr >= hyperhold_get_partition_threshold()) {
		return global_settings.backing_bdev[HP_SPACE_PARTITION].bdev;
	} else if (vaddr >= hyperhold_get_file_threshold()) {
		return global_settings.backing_bdev[HP_SPACE_FIRST_FILE].bdev;
	}

	return NULL;
}

static int hyperhold_space_get_index_by_vaddr(sector_t vaddr)
{
	if (vaddr >= hyperhold_get_partition_threshold()) {
		return HP_SPACE_PARTITION;
	} else if (vaddr >= hyperhold_get_file_threshold()) {
		return HP_SPACE_FIRST_FILE;
	}

	return HP_SPACE_MAX_NR;
}

void hyperhold_space_fill_entry_addr(struct hyperhold_entry *io_entry)
{
	if (!hyperhold_space_is_file_arch())
		return;

	io_entry->vaddr = io_entry->addr;
	if (!hyperhold_space_map_vaddr(io_entry->vaddr, &io_entry->addr)) {
		hh_print(HHLOG_ERR, "map vaddr error");
	}
}

bool hyperhold_space_should_merge_ext(struct hyperhold_segment *segment,
				      struct hyperhold_entry *io_entry)
{
	struct hyperhold_entry *head_io_entry, *tail_io_entry;
	int space_index, head_space_index, tail_space_index;

	if (!hyperhold_space_is_file_arch())
		return true;

	space_index = hyperhold_space_get_index_by_vaddr(io_entry->vaddr);
	if (space_index >= HP_SPACE_MAX_NR)
		return false;

	head_io_entry = list_first_entry(&segment->io_entries,
					 struct hyperhold_entry, list);
	tail_io_entry = list_last_entry(&segment->io_entries,
					struct hyperhold_entry, list);
	head_space_index =
		hyperhold_space_get_index_by_vaddr(head_io_entry->vaddr);
	tail_space_index =
		hyperhold_space_get_index_by_vaddr(tail_io_entry->vaddr);

	if (space_index == head_space_index &&
	    space_index == tail_space_index) {
		return true;
	}

	return false;
}

struct block_device *
hyperhold_space_get_bio_bdev(struct hyperhold_segment *segment)
{
	struct hyperhold_entry *io_entry = NULL, *tmp = NULL;
	bool got_vaddr = false;
	sector_t vaddr;

	if (!hyperhold_space_is_file_arch())
		return segment->req->io_para.bdev;

	list_for_each_entry_safe(io_entry, tmp, &segment->io_entries, list) {
		vaddr = io_entry->vaddr;
		got_vaddr = true;
		break;
	}
	if (!got_vaddr)
		return NULL;

	return hyperhold_space_get_bdev_by_vaddr(vaddr);
}

bool hyperhold_space_need_encrypt(struct hyperhold_segment *segment)
{
	struct hyperhold_entry *io_entry = NULL, *tmp = NULL;
	bool got_vaddr = false;
	sector_t vaddr;
	bool need_encrypt = true;

	if (!hyperhold_space_is_file_arch())
		return need_encrypt;

	list_for_each_entry_safe(io_entry, tmp, &segment->io_entries, list) {
		vaddr = io_entry->vaddr;
		got_vaddr = true;
		break;
	}
	if (!got_vaddr)
		return need_encrypt;

	if (vaddr >= hyperhold_get_partition_threshold()) {
		need_encrypt = global_settings.backing_bdev[HP_SPACE_PARTITION]
				       .pddr_continuous;
	} else if (vaddr >= hyperhold_get_file_threshold()) {
		//loopdev do not need encrypt once more, it's done by f2fs.
		need_encrypt = global_settings.backing_bdev[HP_SPACE_FIRST_FILE]
				       .pddr_continuous;
	}

	return need_encrypt;
}

#endif
