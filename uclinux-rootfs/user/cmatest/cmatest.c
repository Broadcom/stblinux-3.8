#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <assert.h>
#include <signal.h>
#include <errno.h>
#include "cmatest.h"

static int cma_get_mem(int fd, uint32_t dev_index, uint32_t num_bytes,
			uint32_t align_bytes, uint64_t *addr)
{
	int ret;
	struct ioc_params get_mem_p;

	memset(&get_mem_p, 0, sizeof(get_mem_p));

	get_mem_p.cma_dev_index = dev_index;
	get_mem_p.num_bytes = num_bytes;
	get_mem_p.align_bytes = align_bytes;

	ret = ioctl(fd, CMA_DEV_IOC_GETMEM, &get_mem_p);
	if (ret != 0) {
		printf("	ioctl failed (%d)\n", ret);
		return ret;
	}

	*addr = get_mem_p.addr;

	ret = get_mem_p.status;
	if (ret == 0)
		printf("	alloc PA=%llxh LEN=%xh\n", *addr, num_bytes);
	else
		printf("	alloc PA=%llxh LEN=%xh failed (%d)\n", *addr,
			num_bytes, ret);

	return ret;
}

static int cma_put_mem(int fd, uint32_t dev_index, uint64_t addr,
	uint32_t num_bytes)
{
	int ret;
	struct ioc_params put_mem_p;

	memset(&put_mem_p, 0, sizeof(put_mem_p));

	put_mem_p.cma_dev_index = dev_index;
	put_mem_p.addr = addr;
	put_mem_p.num_bytes = num_bytes;

	ret = ioctl(fd, CMA_DEV_IOC_PUTMEM, &put_mem_p);
	if (ret)
		return ret;

	ret = put_mem_p.status;
	if (ret == 0)
		printf("	freed PA=%llxh LEN=%xh\n", addr, num_bytes);
	else
		printf("	freed PA=%llxh LEN=%xh failed (%d)\n", addr,
			num_bytes, ret);

	return ret;
}

static int cma_get_phys_info(int fd, uint32_t dev_index, uint64_t *addr,
	uint32_t *num_bytes)
{
	int ret;
	struct ioc_params physinfo_p;

	memset(&physinfo_p, 0, sizeof(physinfo_p));

	physinfo_p.cma_dev_index = dev_index;

	ret = ioctl(fd, CMA_DEV_IOC_GETPHYSINFO, &physinfo_p);
	if (ret)
		return ret;

	ret = physinfo_p.status;
	if (ret == 0) {
		*addr = physinfo_p.addr;
		*num_bytes = physinfo_p.num_bytes;
		printf("	physinfo PA=%llxh LEN=%xh\n", *addr,
			*num_bytes);
	} else
		printf("	getphysinfo failed\n");

	return ret;
}

static int cma_get_num_regions(int fd, uint32_t dev_index, uint32_t *num)
{
	int ret;
	struct ioc_params getnumregs_p;
	memset(&getnumregs_p, 0, sizeof(getnumregs_p));
	getnumregs_p.cma_dev_index = dev_index;

	ret = ioctl(fd, CMA_DEV_IOC_GETNUMREGS, &getnumregs_p);
	if (ret)
		return ret;

	ret = getnumregs_p.status;
	*num = getnumregs_p.num_regions;

	return ret;
}

static int cma_get_region_info(int fd, uint32_t dev_index, uint32_t region_num,
				uint32_t *memc, uint64_t *addr,
				uint32_t *num_bytes)
{
	int ret;
	struct ioc_params getreginfo_p;
	memset(&getreginfo_p, 0, sizeof(getreginfo_p));
	getreginfo_p.cma_dev_index = dev_index;
	getreginfo_p.region = region_num;

	ret = ioctl(fd, CMA_DEV_IOC_GETREGINFO, &getreginfo_p);
	if (ret)
		return ret;

	ret = getreginfo_p.status;

	*memc = getreginfo_p.memc;
	*addr = getreginfo_p.addr;
	*num_bytes = getreginfo_p.num_bytes;

	if (!ret) {
		printf("	memc=%d addr=%llxh num_bytes=%xh\n", *memc,
			*addr, *num_bytes);
	} else
		printf("	%s failed (%d)\n", __func__, ret);

	return ret;
}

static void reset_all(int fd, uint32_t cma_idx)
{
	uint32_t count;

	if (cma_get_num_regions(fd, cma_idx, &count) != 0)
		return;

	while (count != 0) {
		int ret;
		uint32_t memc;
		uint64_t addr;
		uint32_t len;

		ret = cma_get_region_info(fd, cma_idx, 0, &memc, &addr, &len);
		if (ret == 0) {
			assert(cma_put_mem(fd, cma_idx, addr, len) == 0);
			assert(cma_get_num_regions(fd, cma_idx, &count) == 0);
		} else
			break;
	}
}

static void run_unit_tests(int fd)
{
	int i;
	uint64_t addr[32];
	uint32_t len[32];
	uint32_t x;
	uint64_t y;

	/* === TEST CASE 1 === */

	printf("t: alloc (1) region\n");
	len[0] = 0x1000;
	len[1] = 0x1000;
	assert(cma_get_mem(fd, 1, len[0], 0, &addr[0]) == 0);
	printf("ok\n\n");

	printf("t: verify (1) region allocated\n");
	assert(cma_get_num_regions(fd, 1, &x) == 0);
	assert(x == 1);
	printf("ok\n\n");

	printf("t: free (1) region\n");
	assert(cma_put_mem(fd, 1, addr[0], len[0]) == 0);
	printf("ok\n\n");

	/* === TEST CASE 2 === */

	printf("t: alloc (2) regions w/ 1MB alignment\n");
	for (i = 0; i < 2; i++) {
		len[i] = 0x1000;
		assert(cma_get_mem(fd, 1, len[i], 1 * 1024 * 1024, &addr[i])
			== 0);
	}
	printf("ok\n\n");

	printf("t: verify (2) regions allocated\n");
	assert(cma_get_num_regions(fd, 1, &x) == 0);
	assert(x == 2);
	printf("ok\n\n");

	printf("t: verify (2) regions have expected alignment and size\n");
	for (i = 0; i < 2; i++) {
		uint32_t memc;
		uint64_t addr;
		uint32_t num_bytes;

		assert(cma_get_region_info(fd, 1, i, &memc, &addr, &num_bytes)
			== 0);
		assert((addr % (1 * 1024 * 1024)) == 0);
		assert(num_bytes == len[i]);
	}
	printf("ok\n\n");

	printf("t: free (2) region\n");
	for (i = 0; i < 2; i++)
		assert(cma_put_mem(fd, 1, addr[i], len[i]) == 0);
	printf("ok\n\n");

	/* === TEST CASE 3 === */

	printf("t: alloc and free (2) regions\n");
	len[0] = 0x1000;
	len[1] = 0x2000;
	for (i = 0; i < 2; i++)
		assert(cma_get_mem(fd, 1, len[i], 0, &addr[i]) == 0);
	for (i = 0; i < 2; i++)
		assert(cma_put_mem(fd, 1, addr[i], len[i]) == 0);
	printf("ok\n\n");

	/* === TEST CASE 4 === */

	printf("t: alloc and attempt free with mismatched length\n");
	len[0] = 0x1000;
	assert(cma_get_mem(fd, 1, len[0], 0, &addr[0]) == 0);
	assert(cma_put_mem(fd, 1, addr[0], len[0] + 1) == -EINVAL);
	assert(cma_put_mem(fd, 1, addr[0], len[0]) == 0);
	printf("ok\n\n");

	/* === TEST CASE 5 === */

	printf("t: alloc (5) and free in same order\n");
	for (i = 0; i < 5; i++) {
		len[i] = (i + 1) * 0x1000;
		assert(cma_get_mem(fd, 1, len[i], 0, &addr[i]) == 0);
	}
	for (i = 0; i < 5; i++)
		assert(cma_put_mem(fd, 1, addr[i], len[i]) == 0);
	printf("ok\n\n");

	printf("t: verify (0) regions allocated\n");
	assert(cma_get_num_regions(fd, 1, &x) == 0);
	assert(x == 0);
	printf("ok\n\n");

	/* === TEST CASE 6 === */

	printf("t: alloc (5) and free in reverse order\n");
	for (i = 0; i < 5; i++) {
		len[i] = (i + 1) * 0x1000;
		assert(cma_get_mem(fd, 1, len[i], 0, &addr[i]) == 0);
	}
	for (i = 4; i >= 0; i--)
		assert(cma_put_mem(fd, 1, addr[i], len[i]) == 0);
	printf("ok\n\n");

	printf("t: verify (0) regions allocated\n");
	assert(cma_get_num_regions(fd, 1, &x) == 0);
	assert(x == 0);
	printf("ok\n\n");

	/* === TEST CASE 7 === */

	printf("t: alloc (3), free middle\n");
	for (i = 0; i < 3; i++) {
		len[i] = (i + 1) * 0x1000;
		assert(cma_get_mem(fd, 1, len[i], 0, &addr[i]) == 0);
	}
	assert(cma_put_mem(fd, 1, addr[1], len[1]) == 0);
	printf("ok\n\n");

	printf("t: verify (2) regions allocated\n");
	assert(cma_get_num_regions(fd, 1, &x) == 0);
	assert(x == 2);
	printf("ok\n\n");

	printf("t: free the remaining (2) regions, verify (0) regions\n");
	assert(cma_put_mem(fd, 1, addr[0], len[0]) == 0);
	assert(cma_put_mem(fd, 1, addr[2], len[2]) == 0);
	assert(cma_get_num_regions(fd, 1, &x) == 0);
	assert(x == 0);
	printf("ok\n\n");

	/* === TEST CASE 8 === */

	printf("t: attempt retrieval of invalid region info\n");
	assert(cma_get_region_info(fd, 1, 42, &x, &y, &x) == -EINVAL);
	printf("ok\n\n");

	/* === TEST CASE 9 === */
	printf("t: attempt retrieval of invalid phys info\n");
	assert(cma_get_phys_info(fd, 42, &y, &x) == -1);
	printf("ok\n\n");

	printf("UNIT TEST PASSED!\n");
}

static void cma_mmap(int fd, uint32_t base, uint32_t len)
{
	uint32_t *mem;

	printf("cma_mmap() BASE=%xh LEN=%xh\n", base, len);

	mem = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, base);
	if (mem == NULL) {
		printf("mmap failed\n");
		return;
	}

	*mem++ = 0xDEADBEEF;
	*mem++ = 0xDEADBEEF;
	*mem++ = 0xDEADBEEF;
	*mem++ = 0xDEADBEEF;

	munmap(mem, len);
}

static void show_usage(char *argv[])
{
	printf("usage: %s <command> <device> <args...>\n", argv[0]);
	printf("\ncommands <args...>:\n");
	printf("    alloc   <cmadevidx> 0x<num_bytes> 0x<align_bytes>\n");
	printf("    free    <cmadevidx> 0x<PA> 0x<num_bytes>\n");
	printf("    list    <cmadevidx>\n");
	printf("    getprot\n");
	printf("    setprot <pg_prot val>\n");
	printf("    unittest\n");
	printf("    resetall\n");
	printf("    mmap    0x<addr> 0x<len>\n");
	printf("\nexamples:\n");
	printf("    %s /dev/brcm_cma0 alloc 2 0x1000 0x0\n", argv[0]);
	printf("    %s /dev/brcm_cma0 unittest\n\n", argv[0]);
}

int main(int argc, char *argv[])
{
	int fd;
	int ret;

	if (argc < 2) {
		show_usage(argv);
		return -1;
	}

	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		printf("couldn't open device %s\n", argv[1]);
		goto done;
	}

	if (strcmp(argv[2], "alloc") == 0) {
		uint32_t cma_dev_index;
		uint32_t num_bytes;
		uint32_t align_bytes;
		uint64_t addr;

		if (argc != 6) {
			show_usage(argv);
			return -1;
		}

		sscanf(argv[3], "%x", &cma_dev_index);
		sscanf(argv[4], "%x", &num_bytes);
		sscanf(argv[5], "%x", &align_bytes);

		ret = cma_get_mem(fd, cma_dev_index, num_bytes, align_bytes,
			&addr);
		if (ret) {
			printf("bug @ line %d (%d)\n", __LINE__, ret);
			goto done;
		}

		printf("PA=%llxh\n", addr);
	} else if (strcmp(argv[2], "free") == 0) {
		uint32_t cma_dev_index;
		uint64_t addr;
		uint32_t num_bytes;

		if (argc != 6) {
			show_usage(argv);
			return -1;
		}

		sscanf(argv[3], "%x", &cma_dev_index);
		sscanf(argv[4], "%llx", &addr);
		sscanf(argv[5], "%x", &num_bytes);

		ret = cma_put_mem(fd, cma_dev_index, addr, num_bytes);
		if (ret) {
			printf("bug @ line %d (%d)\n", __LINE__, ret);
			goto done;
		}
	} else if (strcmp(argv[2], "list") == 0) {
		unsigned int i;
		uint32_t cma_dev_index;
		uint32_t num_regions;
		uint64_t addr;
		uint32_t len;

		if (argc != 4) {
			show_usage(argv);
			return -1;
		}

		sscanf(argv[3], "%x", &cma_dev_index);

		ret = cma_get_phys_info(fd, cma_dev_index, &addr, &len);
		if (ret) {
			printf("bug @ line %d (%d)\n", __LINE__, ret);
			goto done;
		}

		ret = cma_get_num_regions(fd, cma_dev_index, &num_regions);
		if (ret) {
			printf("bug @ line %d (%d)\n", __LINE__, ret);
			goto done;
		}

		printf("num regions = %d\n", num_regions);

		for (i = 0; i < num_regions; i++) {
			uint32_t memc;
			uint64_t addr;
			uint32_t num_bytes;

			ret = cma_get_region_info(fd, cma_dev_index, i, &memc,
						&addr, &num_bytes);
			if (ret) {
				printf("bug @ line %d (%d)\n", __LINE__, ret);
				goto done;
			}

			printf("memc[%d]      = %xh\n", i, memc);
			printf("addr[%d]      = %llxh\n", i, addr);
			printf("num_bytes[%d] = %xh\n", i, num_bytes);
		}
	} else if (strcmp(argv[2], "getprot") == 0) {
		uint32_t x;

		ret = ioctl(fd, CMA_DEV_IOC_GET_PG_PROT, &x);
		if (ret) {
			printf("bug @ line %d (%d)\n", __LINE__, ret);
			goto done;
		}

		printf("pg_prot is %d\n", x);
	} else if (strcmp(argv[2], "setprot") == 0) {
		uint32_t x;

		if (argc != 4) {
			show_usage(argv);
			return -1;
		}

		sscanf(argv[3], "%x", &x);

		ret = ioctl(fd, CMA_DEV_IOC_SET_PG_PROT, &x);
		if (ret) {
			printf("bug @ line %d (%d)\n", __LINE__, ret);
			goto done;
		}
	} else if (strcmp(argv[2], "unittest") == 0) {
		run_unit_tests(fd);
	} else if (strcmp(argv[2], "resetall") == 0) {
		reset_all(fd, 0);
		reset_all(fd, 1);
		reset_all(fd, 2);
	} else if (strcmp(argv[2], "mmap") == 0) {
		uint32_t base;
		uint32_t len;

		sscanf(argv[3], "%x", &base);
		sscanf(argv[4], "%x", &len);

		cma_mmap(fd, base, len);
	}

	close(fd);

done:
	return 0;
}
