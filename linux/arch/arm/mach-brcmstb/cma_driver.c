/*
 *  cma_driver.c - Broadcom STB platform CMA driver
 *
 *  Copyright (C) 2009 - 2013 Broadcom Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/of.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/brcmstb/cma_driver.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>

extern phys_addr_t dma_contiguous_def_base;

struct cma_devs_list {
	struct list_head list;
	struct cma_dev *cma_dev;
};

struct cma_root_dev {
	u32 mmap_type;
	struct cdev cdev;
	struct device *dev;
	struct cma_devs_list cma_devs;
	struct mem_range *cached_region;
};

/* Mutex for serializing accesses to private variables */
static DEFINE_MUTEX(cma_dev_mutex);
static dev_t cma_dev_devno;
static struct class *cma_dev_class;
static struct cma_root_dev *cma_root_dev;

static int cma_dev_open(struct inode *inode, struct file *filp)
{
	dev_dbg(cma_root_dev->dev, "opened cma root device\n");
	return 0;
}

static int cma_dev_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static void cma_dev_vma_open(struct vm_area_struct *vma)
{
	dev_dbg(cma_root_dev->dev, "%s: VA=%lxh PA=%lxh\n", __func__,
		vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

static void cma_dev_vma_close(struct vm_area_struct *vma)
{
	dev_dbg(cma_root_dev->dev, "%s: VA=%lxh PA=%lxh\n", __func__,
		vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

static struct vm_operations_struct cma_dev_vm_ops = {
	.open  = cma_dev_vma_open,
	.close = cma_dev_vma_close,
};

static int cma_dev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	switch (cma_root_dev->mmap_type) {
	case MMAP_TYPE_NORMAL:
		break;
	case MMAP_TYPE_UNCACHED:
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		break;
	case MMAP_TYPE_WC:
#if defined(__arm__)
		/*
		 * ARM has an explicit setting for WC. Use default for other
		 * architectures.
		 */
		vma->vm_page_prot = __pgprot_modify(vma->vm_page_prot,
				    L_PTE_MT_MASK, L_PTE_MT_DEV_WC);
#endif
		break;
	default:
		return -EINVAL;
	}

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start,
			    vma->vm_page_prot)) {
		return -EAGAIN;
	} else {
		vma->vm_ops = &cma_dev_vm_ops;
		cma_dev_vma_open(vma);
	}

	return 0;
}

/*
 * Search through all reserved ranges to check if the page falls
 * completely within any of them.
 */
static int cma_dev_is_page_reserved(struct page *page)
{
	const unsigned long pg_start = page_to_phys(page);
	const unsigned long pg_end = pg_start + PAGE_SIZE;
	struct list_head *dev_pos;
	int match = 0;
	unsigned long range_start;
	unsigned long range_end;

	mutex_lock(&cma_dev_mutex);

	/*
	 * The reserved regions tend to be larger than a page size, so optimize
	 * repeated calls to cma_dev_is_page_reserved() by caching the region
	 * which was determined to be a full-hit.
	 */
	if (cma_root_dev->cached_region) {
		range_start = cma_root_dev->cached_region->base;
		range_end = range_start + cma_root_dev->cached_region->size;
		if (pg_start >= range_start && pg_end <= range_end)
			match = 1;
	}

	if (match)
		goto done;

	list_for_each(dev_pos, &cma_root_dev->cma_devs.list) {
		struct list_head *reg_pos;
		struct cma_dev *curr_cma_dev;

		curr_cma_dev = list_entry(dev_pos, struct cma_dev, list);
		BUG_ON(curr_cma_dev == NULL);

		list_for_each(reg_pos, &curr_cma_dev->regions.list) {
			struct region_list *reg;
			reg = list_entry(reg_pos, struct region_list, list);

			range_start = reg->region.base;
			range_end = range_start + reg->region.size;
			if (pg_start >= range_start && pg_end <= range_end) {
				cma_root_dev->cached_region = &reg->region;
				match = 1;
				break;
			}
		}
	}

done:
	mutex_unlock(&cma_dev_mutex);
	return match;
}

/**
 * cma_dev_get_cma_dev() - Get a cma_dev * by memc index
 *
 * @memc: The MEMC index
 */
struct cma_dev *cma_dev_get_cma_dev(int memc)
{
	struct list_head *pos;
	struct cma_dev *cma_dev = NULL;

	mutex_lock(&cma_dev_mutex);

	list_for_each(pos, &cma_root_dev->cma_devs.list) {
		struct cma_dev *curr_cma_dev;
		curr_cma_dev = list_entry(pos, struct cma_dev, list);
		BUG_ON(curr_cma_dev == NULL);
		if (curr_cma_dev->cma_dev_index == memc) {
			cma_dev = curr_cma_dev;
			break;
		}
	}

	mutex_unlock(&cma_dev_mutex);

	dev_dbg(cma_root_dev->dev, "cma_dev index %d not in list\n", memc);
	return cma_dev;
}
EXPORT_SYMBOL(cma_dev_get_cma_dev);

/**
 * cma_dev_get_mem() - Carve out physical memory
 *
 * @cma_dev: The CMA device
 * @addr: Out pointer which will be populated with the start
 * physical address of the contiguous region that was carved out
 * @len: Number of bytes to allocate
 * @align: Byte alignment
 */
int cma_dev_get_mem(struct cma_dev *cma_dev, u64 *addr, u32 len,
			u32 align)
{
	int status = 0;
	struct page *page;
	struct region_list *new_region;
	struct device *dev = cma_dev->dev;

	if ((len & ~PAGE_MASK) || (len == 0)) {
		dev_dbg(dev, "bad length (%xh)\n", len);
		status = -EINVAL;
		goto done;
	}

	if (align & ~PAGE_MASK) {
		dev_dbg(dev, "bad alignment (%xh)\n", align);
		status = -EINVAL;
		goto done;
	}

	new_region = devm_kzalloc(dev, sizeof(*new_region), GFP_KERNEL);
	if (new_region == NULL) {
		dev_dbg(dev, "devm_kzalloc() failure\n");
		status = -ENOMEM;
		goto done;
	}

	page = dma_alloc_from_contiguous(dev, len >> PAGE_SHIFT,
					 get_order(align));
	if (page == NULL) {
		status = -ENOMEM;
		goto free_new_region;
	}

	*addr = page_to_phys(page);

	/* Accounting */
	mutex_lock(&cma_dev_mutex);
	new_region->region.base = *addr;
	new_region->region.size = len;
	list_add(&new_region->list, &cma_dev->regions.list);
	cma_root_dev->cached_region = NULL;
	mutex_unlock(&cma_dev_mutex);

	goto done;

free_new_region:
	devm_kfree(dev, new_region);

done:
	return status;
}
EXPORT_SYMBOL(cma_dev_get_mem);

/**
 * cma_dev_put_mem() - Return carved out physical memory
 *
 * @cma_dev: The CMA device
 * @addr: Start physical address of allocated region
 * @len: Number of bytes that were allocated (this must match with get_mem
 * call!)
 */
int cma_dev_put_mem(struct cma_dev *cma_dev, u64 addr, u32 len)
{
	struct page *page = phys_to_page(addr);
	struct list_head *pos;
	int match = 0;
	struct device *dev = cma_dev->dev;
	struct region_list *region_entry = NULL;
	int status = 0;

	if (page == NULL) {
		dev_dbg(cma_root_dev->dev, "bad addr (%llxh)\n", addr);
		return -EINVAL;
	}

	if (len % PAGE_SIZE) {
		dev_dbg(cma_root_dev->dev, "bad length (%xh)\n", len);
		return -EINVAL;
	}

	/* Accounting - confirm address has been allocated */
	mutex_lock(&cma_dev_mutex);
	list_for_each(pos, &cma_dev->regions.list) {
		struct mem_range *region;

		region_entry = list_entry(pos, struct region_list, list);
		region = &region_entry->region;

		if ((region->base == addr) && (region->size == len)) {
			match = 1;
			break;
		}
	}

	if (match == 0) {
		dev_err(dev, "no region matched that address\n");
		status = -EINVAL;
		goto done;
	}

	if (!dma_release_from_contiguous(dev, page, len / PAGE_SIZE)) {
		status = -EIO;
		goto done;
	}

	cma_root_dev->cached_region = NULL;
	list_del(pos);
	devm_kfree(dev, region_entry);

done:
	mutex_unlock(&cma_dev_mutex);
	return status;
}
EXPORT_SYMBOL(cma_dev_put_mem);

/**
 * cma_dev_get_num_regions() - Get number of allocated regions
 *
 * @cma_dev: The CMA device
 */
int cma_dev_get_num_regions(struct cma_dev *cma_dev)
{
	int count = 0;
	struct list_head *pos;

	list_for_each(pos, &cma_dev->regions.list)
		count++;

	return count;
}
EXPORT_SYMBOL(cma_dev_get_num_regions);

/**
 * cma_dev_get_region_info() - Get information about allocate region
 *
 * @cma_dev: The CMA device
 * @region_num: Region index
 * @memc: MEMC index associated with the region
 * @addr: Physical address of region
 * @num_bytes: Size of region in bytes
 */
int cma_dev_get_region_info(struct cma_dev *cma_dev, int region_num,
				   u32 *memc, u64 *addr, u32 *num_bytes)
{
	int status = -EINVAL;
	struct list_head *pos;
	int count = region_num;

	list_for_each(pos, &cma_dev->regions.list) {
		struct region_list *region;
		if (count == 0) {
			region = list_entry(pos, struct region_list, list);

			*memc = cma_dev->cma_dev_index;
			*addr = region->region.base;
			*num_bytes = region->region.size;

			status = 0;

			break;
		} else
			count--;
	}

	return status;
}
EXPORT_SYMBOL(cma_dev_get_region_info);

/**
 * Special handling for __get_user_pages() on CMA reserved memory:
 *
 * 1) Override the VM_IO | VM_PFNMAP sanity checks
 * 2) No cache flushes (this is explicitly under application control)
 * 3) vm_normal_page() does not work on these regions
 * 4) Don't need to worry about any kinds of faults; pages are always present
 *
 * The vanilla kernel behavior was to prohibit O_DIRECT operations on our
 * CMA regions, but direct I/O is absolutely required for PVR and video
 * playback from SATA/USB.
 */
int cma_dev_get_page(struct mm_struct *mm, struct vm_area_struct *vma,
	unsigned long start, struct page **page)
{
	unsigned long pg = start & PAGE_MASK, pfn;
	int ret = -EFAULT;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;
	struct page *tmp_page;

	pgd = pgd_offset(mm, pg);
	BUG_ON(pgd_none(*pgd));
	pud = pud_offset(pgd, pg);
	BUG_ON(pud_none(*pud));
	pmd = pmd_offset(pud, pg);
	if (pmd_none(*pmd))
		return ret;

	pte = pte_offset_map(pmd, pg);
	if (!pte)
		return ret;

	if (pte_none(*pte))
		goto out;

	pfn = pte_pfn(*pte);

	tmp_page = pfn_to_page(pfn);
	if (!tmp_page)
		goto out;

	if (get_pageblock_migratetype(tmp_page) != MIGRATE_CMA)
		goto out;

	if (!cma_dev_is_page_reserved(tmp_page))
		goto out;

	if (page) {
		*page = tmp_page;
		get_page(*page);
	}
	ret = 0;

out:
	pte_unmap(pte);
	return ret;
}

static int pte_callback(pte_t *pte, unsigned long x, unsigned long y,
			struct mm_walk *walk)
{
	const pgprot_t pte_prot = __pgprot(*pte);
	const pgprot_t req_prot = (pgprot_t)walk->private;
	const pgprot_t prot_msk = L_PTE_MT_MASK | L_PTE_VALID;
	return (((pte_prot ^ req_prot) & prot_msk) == 0) ? 0 : -1;
}

static void *page_to_virt_contig(const struct page *page, unsigned int pg_cnt,
					pgprot_t pgprot)
{
	int rc;
	struct mm_walk walk;
	unsigned long pfn;
	unsigned long pfn_start;
	unsigned long pfn_end;
	unsigned long va_start;
	unsigned long va_end;

	if ((page == NULL) || !pg_cnt)
		return ERR_PTR(-EINVAL);

	pfn_start = page_to_pfn(page);
	pfn_end = pfn_start + pg_cnt;
	for (pfn = pfn_start; pfn < pfn_end; pfn++) {
		const struct page *cur_pg = pfn_to_page(pfn);
		phys_addr_t pa;

		/* Verify range is in low memory only */
		if (PageHighMem(cur_pg))
			return NULL;

		/* Must be mapped */
		pa = page_to_phys(cur_pg);
		if (page_address(cur_pg) == NULL)
			return NULL;
	}

	/*
	 * Aliased mappings with different cacheability attributes on ARM can
	 * lead to trouble!
	 */
	memset(&walk, 0, sizeof(walk));
	walk.pte_entry = &pte_callback;
	walk.private = (void *)pgprot;
	walk.mm = current->mm;
	va_start = (unsigned long)page_address(page);
	va_end = (unsigned long)(page_address(page) + (pg_cnt << PAGE_SHIFT));
	rc = walk_page_range(va_start,
			     va_end,
			     &walk);
	if (rc)
		pr_debug("cacheability mismatch\n");

	return rc ? NULL : page_address(page);
}

static int cma_dev_ioctl_check_cmd(unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (_IOC_TYPE(cmd) != CMA_DEV_MAGIC)
		return -ENOTTY;

	if (_IOC_NR(cmd) > CMA_DEV_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void __user *)arg,
				 _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void __user *)arg,
				 _IOC_SIZE(cmd));

	if (ret)
		return -EFAULT;

	return 0;
}

static struct page **get_pages(struct page *page, int num_pages)
{
	struct page **pages;
	long pfn;
	int i;

	if (num_pages == 0) {
		pr_err("bad count\n");
		return NULL;
	}

	if (page == NULL) {
		pr_err("bad page\n");
		return NULL;
	}

	pages = kmalloc(sizeof(struct page *) * num_pages, GFP_KERNEL);
	if (pages == NULL) {
		pr_err("cannot alloc pages\n");
		return NULL;
	}

	pfn = page_to_pfn(page);
	for (i = 0; i < num_pages; i++) {
		/*
		 * pfn_to_page() should resolve to simple arithmetic for the
		 * FLATMEM memory model.
		 */
		pages[i] = pfn_to_page(pfn++);
	}

	return pages;
}

static void put_pages(struct page **pages)
{
	if (pages == NULL) {
		pr_err("null ptr\n");
		return;
	}

	kfree(pages);
}

/**
 * cma_dev_kva_map() - Map page(s) to a kernel virtual address
 *
 * @page: A struct page * that points to the beginning of a chunk of physical
 * contiguous memory.
 * @num_pages: Number of pages
 * @pgprot: Page protection bits
 */
void *cma_dev_kva_map(struct page *page, int num_pages, pgprot_t pgprot)
{
	void *va;

	if (cma_root_dev->dev == NULL) {
		pr_err("cma root dev not initialized\n");
		return NULL;
	}

	/* get the virtual address for this range if it exists */
	va = page_to_virt_contig(page, num_pages, pgprot);
	if (IS_ERR(va)) {
		pr_debug("page_to_virt_contig() failed (%ld)\n", PTR_ERR(va));
		return NULL;
	} else if (va == NULL || is_vmalloc_addr(va)) {
		struct page **pages;

		pages = get_pages(page, num_pages);
		if (pages == NULL) {
			pr_err("couldn't get pages\n");
			return NULL;
		}

		va = vmap(pages, num_pages, 0, pgprot);

		put_pages(pages);

		if (va == NULL) {
			pr_err("vmap failed (num_pgs=%d)\n", num_pages);
			return NULL;
		}
	}

	return va;
}
EXPORT_SYMBOL(cma_dev_kva_map);

/**
 * cma_dev_kva_unmap() - Unmap a kernel virtual address associated
 * to physical pages mapped by cma_dev_kva_map()
 *
 * @kva: Kernel virtual address previously mapped by cma_dev_kva_map()
 */
int cma_dev_kva_unmap(const void *kva)
{
	if (cma_root_dev->dev == NULL) {
		pr_err("cma root dev not initialized\n");
		return -EFAULT;
	}

	if (kva == NULL)
		return -EINVAL;

	if (!is_vmalloc_addr(kva)) {
		/* unmapping not necessary for low memory VAs */
		return 0;
	}

	vunmap(kva);

	return 0;
}
EXPORT_SYMBOL(cma_dev_kva_unmap);

static long cma_dev_ioctl(struct file *filp, unsigned int cmd,
			  unsigned long arg)
{
	int ret = 0;
	struct device *dev = cma_root_dev->dev;
	struct cma_dev *cma_dev;
	struct ioc_params p;

	ret = cma_dev_ioctl_check_cmd(cmd, arg);
	if (ret)
		return ret;

	if (cmd < CMA_DEV_IOC_GET_PG_PROT) {
		if (copy_from_user(&p, (void __user *)(arg), sizeof(p)))
			return -EFAULT;
	}

	switch (cmd) {
	case CMA_DEV_IOC_GETMEM: {
		ret = -EINVAL;

		cma_dev = cma_dev_get_cma_dev(p.cma_dev_index);
		if (cma_dev == NULL)
			break;

		p.status = cma_dev_get_mem(cma_dev, &p.addr, p.num_bytes,
						p.align_bytes);

		ret = 0;

		break;
	}
	case CMA_DEV_IOC_PUTMEM: {
		ret = -EINVAL;

		cma_dev = cma_dev_get_cma_dev(p.cma_dev_index);
		if (cma_dev == NULL)
			break;

		p.status = cma_dev_put_mem(cma_dev, p.addr, p.num_bytes);

		ret = 0;

		break;
	}
	case CMA_DEV_IOC_GETPHYSINFO: {
		ret = -EINVAL;

		cma_dev = cma_dev_get_cma_dev(p.cma_dev_index);
		if (cma_dev == NULL)
			break;

		p.addr = cma_dev->range.base;
		p.num_bytes = cma_dev->range.size;
		p.status = 0;

		ret = 0;

		break;
	}
	case CMA_DEV_IOC_GETNUMREGS: {
		ret = -EINVAL;

		cma_dev = cma_dev_get_cma_dev(p.cma_dev_index);
		if (cma_dev == NULL)
			break;

		p.num_regions = cma_dev_get_num_regions(cma_dev);

		ret = 0;

		break;
	}
	case CMA_DEV_IOC_GETREGINFO: {
		ret = -EINVAL;

		cma_dev = cma_dev_get_cma_dev(p.cma_dev_index);
		if (cma_dev == NULL)
			break;

		p.status = cma_dev_get_region_info(cma_dev, p.region, &p.memc,
							&p.addr, &p.num_bytes);

		ret = 0;

		break;
	}
	case CMA_DEV_IOC_GET_PG_PROT: {
		__put_user(cma_root_dev->mmap_type, (u32 __user *)arg);
		break;
	}
	case CMA_DEV_IOC_SET_PG_PROT: {
		int mmap_type;

		__get_user(mmap_type, (u32 __user *)arg);

		if (mmap_type > MMAP_TYPE_WC) {
			dev_err(dev, "bad mmap_type (%d)\n", mmap_type);
			ret = -EINVAL;
		} else
			cma_root_dev->mmap_type = mmap_type;

		break;
	}
	default: {
		return -ENOTTY;
	}
	}

	if (cmd < CMA_DEV_IOC_GET_PG_PROT) {
		if (copy_to_user((void __user *)(arg), &p, sizeof(p)))
			return -EFAULT;
	}

	return ret;
}

static const struct file_operations cma_dev_fops = {
	.owner          = THIS_MODULE,
	.open           = cma_dev_open,
	.release        = cma_dev_release,
	.mmap           = cma_dev_mmap,
	.unlocked_ioctl = cma_dev_ioctl,
};

static int cma_drvr_alloc_devno(struct device *dev)
{
	int ret = 0;

	if (MAJOR(cma_dev_devno) == 0) {
		ret = alloc_chrdev_region(&cma_dev_devno, 0, CMA_DEV_MAX,
					  CMA_DEV_NAME);
	}

	if (ret) {
		dev_err(dev, "couldn't alloc major devno\n");
		return ret;
	}

	dev_dbg(dev, "maj=%d min=%d\n", MAJOR(cma_dev_devno),
	MINOR(cma_dev_devno));

	return ret;
}

static int cma_drvr_get_node_num(const char *p)
{
	char *d = strchr(p, '@');
	long val;

	if (!d || kstrtol(d + 1, 10, &val))
		return -EINVAL;
	return val;
}

static int cma_drvr_test_cma_dev(struct device *dev)
{
	struct page *page;

	/* Do a dummy alloc to ensure the CMA device is truly ready */
	page = dma_alloc_from_contiguous(dev, 1, 0);
	if (page == NULL)
		return -EINVAL;

	if (!dma_release_from_contiguous(dev, page, 1)) {
		dev_err(dev, "test dma release failed!\n");
		return -EINVAL;
	}

	return 0;
}

static int cma_drvr_parse_fdt(struct platform_device *pdev,
				     struct cma_dev *cma_dev)
{
	int minor;
	struct device *dev = &pdev->dev;
	struct device_node *of_node = pdev->dev.of_node;
	struct device_node *cma_region_node;
	const __be32 *prop_reg;

	/*
	 * Lookup the linked CMA region node and get the memory range
	 * associated with it.
	 */

	cma_region_node = of_parse_phandle(of_node, "linux,contiguous-region",
					   0);
	if (cma_region_node == NULL) {
		dev_err(dev, "missing phandle to cma region node\n");
		return -EINVAL;
	}

	prop_reg = of_get_property(cma_region_node, "reg", NULL);
	if (prop_reg == NULL) {
		dev_err(dev, "missing reg prop\n");
		return -EINVAL;
	}

	/*
	 * Obtain the base address from dma-contiguous when node is marked
	 * as a default region because it is dynamically placed by the kernel.
	 */
	if (of_get_property(cma_region_node,
		"linux,default-contiguous-region", NULL))
		cma_dev->range.base = dma_contiguous_def_base;
	else
		cma_dev->range.base = be32_to_cpup(prop_reg);

	cma_dev->range.size = be32_to_cpup(prop_reg + 1);

	dev_info(dev, "base=%xh size=%xh\n", cma_dev->range.base,
		cma_dev->range.size);

	/* Derive the node number from device tree */
	minor = cma_drvr_get_node_num(of_node_full_name(of_node));
	if ((minor < 0) || (minor > CMA_DEV_MAX)) {
		dev_err(dev, "node num (%d) exceeds supported (%d)\n", minor,
			CMA_DEV_MAX);
		return -EINVAL;
	}
	cma_dev->cma_dev_index = minor;

	if (cma_drvr_test_cma_dev(dev)) {
		dev_err(dev, "test CMA dev failed!\n");
		return -EINVAL;
	}

	return 0;
}

static int cma_drvr_init_root_dev(struct device *dev)
{
	int ret;
	struct device *dev2;
	struct cma_root_dev *my_cma_root_dev;
	struct cdev *cdev;
	int minor = 0;

	my_cma_root_dev = devm_kzalloc(dev, sizeof(*my_cma_root_dev),
					GFP_KERNEL);
	if (my_cma_root_dev == NULL)
		return -ENOMEM;

	/* Initialize list of CMA devices referenced by the root CMA device */
	INIT_LIST_HEAD(&my_cma_root_dev->cma_devs.list);

	/* Setup character device */
	cdev = &my_cma_root_dev->cdev;
	cdev_init(cdev, &cma_dev_fops);
	cdev->owner = THIS_MODULE;
	ret = cdev_add(cdev, cma_dev_devno + minor, 1);
	if (ret) {
		dev_err(dev, "cdev_add() failed (%d)\n", ret);
		goto free_cma_root_dev;
	}

	if (cma_dev_class == NULL)
		cma_dev_class = class_create(THIS_MODULE, CMA_CLASS_NAME);

	/* Setup device */
	dev2 = device_create(cma_dev_class, dev, cma_dev_devno + minor, NULL,
			     CMA_DEV_NAME"%d", minor);
	if (IS_ERR(dev2)) {
		dev_err(dev, "error creating device (%d)\n", ret);
		ret = PTR_ERR(dev2);
		goto del_cdev;
	}

	my_cma_root_dev->cached_region = NULL;
	my_cma_root_dev->dev = dev2;
	cma_root_dev = my_cma_root_dev;

	dev_info(dev, "Initialized Broadcom CMA root device\n");

	goto done;

del_cdev:
	cdev_del(cdev);

free_cma_root_dev:
	devm_kfree(dev, cma_root_dev);

done:
	return ret;
}

static int cma_drvr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cma_dev *cma_dev = NULL;
	int ret = 0;

	mutex_lock(&cma_dev_mutex);

	/* Prepare a major number for the devices if not already done */
	ret = cma_drvr_alloc_devno(dev);
	if (ret)
		goto free_cma_dev;

	/* Initialize the root device only once */
	if (cma_root_dev == NULL)
		ret = cma_drvr_init_root_dev(dev);

	if (ret)
		goto done;

	cma_dev = devm_kzalloc(dev, sizeof(*cma_dev), GFP_KERNEL);
	if (cma_dev == NULL) {
		dev_err(dev, "can't alloc cma_dev\n");
		ret = -ENOMEM;
		goto done;
	}

	/* Extract information from the FDT and instantiate the devices */
	ret = cma_drvr_parse_fdt(pdev, cma_dev);
	if (ret)
		goto free_cma_dev;

	INIT_LIST_HEAD(&cma_dev->regions.list);
	cma_dev->dev = dev;
	dev->platform_data = cma_dev;

	/*
	 * Keep a pointer to all of the devs so we don't have to search for it
	 * elsewhere.
	 */
	INIT_LIST_HEAD(&cma_dev->list);
	list_add(&cma_dev->list, &cma_root_dev->cma_devs.list);
	dev_info(dev, "Added CMA device @ PA=0x%x LEN=%xh\n",
		 cma_dev->range.base, cma_dev->range.size);

	goto done;

free_cma_dev:
	devm_kfree(dev, cma_dev);

done:
	mutex_unlock(&cma_dev_mutex);
	return ret;
}

static const struct of_device_id cma_drvr_of_match[] = {
	{ .compatible = "brcm,cma-plat-dev", },
	{},
};

static struct platform_driver cma_drvr = {
	.probe = cma_drvr_probe,
	.driver = {
		.name = "cma-plat-drvr",
		.owner = THIS_MODULE,
		.of_match_table = cma_drvr_of_match,
	},
};
module_platform_driver(cma_drvr);
