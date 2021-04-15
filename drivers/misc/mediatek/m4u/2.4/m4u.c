/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/fb.h>
/* #include <linux/earlysuspend.h> */
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <mt-plat/sync_write.h>
#include <asm/cacheflush.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/dma-direction.h>
#include <asm/page.h>
#include <linux/proc_fs.h>
#ifndef CONFIG_ARM64
#include "mm/dma.h"
#endif

#include "m4u_priv.h"
#include "m4u.h"
#include "m4u_hw.h"

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#if IS_ENABLED(CONFIG_COMPAT)
#include <linux/uaccess.h>
#include <linux/compat.h>
#endif

static struct m4u_buf_info_t gMvaNode_unknown = {
	.va = 0,
	.mva = 0,
	.size = 0,
	.port = M4U_PORT_UNKNOWN,
};

/* -------------------------------------Global variables------------------------------------------------// */
#ifdef M4U_PROFILE
mmp_event M4U_MMP_Events[M4U_MMP_MAX];
#endif

#define M4U_DEV_NAME "m4u"
struct m4u_device *gM4uDev;

static int m4u_buf_show(void *priv, unsigned int mva_start, unsigned int mva_end, void *data)
{
	struct m4u_buf_info_t *pMvaInfo = priv;

	M4U_PRINT_LOG_OR_SEQ(data, "0x%-8x, 0x%-8x, 0x%lx, 0x%-8x, 0x%x, %s, 0x%x, 0x%x, 0x%x\n",
			pMvaInfo->mva, pMvaInfo->mva+pMvaInfo->size-1, pMvaInfo->va,
			pMvaInfo->size, pMvaInfo->prot, m4u_get_port_name(pMvaInfo->port),
			     pMvaInfo->flags, mva_start, mva_end);

	return 0;
}


int m4u_dump_buf_info(struct seq_file *seq)
{

	M4U_PRINT_LOG_OR_SEQ(seq, "\ndump mva allocated info ========>\n");
	M4U_PRINT_LOG_OR_SEQ(seq,
	"mva_start   mva_end          va       size     prot   module   flags   debug1  debug2\n");

	mva_foreach_priv((void *) m4u_buf_show, seq);

	M4U_PRINT_LOG_OR_SEQ(seq, " dump mva allocated info done ========>\n");
	return 0;
}

#ifdef M4U_PROFILE
static void m4u_profile_init(void)
{
	mmp_event M4U_Event;

	mmprofile_enable(1);
	M4U_Event = mmprofile_register_event(MMP_ROOT_EVENT, "M4U");
	/* register events */
	M4U_MMP_Events[M4U_MMP_ALLOC_MVA] = mmprofile_register_event(M4U_Event, "Alloc MVA");
	M4U_MMP_Events[M4U_MMP_DEALLOC_MVA] = mmprofile_register_event(M4U_Event, "DeAlloc MVA");
	M4U_MMP_Events[M4U_MMP_CONFIG_PORT] = mmprofile_register_event(M4U_Event, "Config Port");
	M4U_MMP_Events[M4U_MMP_M4U_ERROR] = mmprofile_register_event(M4U_Event, "M4U ERROR");
	M4U_MMP_Events[M4U_MMP_CACHE_SYNC] = mmprofile_register_event(M4U_Event, "M4U_CACHE_SYNC");
	M4U_MMP_Events[M4U_MMP_TOGGLE_CG] = mmprofile_register_event(M4U_Event, "M4U_Toggle_CG");
	M4U_MMP_Events[M4U_MMP_TOGGLE_MVA_DBG] = mmprofile_register_event(M4U_Event, "MVA_DBG");

	/* enable events by default */
	mmprofile_enable_event(M4U_MMP_Events[M4U_MMP_ALLOC_MVA], 1);
	mmprofile_enable_event(M4U_MMP_Events[M4U_MMP_DEALLOC_MVA], 1);
	mmprofile_enable_event(M4U_MMP_Events[M4U_MMP_CONFIG_PORT], 1);
	mmprofile_enable_event(M4U_MMP_Events[M4U_MMP_M4U_ERROR], 1);
	mmprofile_enable_event(M4U_MMP_Events[M4U_MMP_CACHE_SYNC], 1);
	mmprofile_enable_event(M4U_MMP_Events[M4U_MMP_TOGGLE_MVA_DBG], 1);
	/* mmprofile_enable_event(M4U_MMP_Events[M4U_MMP_TOGGLE_CG], 0); */
	mmprofile_start(1);
}
#endif

/* get ref count on all pages in sgtable */
int m4u_get_sgtable_pages(struct sg_table *table)
{
	return 0;
}

/* put ref count on all pages in sgtable */
int m4u_put_sgtable_pages(struct sg_table *table)
{
	int i;
	struct scatterlist *sg;

	for_each_sg(table->sgl, sg, table->nents, i) {
		struct page *page = sg_page(sg);

		if (page) {
			if (!PageReserved(page))
				SetPageDirty(page);
			page_cache_release(page);
		}
	}
	return 0;
}

static struct m4u_buf_info_t *m4u_alloc_buf_info(void)
{
	struct m4u_buf_info_t *pList = NULL;

	pList = kzalloc(sizeof(struct m4u_buf_info_t), GFP_KERNEL);
	if (pList == NULL) {
		M4UMSG("m4u_client_add_buf(), pList=0x%p\n", pList);
		return NULL;
	}

	INIT_LIST_HEAD(&(pList->link));
	return pList;
}

static int m4u_free_buf_info(struct m4u_buf_info_t *pList)
{
	kfree(pList);
	return 0;
}

static int m4u_client_add_buf(m4u_client_t *client, struct m4u_buf_info_t *pList)
{
	mutex_lock(&(client->dataMutex));
	list_add(&(pList->link), &(client->mvaList));
	mutex_unlock(&(client->dataMutex));

	return 0;
}


/*static int m4u_client_del_buf(m4u_client_t *client, m4u_buf_info_t *pList)*/
/*{*/
/*    mutex_lock(&(client->dataMutex));*/
/*    list_del(&(pList->link));*/
/*    mutex_unlock(&(client->dataMutex));*/

/*    return 0;*/
/*}*/


/***********************************************************/
/** find or delete a buffer from client list
* @param   client   -- client to be searched
* @param   mva      -- mva to be searched
* @param   del      -- should we del this buffer from client?
*
* @return buffer_info if found, NULL on fail
* @remark
* @see
* @to-do    we need to add multi domain support here.
* @author K Zhang      @date 2013/11/14
************************************************************/
static struct m4u_buf_info_t *m4u_client_find_buf(m4u_client_t *client, unsigned int mva, int del)
{
	struct list_head *pListHead;
	struct m4u_buf_info_t *pList = NULL;
	struct m4u_buf_info_t *ret = NULL;

	if (client == NULL) {
		M4UERR("m4u_delete_from_garbage_list(), client is NULL!\n");
		m4u_dump_buf_info(NULL);
		return NULL;
	}

	mutex_lock(&(client->dataMutex));
	list_for_each(pListHead, &(client->mvaList)) {
		pList = container_of(pListHead, struct m4u_buf_info_t, link);
		if (pList->mva == mva)
			break;
	}
	if (pListHead == &(client->mvaList)) {
		ret = NULL;
	} else {
		if (del)
			list_del(pListHead);
		ret = pList;
	}

	mutex_unlock(&(client->dataMutex));

	return ret;
}


/*dump buf info in client*/
/*static void m4u_client_dump_buf(m4u_client_t *client, const char *pMsg)*/
/*{*/
/*    m4u_buf_info_t *pList;*/
/*    struct list_head *pListHead;*/

/*    M4UMSG("print mva list [%s] ================================>\n", pMsg);*/
/*    mutex_lock(&(client->dataMutex));*/
/*    list_for_each(pListHead, &(client->mvaList))*/
/*    {*/
/*	pList = container_of(pListHead, m4u_buf_info_t, link);*/
/*	M4UMSG("port=%s, va=0x%x, size=0x%x, mva=0x%x, prot=%d\n",*/
/*		m4u_get_port_name(pList->port), pList->va, pList->size, pList->mva, pList->prot);*/
/*    }*/
/*   mutex_unlock(&(client->dataMutex));*/

/*    M4UMSG("print mva list done ==========================>\n");*/
/*}*/


m4u_client_t *m4u_create_client(void)
{
	m4u_client_t *client;

	client = kmalloc(sizeof(m4u_client_t), GFP_ATOMIC);
	if (!client)
		return NULL;

	mutex_init(&(client->dataMutex));
	mutex_lock(&(client->dataMutex));
	client->open_pid = current->pid;
	client->open_tgid = current->tgid;
	INIT_LIST_HEAD(&(client->mvaList));
	mutex_unlock(&(client->dataMutex));

	return client;
}

int m4u_destroy_client(m4u_client_t *client)
{
	struct m4u_buf_info_t *pMvaInfo;
	unsigned int mva, size;
	M4U_PORT_ID port;

	while (1) {
		mutex_lock(&(client->dataMutex));
		if (list_empty(&client->mvaList)) {
			mutex_unlock(&(client->dataMutex));
			break;
		}
		pMvaInfo = container_of(client->mvaList.next, struct m4u_buf_info_t, link);
		M4UMSG("warnning: clean garbage at m4u close: module=%s,va=0x%lx,mva=0x%x,size=%d\n",
			m4u_get_port_name(pMvaInfo->port), pMvaInfo->va, pMvaInfo->mva,
			pMvaInfo->size);

		port = pMvaInfo->port;
		mva = pMvaInfo->mva;
		size = pMvaInfo->size;

		mutex_unlock(&(client->dataMutex));

		m4u_reclaim_notify(port, mva, size);

		/* m4u_dealloc_mva will lock client->dataMutex again */
		m4u_dealloc_mva(client, port, mva);
	}

	kfree(client);

	return 0;
}

static int m4u_dump_mmaps(unsigned long addr)
{
	struct vm_area_struct *vma;

	M4ULOG_MID("addr=0x%lx, name=%s, pid=0x%x,", addr, current->comm, current->pid);

	vma = find_vma(current->mm, addr);

	if (vma && (addr >= vma->vm_start)) {
		M4ULOG_MID("find vma: 0x%16lx-0x%16lx, flags=0x%lx\n",
			   (vma->vm_start), (vma->vm_end), vma->vm_flags);
		return 0;
	}

	M4UMSG("cannot find vma for addr 0x%lx\n", addr);
	return -1;
}

/* to-do: need modification to support 4G DRAM */
static phys_addr_t m4u_user_v2p(unsigned long va)
{
	unsigned long pageOffset = (va & (PAGE_SIZE - 1));
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;
	phys_addr_t pa;

	if (current == NULL) {
		M4UMSG("warning: m4u_user_v2p, current is NULL!\n");
		return 0;
	}
	if (current->mm == NULL) {
		M4UMSG("warning: m4u_user_v2p, current->mm is NULL! tgid=0x%x, name=%s\n",
		       current->tgid, current->comm);
		return 0;
	}

	pgd = pgd_offset(current->mm, va);	/* what is tsk->mm */
	if (pgd_none(*pgd) || pgd_bad(*pgd)) {
		M4UMSG("m4u_user_v2p(), va=0x%lx, pgd invalid!\n", va);
		return 0;
	}

	pud = pud_offset(pgd, va);
	if (pud_none(*pud) || pud_bad(*pud)) {
		M4UMSG("m4u_user_v2p(), va=0x%lx, pud invalid!\n", va);
		return 0;
	}

	pmd = pmd_offset(pud, va);
	if (pmd_none(*pmd) || pmd_bad(*pmd)) {
		M4UMSG("m4u_user_v2p(), va=0x%lx, pmd invalid!\n", va);
		return 0;
	}

	pte = pte_offset_map(pmd, va);
	if (pte_present(*pte)) {
		/* pa=(pte_val(*pte) & (PAGE_MASK)) | pageOffset; */
		pa = (pte_val(*pte) & (PHYS_MASK) & (~((phys_addr_t) 0xfff))) | pageOffset;
		pte_unmap(pte);
		return pa;
	}

	pte_unmap(pte);

	M4UMSG("m4u_user_v2p(), va=0x%lx, pte invalid!\n", va);
	return 0;
}


static int m4u_fill_sgtable_user(struct vm_area_struct *vma, unsigned long va, int page_num,
				 struct scatterlist **pSg, int has_page)
{
	unsigned long va_align;
	phys_addr_t pa = 0;
	int i, ret;
	struct scatterlist *sg = *pSg;
	struct page *pages;

	va_align = round_down(va, PAGE_SIZE);

	for (i = 0; i < page_num; i++) {
		int fault_cnt;
		unsigned long va_tmp = va_align+i*PAGE_SIZE;

		pa = 0;

		for (fault_cnt = 0; fault_cnt < 3000; fault_cnt++) {
			if (has_page) {
				ret = get_user_pages(current, current->mm, va_tmp, 1,
					(vma->vm_flags & VM_WRITE), 0, &pages, NULL);

				if (ret == 1)
					pa = page_to_phys(pages) | (va_tmp & ~PAGE_MASK);
			} else {
				pa = m4u_user_v2p(va_tmp);
				if (!pa) {
					handle_mm_fault(current->mm, vma, va_tmp,
						(vma->vm_flags&VM_WRITE) ? FAULT_FLAG_WRITE : 0);
				}
			}

			if (pa) {
				/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
				break;
			}
			cond_resched();
		}

		if (!pa || !sg) {
			struct vm_area_struct *vma_temp;

			M4UMSG("%s: fail va=0x%lx,page_num=0x%x,fail_va=0x%lx,pa=0x%lx,sg=0x%p,i=%d\n",
				__func__, va, page_num, va_tmp, (unsigned long)pa, sg, i);

			vma_temp = find_vma(current->mm, va_tmp);
			if (vma_temp != NULL) {
				M4UMSG("vm_start=0x%lx, vm_end=%lx, vm_flag= %lx\n",
				vma->vm_start, vma->vm_end, vma->vm_flags);
				M4UMSG("vma_temp_start=0x%lx, vma_temp_end=%lx, vm_temp_flag= %lx\n",
				vma_temp->vm_start, vma_temp->vm_end, vma_temp->vm_flags);
			}

			show_pte(current->mm, va_tmp);
			m4u_dump_mmaps(va);
			m4u_dump_mmaps(va_tmp);
			return -1;
		}

		if (fault_cnt > 2) {
			M4UINFO("warning: handle_mm_fault for %d times\n", fault_cnt);
			show_pte(current->mm, va_tmp);
			m4u_dump_mmaps(va_tmp);
		}
		/* debug check... */
		if ((pa & (PAGE_SIZE - 1)) != 0) {
			M4ULOG_MID("pa error, pa: 0x%lx, va: 0x%lx, align: 0x%lx\n",
				   (unsigned long)pa, va_tmp, va_align);
		}

		if (has_page) {
			struct page *page;

			page = phys_to_page(pa);
			/* M4UMSG("page=0x%x, pfn=%d\n", page, __phys_to_pfn(pa)); */
			sg_set_page(sg, page, PAGE_SIZE, 0);
			#ifdef CONFIG_NEED_SG_DMA_LENGTH
				sg->dma_length = sg->length;
			#endif
		} else {
			sg_dma_address(sg) = pa;
			sg_dma_len(sg) = PAGE_SIZE;
		}
		sg = sg_next(sg);
	}
	*pSg = sg;
	return 0;
}

static int m4u_create_sgtable_user(unsigned long va_align, struct sg_table *table)
{
	int ret = 0;
	struct vm_area_struct *vma;
	struct scatterlist *sg = table->sgl;
	unsigned int left_page_num = table->nents;
	unsigned long va = va_align;

	down_read(&current->mm->mmap_sem);

	while (left_page_num) {
		unsigned int vma_page_num;

		vma = find_vma(current->mm, va);
		if (vma == NULL || vma->vm_start > va) {
			M4UMSG("cannot find vma: va=0x%lx, vma=0x%p\n", va, vma);
			if (vma != NULL) {
				M4UMSG("vm_start=0x%lx, vm_end=0x%lx, vm_flag= 0x%lx\n",
				vma->vm_start, vma->vm_end, vma->vm_flags);
			}
			m4u_dump_mmaps(va);
			ret = -1;
			goto out;
		} else {
			/* M4ULOG_MID("%s va: 0x%lx, vma->vm_start=0x%lx, vma->vm_end=0x%lx\n",*/
			/*__func__, va, vma->vm_start, vma->vm_end); */
		}

		vma_page_num = (vma->vm_end - va) / PAGE_SIZE;
		vma_page_num = min(vma_page_num, left_page_num);

		if ((vma->vm_flags) & VM_PFNMAP) {
			/* ion va or ioremap vma has this flag */
			/* VM_PFNMAP: Page-ranges managed without "struct page", just pure PFN */
			ret = m4u_fill_sgtable_user(vma, va, vma_page_num, &sg, 0);
			M4ULOG_MID("alloc_mva VM_PFNMAP va=0x%lx, page_num=0x%x\n", va,
				   vma_page_num);
		} else {
			/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
			ret = m4u_fill_sgtable_user(vma, va, vma_page_num, &sg, 1);
			if (-1 == ret) {
				struct vm_area_struct *vma_temp;

				vma_temp = find_vma(current->mm, va_align);
				if (!vma_temp) {
					M4UMSG("%s cannot find vma\n", __func__);
					return -1;
				}
				M4UMSG("m4u_create_sgtable_user: vm_start=0x%lx, vm_end=0x%lx, vm_flag= 0x%lx\n",
				vma_temp->vm_start, vma_temp->vm_end, vma_temp->vm_flags);
			}
		}
		if (ret) {
			/* Add one line comment for avoid kernel coding style, WARNING:BRACES: */
			goto out;
		}

		left_page_num -= vma_page_num;
		va += vma_page_num * PAGE_SIZE;
	}

out:
	up_read(&current->mm->mmap_sem);
	return ret;
}

/* make a sgtable for virtual buffer */
struct sg_table *m4u_create_sgtable(unsigned long va, unsigned int size)
{
	struct sg_table *table;
	int ret, i, page_num;
	unsigned long va_align;
	phys_addr_t pa;
	struct scatterlist *sg;
	struct page *page;

	page_num = M4U_GET_PAGE_NUM(va, size);
	va_align = round_down(va, PAGE_SIZE);

	table = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!table) {
		M4UMSG("%s table kmalloc fail: va=0x%lx, size=0x%x, page_num=%d\n",
				__func__, va, size, page_num);
		return ERR_PTR(-ENOMEM);
	}

	ret = sg_alloc_table(table, page_num, GFP_KERNEL);
	if (ret) {
		kfree(table);
		M4UMSG("%s alloc_sgtable fail: va=0x%lx, size=0x%x, page_num=%d\n",
				__func__, va, size, page_num);
		return ERR_PTR(-ENOMEM);
	}

	M4ULOG_LOW("%s va=0x%lx, PAGE_OFFSET=0x%lx, VMALLOC_START=0x%lx, VMALLOC_END=0x%lx\n",
		   __func__, va, PAGE_OFFSET, VMALLOC_START, VMALLOC_END);

	if (va < PAGE_OFFSET) {	/* from user space */
		if (va >= VMALLOC_START && va <= VMALLOC_END) {	/* vmalloc */
			M4ULOG_MID(" from user space vmalloc, va = 0x%lx", va);
			for_each_sg(table->sgl, sg, table->nents, i) {
				page = vmalloc_to_page((void *)(va_align + i * PAGE_SIZE));
				if (!page) {
					M4UMSG("vmalloc_to_page fail, va=0x%lx\n",
					       va_align + i * PAGE_SIZE);
					goto err;
				}
				sg_set_page(sg, page, PAGE_SIZE, 0);
			}
		} else {
			ret = m4u_create_sgtable_user(va_align, table);
			if (ret) {
				M4UMSG("%s error va=0x%lx, size=%d\n", __func__, va, size);
				goto err;
			}
		}
	} else {		/* from kernel space */
		if (va >= VMALLOC_START && va <= VMALLOC_END) {	/* vmalloc */
			M4ULOG_MID(" from kernel space vmalloc, va = 0x%lx", va);
			for_each_sg(table->sgl, sg, table->nents, i) {
				page = vmalloc_to_page((void *)(va_align + i * PAGE_SIZE));
				if (!page) {
					M4UMSG("vmalloc_to_page fail, va=0x%lx\n",
					       va_align + i * PAGE_SIZE);
					goto err;
				}
				sg_set_page(sg, page, PAGE_SIZE, 0);
			}
		} else {	/* kmalloc to-do: use one entry sgtable. */
			for_each_sg(table->sgl, sg, table->nents, i) {
				pa = virt_to_phys((void *)(va_align + i * PAGE_SIZE));
				page = phys_to_page(pa);
				sg_set_page(sg, page, PAGE_SIZE, 0);
			}
		}
	}

	return table;

err:
	sg_free_table(table);
	kfree(table);
	return ERR_PTR(-EFAULT);
}

int m4u_destroy_sgtable(struct sg_table *table)
{
	if (!IS_ERR_OR_NULL(table)) {
		sg_free_table(table);
		kfree(table);
	}
	return 0;
}

/* #define __M4U_MAP_MVA_TO_KERNEL_FOR_DEBUG__ */

int m4u_alloc_mva(m4u_client_t *client, M4U_PORT_ID port,
		  unsigned long va, struct sg_table *sg_table,
		  unsigned int size, unsigned int prot,
		  unsigned int flags, unsigned int *pMva)
{
	int ret;
	struct m4u_buf_info_t *pMvaInfo;
	unsigned int mva = 0, mva_align, size_align;
	int larb_id = m4u_port_2_larb_id(port);

	if (larb_id == -1)
		return -EFAULT;
#ifdef M4U_PROFILE
	mmprofile_log_ex(M4U_MMP_Events[M4U_MMP_ALLOC_MVA], MMPROFILE_FLAG_START, va, size);
#endif

	if (va && sg_table) {
		M4ULOG_MID("%s, va or sg_table are both valid: va=0x%lx, sg=0x%p\n", __func__,
		       va, sg_table);
	}

	if (!va && !sg_table)	{
		M4UMSG("%s, va or sg_table are both invalid: va=0x%lx, sg=0x%p\n", __func__, va, sg_table);
		ret = -EFAULT;
		goto err;
	}

	if (va && ((flags & M4U_FLAGS_SG_READY) == 0)) {
		sg_table = m4u_create_sgtable(va, size);
		if (IS_ERR_OR_NULL(sg_table)) {
			M4UMSG("%s, cannot create sg: larb=%d,module=%s,va=0x%lx,sg=0x%p,size=%d,prot=0x%x,flags=0x%x\n"
				, __func__, larb_id, m4u_get_port_name(port),
				va, sg_table, size, prot, flags);
			ret = -EFAULT;
			goto err;
		}
	}

	/* here we get correct sg_table for this buffer */

	pMvaInfo = m4u_alloc_buf_info();
	if (!pMvaInfo) {
		ret = -ENOMEM;
		goto err;
	}

	pMvaInfo->va = va;
	pMvaInfo->port = port;
	pMvaInfo->size = size;
	pMvaInfo->prot = prot;
	pMvaInfo->flags = flags;
	pMvaInfo->sg_table = sg_table;

	if (flags & M4U_FLAGS_FIX_MVA)
		mva = m4u_do_mva_alloc_fix(va, *pMva, size, pMvaInfo);
	else if (flags & M4U_FLAGS_START_FROM)
		mva = m4u_do_mva_alloc_start_from(va, *pMva, size, pMvaInfo);
	else
		mva = m4u_do_mva_alloc(va, size, pMvaInfo);

	if (mva == 0) {
		m4u_aee_print("alloc mva fail: larb=%d,module=%s,size=%d\n",
				larb_id, m4u_get_port_name(port), size);
		m4u_dump_buf_info(NULL);
		ret = -EINVAL;
		goto err1;
	} else
		M4ULOG_LOW("%s,mva = 0x%x\n", __func__, mva);

	m4u_get_sgtable_pages(sg_table);

	mva_align = round_down(mva, PAGE_SIZE);
	size_align = PAGE_ALIGN(mva + size - mva_align);

	ret = m4u_map_sgtable(m4u_get_domain_by_port(port), mva_align, sg_table,
			size_align, pMvaInfo->prot);
	if (ret < 0) {
		M4UMSG("error to map sgtable\n");
		goto err2;
	}

	pMvaInfo->mva = mva;
	pMvaInfo->mva_align = mva_align;
	pMvaInfo->size_align = size_align;
	*pMva = mva;

	if (flags & M4U_FLAGS_SEQ_ACCESS)
		pMvaInfo->seq_id = m4u_insert_seq_range(port, mva, mva + size - 1);

	m4u_client_add_buf(client, pMvaInfo);

	M4ULOG_MID("%s: pMvaInfo=0x%p, larb=%d,module=%s,va=0x%lx,sg=0x%p,size=%d,prot=0x%x,flags=0x%x,mva=0x%x\n",
		__func__, pMvaInfo, larb_id, m4u_get_port_name(port), va, sg_table,
		size, prot, flags, mva);

#ifdef M4U_PROFILE
	mmprofile_log_ex(M4U_MMP_Events[M4U_MMP_ALLOC_MVA], MMPROFILE_FLAG_END, port, mva);
#endif

#ifdef __M4U_MAP_MVA_TO_KERNEL_FOR_DEBUG__
	/* map this mva to kernel va just for debug */
	{
		unsigned long kernel_va;
		unsigned int kernel_size;
		int ret;

		ret = m4u_mva_map_kernel(mva, size, &kernel_va, &kernel_size);
		if (ret)
			M4UMSG("error to map kernel va: mva=0x%x, size=%d\n", mva, size);
		else {
			pMvaInfo->mapped_kernel_va_for_debug = kernel_va;
			M4ULOG_MID("[kernel_va_debug] map va: mva=0x%x, kernel_va=0x%lx, size=0x%x\n",
				mva, kernel_va, size);
		}
	}
#endif

	return 0;

err2:
	m4u_do_mva_free(mva, size);

err1:
	m4u_free_buf_info(pMvaInfo);

err:
	if (va)
		m4u_destroy_sgtable(sg_table);

	*pMva = 0;

	M4UMSG("error: larb=%d,module=%s,va=0x%lx,size=%d,prot=0x%x,flags=0x%x, mva=0x%x\n",
		larb_id, m4u_get_port_name(port), va, size, prot, flags, mva);

#ifdef M4U_PROFILE
	mmprofile_log_ex(M4U_MMP_Events[M4U_MMP_ALLOC_MVA], MMPROFILE_FLAG_END, port, 0);
#endif

	return ret;
}

/* interface for ion */
static m4u_client_t *ion_m4u_client;
int m4u_alloc_mva_sg(port_mva_info_t *port_info,
				struct sg_table *sg_table)
{
	int prot;
	int ret;
	unsigned int flags = 0;

	if (!ion_m4u_client) {
		ion_m4u_client = m4u_create_client();
		if (IS_ERR_OR_NULL(ion_m4u_client)) {
			ion_m4u_client = NULL;
			return -1;
		}
	}

	prot = M4U_PROT_READ | M4U_PROT_WRITE
	    | (port_info->cache_coherent ? (M4U_PROT_SHARE | M4U_PROT_CACHE) : 0)
	    | (port_info->security ? M4U_PROT_SEC : 0);

	if (port_info->flags & M4U_FLAGS_FIX_MVA) {
		if (port_info->iova_end > port_info->iova_start + port_info->BufSize) {
			port_info->mva = port_info->iova_start;
			flags = M4U_FLAGS_START_FROM;
		} else
			flags = M4U_FLAGS_FIX_MVA;
	}
	if (port_info->flags & M4U_FLAGS_SG_READY)
		flags |= M4U_FLAGS_SG_READY;
	else
		port_info->va = 0;
	ret = m4u_alloc_mva(ion_m4u_client, port_info->eModuleID, port_info->va, sg_table,
		port_info->BufSize, prot, flags, &port_info->mva);
	return ret;
}

int m4u_dealloc_mva_sg(int eModuleID, struct sg_table *sg_table,
			const unsigned int BufSize, const unsigned int MVA)
{
	if (!ion_m4u_client) {
		m4u_aee_print("ion_m4u_client==NULL !! oops oops~~~~\n");
		return -1;
	}

	return m4u_dealloc_mva(ion_m4u_client, eModuleID, MVA);
}

/* should not hold client->dataMutex here. */
int m4u_dealloc_mva(m4u_client_t *client, M4U_PORT_ID port, unsigned int mva)
{
	struct m4u_buf_info_t *pMvaInfo;
	int ret, is_err = 0;
	unsigned int size;
	int larb_id = m4u_port_2_larb_id(port);

	if (larb_id == -1)
		return -EFAULT;
#ifdef M4U_PROFILE
	mmprofile_log_ex(M4U_MMP_Events[M4U_MMP_DEALLOC_MVA], MMPROFILE_FLAG_START, port, mva);
#endif

	pMvaInfo = m4u_client_find_buf(client, mva, 1);
	if (!pMvaInfo) {
		ret = -ENOMEM;
		return ret;
	}
	if (unlikely(!pMvaInfo)) {
		M4UMSG("error: m4u_dealloc_mva no mva found in client! module=%s, mva=0x%x\n",
		       m4u_get_port_name(port), mva);
		m4u_dump_buf_info(NULL);
#ifdef M4U_PROFILE
		mmprofile_log_ex(M4U_MMP_Events[M4U_MMP_DEALLOC_MVA], MMPROFILE_FLAG_START, 0x5a5a5a5a, mva);
#endif
		return -EINVAL;
	}

	pMvaInfo->flags |= M4U_FLAGS_MVA_IN_FREE;

	M4ULOG_MID("m4u_dealloc_mva: larb=%d,module=%s,mva=0x%x, size=%d\n",
		   larb_id, m4u_get_port_name(port), mva, pMvaInfo->size);

	ret = m4u_unmap(m4u_get_domain_by_port(port), pMvaInfo->mva_align, pMvaInfo->size_align);
	if (ret) {
		is_err = 1;
		M4UMSG("m4u_unmap fail\n");
	}

	if (pMvaInfo->va != 0) {
		/* non ion buffer*/
		if (pMvaInfo->va < PAGE_OFFSET) {  /* from user space */
			if (!(pMvaInfo->va >= VMALLOC_START && pMvaInfo->va <= VMALLOC_END)) { /* non vmalloc	 */
				m4u_put_sgtable_pages(pMvaInfo->sg_table);
			}
		}
	}

	ret = m4u_do_mva_free(mva, pMvaInfo->size);
	if (ret) {
		is_err = 1;
		M4UMSG("do_mva_free fail\n");
	}

	if (pMvaInfo->va) {	/* buffer is allocated by va */
		m4u_destroy_sgtable(pMvaInfo->sg_table);
	}

	if (pMvaInfo->flags & M4U_FLAGS_SEQ_ACCESS) {
		if (pMvaInfo->seq_id > 0)
			m4u_invalid_seq_range_by_id(port, pMvaInfo->seq_id);
	}

	if (is_err) {
		m4u_aee_print("%s fail: port=%s, mva=0x%x, size=0x%x, va=0x%lx\n", __func__,
			      m4u_get_port_name(port), mva, pMvaInfo->size, pMvaInfo->va);
		ret = -EINVAL;
	} else
		ret = 0;

	size = pMvaInfo->size;

#ifdef __M4U_MAP_MVA_TO_KERNEL_FOR_DEBUG__
	/* unmap kernel va for debug */
	{
		if (pMvaInfo->mapped_kernel_va_for_debug) {
			M4ULOG_MID("[kernel_va_debug] unmap va: mva=0x%x, kernel_va=0x%lx, size=0x%x\n",
				    pMvaInfo->mva, pMvaInfo->mapped_kernel_va_for_debug, pMvaInfo->size);
				    m4u_mva_unmap_kernel(pMvaInfo->mva, pMvaInfo->size,
				    pMvaInfo->mapped_kernel_va_for_debug);
		}
	}
#endif

	m4u_free_buf_info(pMvaInfo);

#ifdef M4U_PROFILE
	mmprofile_log_ex(M4U_MMP_Events[M4U_MMP_DEALLOC_MVA], MMPROFILE_FLAG_END, size, mva);
#endif

	return ret;
}

int m4u_dma_cache_flush_all(void)
{
	smp_inner_dcache_flush_all();
	outer_flush_all();
	return 0;
}

void m4u_dma_cache_flush_range(void *start, size_t size)
{
#ifndef CONFIG_MTK_CACHE_FLUSH_RANGE_PARALLEL
	dmac_flush_range((void *)start, (void *)(start + size));
#else
	mt_smp_cache_flush_m4u(start, size);
#endif
}

static struct vm_struct *cache_map_vm_struct;
static int m4u_cache_sync_init(void)
{
	cache_map_vm_struct = get_vm_area(PAGE_SIZE, VM_ALLOC);
	if (!cache_map_vm_struct)
		return -ENOMEM;

	return 0;
}

static void *m4u_cache_map_page_va(struct page *page)
{
	int ret;
	struct page **ppPage = &page;

	ret = map_vm_area(cache_map_vm_struct, PAGE_KERNEL, ppPage);
	if (ret) {
		M4UMSG("error to map page\n");
		return NULL;
	}
	return cache_map_vm_struct->addr;
}

static void m4u_cache_unmap_page_va(unsigned int va)
{
	unmap_kernel_range((unsigned long)cache_map_vm_struct->addr, PAGE_SIZE);
}


static int __m4u_cache_sync_kernel(const void *start, size_t size, M4U_CACHE_SYNC_ENUM sync_type)
{
	if (sync_type == M4U_CACHE_CLEAN_BY_RANGE)
		dmac_map_area((void *)start, size, DMA_TO_DEVICE);
	else if (sync_type == M4U_CACHE_INVALID_BY_RANGE)
		dmac_unmap_area((void *)start, size, DMA_FROM_DEVICE);
	else if (sync_type == M4U_CACHE_FLUSH_BY_RANGE)
#ifndef CONFIG_MTK_CACHE_FLUSH_RANGE_PARALLEL
		dmac_flush_range((void *)start, (void *)(start + size));
#else
		mt_smp_cache_flush_m4u(start, size);
#endif

	return 0;
}

static struct page *m4u_cache_get_page(unsigned long va)
{
	unsigned long start;
	phys_addr_t pa;
	struct page *page;

	start = va & (~M4U_PAGE_MASK);
	pa = m4u_user_v2p(start);
	if (pa == 0) {
		M4UMSG("error m4u_get_phys user_v2p return 0 on va=0x%lx\n", start);
		/* dump_page(page); */
		m4u_dump_mmaps(start);
		show_pte(current->mm, va);
		return NULL;
	}
	page = phys_to_page(pa);

	return page;
}

/* lock to protect cache_map_vm_struct */
static DEFINE_MUTEX(gM4u_cache_sync_user_lock);

static int __m4u_cache_sync_user(unsigned long start, size_t size, M4U_CACHE_SYNC_ENUM sync_type)
{
	unsigned long map_size, map_start, map_end;
	unsigned long end = start + size;
	struct page *page;
	unsigned long map_va, map_va_align;
	int ret = 0;

	mutex_lock(&gM4u_cache_sync_user_lock);

	if (!cache_map_vm_struct) {
		M4UMSG(" error: cache_map_vm_struct is NULL, retry\n");
		m4u_cache_sync_init();
	}
	if (!cache_map_vm_struct) {
		M4UMSG("error: cache_map_vm_struct is NULL, no vmalloc area\n");
		ret = -1;
		goto out;
	}

	map_start = start;
	while (map_start < end) {
		map_end = min(((map_start & (~M4U_PAGE_MASK)) + PAGE_SIZE), end);
		map_size = map_end - map_start;

		page = m4u_cache_get_page(map_start);
		if (!page) {
			ret = -1;
			goto out;
		}

		map_va = (unsigned long)m4u_cache_map_page_va(page);
		if (!map_va) {
			ret = -1;
			goto out;
		}

		map_va_align = map_va | (map_start & (PAGE_SIZE - 1));

		__m4u_cache_sync_kernel((void *)map_va_align, map_size, sync_type);

		m4u_cache_unmap_page_va(map_va);
		map_start = map_end;
	}

out:
	mutex_unlock(&gM4u_cache_sync_user_lock);

	return ret;
}

int m4u_cache_sync_by_range(unsigned long va, unsigned int size,
			    M4U_CACHE_SYNC_ENUM sync_type, struct sg_table *table)
{
	int ret = 0;

	if (va < PAGE_OFFSET) {	/* from user space */
		ret = __m4u_cache_sync_user(va, size, sync_type);
	} else {
		ret = __m4u_cache_sync_kernel((void *)va, size, sync_type);
	}

#ifdef CONFIG_OUTER_CACHE
	{
		struct scatterlist *sg;
		int i;

		for_each_sg(table->sgl, sg, table->nents, i) {
			unsigned int len = sg_dma_len(sg);
			phys_addr_t phys_addr = get_sg_phys(sg);

			if (sync_type == M4U_CACHE_CLEAN_BY_RANGE)
				outer_clean_range(phys_addr, phys_addr + len);
			else if (sync_type == M4U_CACHE_INVALID_BY_RANGE)
				outer_inv_range(phys_addr, phys_addr + len);
			else if (sync_type == M4U_CACHE_FLUSH_BY_RANGE)
				outer_flush_range(phys_addr, phys_addr + len);
		}
	}
#endif

	return ret;
}


/*    notes: only mva allocated by m4u_alloc_mva can use this function.*/
/*	if buffer is allocated by ion, please use ion_cache_sync*/

int m4u_cache_sync(m4u_client_t *client, M4U_PORT_ID port,
		   unsigned long va, unsigned int size, unsigned int mva,
		   M4U_CACHE_SYNC_ENUM sync_type)
{
	int ret = 0;

#ifdef M4U_PROFILE
	mmprofile_log_ex(M4U_MMP_Events[M4U_MMP_CACHE_SYNC], MMPROFILE_FLAG_START, va, mva);
	mmprofile_log_ex(M4U_MMP_Events[M4U_MMP_CACHE_SYNC], MMPROFILE_FLAG_PULSE, size, ((sync_type)<<24) | port);
#endif
	M4ULOG_MID("cache_sync port=%s, va=0x%lx, size=0x%x, mva=0x%x, type=%d\n",
		   m4u_get_port_name(port), va, size, mva, sync_type);

	if (sync_type < M4U_CACHE_CLEAN_ALL) {
		struct m4u_buf_info_t *pMvaInfo = NULL;

		if (client)
			pMvaInfo = m4u_client_find_buf(client, mva, 0);

		/* some user may sync mva from other client (eg. ovl may not know*/
		/* who allocated this buffer, but he need to sync cache). */
		/* we make a workaround here by query mva from mva manager */
		if (!pMvaInfo)
			pMvaInfo = mva_get_priv(mva);

		if (!pMvaInfo) {
			M4UMSG("cache sync fail, cannot find buf: mva=0x%x, client=0x%p\n", mva,
			       client);
			m4u_dump_buf_info(NULL);
#ifdef M4U_PROFILE
			mmprofile_log_ex(M4U_MMP_Events[M4U_MMP_CACHE_SYNC], MMPROFILE_FLAG_END, 0, 0);
#endif
			return -1;
		}

		if ((pMvaInfo->size != size) || (pMvaInfo->va != va)) {
			M4UMSG("cache_sync fail: expect mva=0x%x,size=0x%x,va=0x%lx, but mva=0x%x,size=0x%x,va=0x%lx\n",
			       pMvaInfo->mva, pMvaInfo->size, pMvaInfo->va, mva, size, va);
#ifdef M4U_PROFILE
			mmprofile_log_ex(M4U_MMP_Events[M4U_MMP_CACHE_SYNC], MMPROFILE_FLAG_END,
				       pMvaInfo->va, pMvaInfo->mva);
#endif
			return -1;
		}

		if ((va | size) & (L1_CACHE_BYTES - 1)) {	/* va size should be cache line align */
			M4UMSG("warning: cache_sync not align: va=0x%lx,size=0x%x,align=0x%x\n",
			       va, size, L1_CACHE_BYTES);
		}

		ret = m4u_cache_sync_by_range(va, size, sync_type, pMvaInfo->sg_table);
	} else {
		/* All cache operation */
		if (sync_type == M4U_CACHE_CLEAN_ALL) {
			smp_inner_dcache_flush_all();
			outer_clean_all();
		} else if (sync_type == M4U_CACHE_INVALID_ALL) {
			M4UMSG("no one can use invalid all!\n");
			return -1;
		} else if (sync_type == M4U_CACHE_FLUSH_ALL) {
			smp_inner_dcache_flush_all();
			outer_flush_all();
		}
	}
#ifdef M4U_PROFILE
	mmprofile_log_ex(M4U_MMP_Events[M4U_MMP_CACHE_SYNC], MMPROFILE_FLAG_END, size, mva);
#endif
	return ret;
}

void m4u_dma_map_area(void *start, size_t size, M4U_DMA_DIR dir)
{
	if (dir == M4U_DMA_FROM_DEVICE)
		dmac_map_area(start, size, DMA_FROM_DEVICE);
	else if (dir == M4U_DMA_TO_DEVICE)
		dmac_map_area(start, size, DMA_TO_DEVICE);
	else if (dir == M4U_DMA_BIDIRECTIONAL)
		dmac_map_area(start, size, DMA_BIDIRECTIONAL);
}

void m4u_dma_unmap_area(void *start, size_t size, M4U_DMA_DIR dir)
{
	if (dir == M4U_DMA_FROM_DEVICE)
		dmac_unmap_area(start, size, DMA_FROM_DEVICE);
	else if (dir == M4U_DMA_TO_DEVICE)
		dmac_unmap_area(start, size, DMA_TO_DEVICE);
	else if (dir == M4U_DMA_BIDIRECTIONAL)
		dmac_unmap_area(start, size, DMA_BIDIRECTIONAL);
}

long m4u_dma_op(m4u_client_t *client, M4U_PORT_ID port,
		unsigned long va, unsigned int size, unsigned int mva,
		M4U_DMA_TYPE dma_type, M4U_DMA_DIR dma_dir)
{
	struct scatterlist *sg;
	int i, j;
	struct sg_table *table = NULL;
	int npages = 0;
	unsigned long start = -1;

	struct m4u_buf_info_t *pMvaInfo = NULL;

	if (client)
		pMvaInfo = m4u_client_find_buf(client, mva, 0);

	/* some user may sync mva from other client*/
	/*(eg. ovl may not know who allocated this buffer,*/
	/*but he need to sync cache).*/
	/*we make a workaround here by query mva from mva manager */
	if (!pMvaInfo)
		pMvaInfo = mva_get_priv(mva);

	if (!pMvaInfo) {
		M4UMSG("m4u dma fail, cannot find buf: mva=0x%x, client=0x%p.\n", mva, client);
		return -1;
	}

	if ((pMvaInfo->size != size) || (pMvaInfo->va != va)) {
		M4UMSG("m4u dma fail: expect mva=0x%x,size=0x%x,va=0x%lx, but mva=0x%x,size=0x%x,va=0x%lx\n",
			pMvaInfo->mva, pMvaInfo->size, pMvaInfo->va, mva, size, va);
		return -1;
	}

	if ((va|size) & (L1_CACHE_BYTES-1)) /* va size should be cache line align */
		M4UMSG("warning: cache_sync not align: va=0x%lx,size=0x%x,align=0x%x\n",
			va, size, L1_CACHE_BYTES);

	table = pMvaInfo->sg_table;
	/* npages = PAGE_ALIGN(size) / PAGE_SIZE; */
	npages = M4U_GET_PAGE_NUM(va, size);

	mutex_lock(&gM4u_cache_sync_user_lock);

	if (!cache_map_vm_struct) {
		M4UMSG(" error: cache_map_vm_struct is NULL, retry\n");
		m4u_cache_sync_init();
	}

	if (!cache_map_vm_struct) {
		M4UMSG("error: cache_map_vm_struct is NULL, no vmalloc area\n");
		mutex_unlock(&gM4u_cache_sync_user_lock);
		return -ENOMEM;
	}

	for_each_sg(table->sgl, sg, table->nents, i) {
		int npages_this_entry = PAGE_ALIGN(sg_dma_len(sg)) / PAGE_SIZE;
		struct page *page = sg_page(sg);

		if (!page) {
			phys_addr_t pa = sg_dma_address(sg);

			if (!pa) {
				M4UMSG("m4u_dma_op fail, VM_PFNMAP, no page.\n");
				return -EFAULT;
			}
			page = phys_to_page(pa);
			if (!pfn_valid(page_to_pfn(page))) {
				M4UMSG("m4u_dma_op fail, VM_PFNMAP, no page, va = 0x%lx, size = 0x%x, npages = 0x%x.\n",
					va, size, npages);
				return -EFAULT;
			}
		}

		if (i >= npages)
			M4UERR("sg table is over pages number, i=%d, npages=0x%x\n", i, npages);

		for (j = 0; j < npages_this_entry; j++) {
			start = (unsigned long) m4u_cache_map_page_va(page++);

			if (IS_ERR_OR_NULL((void *) start)) {
				M4UMSG("cannot do cache sync: ret=%lu\n", start);
				mutex_unlock(&gM4u_cache_sync_user_lock);
				return -EFAULT;
			}

			if (dma_type == M4U_DMA_MAP_AREA)
				m4u_dma_map_area((void *)start, PAGE_SIZE, dma_dir);
			else if (dma_type == M4U_DMA_UNMAP_AREA)
				m4u_dma_unmap_area((void *)start, PAGE_SIZE, dma_dir);
			else if (dma_type == M4U_DMA_FLUSH_BY_RANGE)
				m4u_dma_cache_flush_range((void *)start, PAGE_SIZE);

			m4u_cache_unmap_page_va(start);
		}
	}

	mutex_unlock(&gM4u_cache_sync_user_lock);

	return 0;
}

int m4u_dump_info(int m4u_index)
{
	return 0;
}

void m4u_get_pgd(m4u_client_t *client, M4U_PORT_ID port, void **pgd_va, void **pgd_pa,
		 unsigned int *size)
{
	struct m4u_domain *pDomain;

	pDomain = m4u_get_domain_by_port(port);
	*pgd_va = pDomain->pgd;
	*pgd_pa = (void *)(uintptr_t)pDomain->pgd_pa;
	*size = M4U_PGD_SIZE;
}

unsigned long m4u_mva_to_pa(m4u_client_t *client, M4U_PORT_ID port, unsigned int mva)
{
	unsigned long pa;
	struct m4u_domain *pDomain;

	pDomain = m4u_get_domain_by_port(port);

	pa = m4u_get_pte(pDomain, mva);

	return pa;
}

int m4u_query_mva_info(unsigned int mva, unsigned int size, unsigned int *real_mva,
		       unsigned int *real_size)
{
	struct m4u_buf_info_t *pMvaInfo;

	if ((!real_mva) || (!real_size))
		return -1;

	pMvaInfo = mva_get_priv_ext(mva);
	if (!pMvaInfo) {
		M4UMSG("%s cannot find mva: mva=0x%x, size=0x%x\n", __func__, mva, size);
		*real_mva = 0;
		*real_size = 0;

		return -2;
	}
	*real_mva = pMvaInfo->mva;
	*real_size = pMvaInfo->size;

	return 0;
}
EXPORT_SYMBOL(m4u_query_mva_info);

/***********************************************************/
/** map mva buffer to kernel va buffer
*   this function should ONLY used for DEBUG
************************************************************/
int m4u_mva_map_kernel(unsigned int mva, unsigned int size, unsigned long *map_va,
		       unsigned int *map_size)
{
	struct m4u_buf_info_t *pMvaInfo;
	struct sg_table *table;
	struct scatterlist *sg;
	int i, j, k, ret = 0;
	struct page **pages;
	unsigned int page_num;
	void *kernel_va;
	unsigned int kernel_size;

	pMvaInfo = mva_get_priv(mva);

	if (!pMvaInfo || pMvaInfo->size < size) {
		M4UMSG("%s cannot find mva: mva=0x%x, size=0x%x\n", __func__, mva, size);
		if (pMvaInfo)
			M4UMSG("pMvaInfo: mva=0x%x, size=0x%x\n", pMvaInfo->mva, pMvaInfo->size);
		return -1;
	}

	table = pMvaInfo->sg_table;

	page_num = M4U_GET_PAGE_NUM(mva, size);
	pages = vmalloc(sizeof(struct page *) * page_num);
	if (pages == NULL) {
		M4UMSG("mva_map_kernel:error to vmalloc for %d\n",
		       (unsigned int)sizeof(struct page *) * page_num);
		return -1;
	}

	k = 0;
	for_each_sg(table->sgl, sg, table->nents, i) {
		struct page *page_start;
		int pages_in_this_sg = PAGE_ALIGN(sg_dma_len(sg)) / PAGE_SIZE;
#ifdef CONFIG_NEED_SG_DMA_LENGTH
		if (sg_dma_address(sg) == 0)
			pages_in_this_sg = PAGE_ALIGN(sg->length) / PAGE_SIZE;
#endif
		page_start = sg_page(sg);
		for (j = 0; j < pages_in_this_sg; j++) {
			pages[k++] = page_start++;
			if (k >= page_num)
				goto get_pages_done;
		}
	}

get_pages_done:
	if (k < page_num) {
		/* this should not happen, because we have checked the size before. */
		M4UMSG("mva_map_kernel:only get %d pages: mva=0x%x, size=0x%x, pg_num=%d\n", k, mva,
		       size, page_num);
		ret = -1;
		goto error_out;
	}

	kernel_va = 0;
	kernel_size = 0;
	kernel_va = vmap(pages, page_num, VM_MAP, PAGE_KERNEL);
	if (kernel_va == 0 || (unsigned long)kernel_va & M4U_PAGE_MASK) {
		M4UMSG("mva_map_kernel:vmap fail: page_num=%d, kernel_va=0x%p\n", page_num,
		       kernel_va);
		ret = -2;
		goto error_out;
	}

	kernel_va += ((unsigned long)mva & (M4U_PAGE_MASK));

	*map_va = (unsigned long)kernel_va;
	*map_size = size;

error_out:
	vfree(pages);
	M4ULOG_LOW("mva_map_kernel:mva=0x%x,size=0x%x,map_va=0x%lx,map_size=0x%x\n",
		   mva, size, *map_va, *map_size);

	return ret;
}
EXPORT_SYMBOL(m4u_mva_map_kernel);

int m4u_mva_unmap_kernel(unsigned int mva, unsigned int size, unsigned long map_va)
{
	M4ULOG_LOW("mva_unmap_kernel:mva=0x%x,size=0x%x,va=0x%lx\n", mva, size, map_va);
	vunmap((void *)(map_va & (~M4U_PAGE_MASK)));
	return 0;
}
EXPORT_SYMBOL(m4u_mva_unmap_kernel);

static int MTK_M4U_open(struct inode *inode, struct file *file)
{
	m4u_client_t *client;

	client = m4u_create_client();
	if (IS_ERR_OR_NULL(client)) {
		M4UMSG("createclientfail\n");
		return -ENOMEM;
	}

	file->private_data = client;

	return 0;
}

static int MTK_M4U_release(struct inode *inode, struct file *file)
{
	m4u_client_t *client = file->private_data;

	m4u_destroy_client(client);
	return 0;
}

static int MTK_M4U_flush(struct file *filp, fl_owner_t a_id)
{
	return 0;
}

#ifdef M4U_TEE_SERVICE_ENABLE

bool m4u_tee_en;
KREE_SESSION_HANDLE m4u_session;

static DEFINE_MUTEX(gM4u_port_tee);

static int m4u_session_init(void)
{
	TZ_RESULT ret;

	ret = KREE_CreateSession(TZ_TA_M4U_UUID, &m4u_session);
	if (ret != TZ_RESULT_SUCCESS) {
		M4UMSG("m4u CreateSession error %d\n", ret);
		return -1;
	}
	M4UINFO("create session: 0x%x\n", m4u_session);
	m4u_tee_en = true;
	return 0;
}

static int m4u_sec_init(unsigned int u4NonSecPa, unsigned int L2_enable, unsigned int *secsize)
{
	union MTEEC_PARAM param[4];
	uint32_t paramTypes;
	TZ_RESULT ret;

	param[0].value.a = u4NonSecPa;
	param[0].value.b = L2_enable;
	param[1].value.a = 1;
	paramTypes = TZ_ParamTypes2(TZPT_VALUE_INPUT, TZPT_VALUE_OUTPUT);
	ret = KREE_TeeServiceCall(m4u_session, M4U_TZCMD_SEC_INIT, paramTypes, param);
	if (ret != TZ_RESULT_SUCCESS) {
		M4UMSG("m4u sec init error 0x%x\n", ret);
		return -1;
	}
	*secsize = param[1].value.a;
	return 0;
}

int m4u_reg_backup_sec(void)
{
	union MTEEC_PARAM param[4];
	uint32_t paramTypes;
	TZ_RESULT ret;

	paramTypes = TZ_ParamTypes1(TZPT_NONE);
	ret = KREE_TeeServiceCall(m4u_session, M4U_TZCMD_REG_BACKUP, paramTypes, param);
	if (ret != TZ_RESULT_SUCCESS) {
		M4UMSG("m4u reg backup ServiceCall error %d\n", ret);
		return -1;
	}
	return 0;
}

int m4u_reg_restore_sec(void)
{
	union MTEEC_PARAM param[4];
	uint32_t paramTypes;
	TZ_RESULT ret;

	paramTypes = TZ_ParamTypes1(TZPT_NONE);
	ret = KREE_TeeServiceCall(m4u_session, M4U_TZCMD_REG_RESTORE, paramTypes, param);
	if (ret != TZ_RESULT_SUCCESS) {
		M4UMSG("m4u reg backup ServiceCall error %d\n", ret);
		return -1;
	}
	return 0;
}

int m4u_dump_secpgd(unsigned int port, unsigned int faultmva)
{
	union MTEEC_PARAM param[4];
	uint32_t paramTypes;
	TZ_RESULT ret;

	param[0].value.a = port;
	param[0].value.b = faultmva;
	paramTypes = TZ_ParamTypes1(TZPT_VALUE_INPUT);

	ret = KREE_TeeServiceCall(m4u_session, M4U_TZCMD_SECPGTDUMP, paramTypes, param);
	if (ret != TZ_RESULT_SUCCESS) {
		M4UMSG("m4u dump secpgd ServiceCall error %d\n", ret);
		return -1;
	}
	return 0;
}

int m4u_config_port_tee(int port, int virt, int sec, int dis, int dir)
{
	union MTEEC_PARAM param[4];
	uint32_t paramTypes;
	TZ_RESULT ret;

	if (port < 0 || port >= M4U_PORT_UNKNOWN) {
		M4UMSG("config err %d\n", port);
		return -EINVAL;
	}

	M4ULOG_LOW("config_port-sec port=%s Vir=%d Sec=%d Dis=%d Dir=%d\n",
		   m4u_get_port_name(port), virt, sec, dis, dir);

	mutex_lock(&gM4u_port_tee);
	param[0].value.a = port;
	param[0].value.b = virt;
	param[1].value.a = dis;
	param[1].value.b = dir;
	paramTypes = TZ_ParamTypes2(TZPT_VALUE_INPUT, TZPT_VALUE_INPUT);
	ret = KREE_TeeServiceCall(m4u_session, M4U_TZCMD_CONFIG_PORT, paramTypes, param);
	mutex_unlock(&gM4u_port_tee);

	if (ret != TZ_RESULT_SUCCESS) {
		M4UMSG("m4u_config_port ServiceCall error 0x%x\n", ret);
		return -EINVAL;
	}
	return 0;
}

int m4u_config_port_array_tee(unsigned char *port_array)
{
	union MTEEC_PARAM param[4];
	uint32_t paramTypes;
	TZ_RESULT ret;

	mutex_lock(&gM4u_port_tee);
	param[0].mem.buffer = port_array;
	param[0].mem.size = (M4U_PORT_NR + 1) / 2;
	paramTypes = TZ_ParamTypes1(TZPT_VALUE_INPUT);
	ret = KREE_TeeServiceCall(m4u_session, M4U_TZCMD_CONFIG_PORT_ARRAY, paramTypes, param);
	mutex_unlock(&gM4u_port_tee);

	if (ret != TZ_RESULT_SUCCESS) {
		M4UMSG("m4u_config_port_array ServiceCall error 0x%x\n", ret);
		return -EINVAL;
	}
	return 0;
}
#endif

/**********************************************************/
static long MTK_M4U_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int m4u_id = 0;
	int MVA_MAX_BLOCK_NR = 4095;
	int MVA_BLOCK_SIZE_ORDER = 20;
	struct M4U_MOUDLE_STRUCT m4u_module;
	M4U_PORT_STRUCT m4u_port;
	M4U_PORT_ID PortID;
	M4U_PORT_ID ModuleID;
	struct M4U_CACHE_STRUCT m4u_cache_data;
	struct M4U_DMA_STRUCT m4u_dma_data;
	m4u_client_t *client = filp->private_data;

	switch (cmd) {
	case MTK_M4U_T_POWER_ON:
		ret = copy_from_user(&ModuleID, (void *)arg, sizeof(unsigned int));
		if (ret) {
			M4UMSG("MTK_M4U_T_POWER_ON,copy_from_user failed,%d\n", ret);
			return -EFAULT;
		}
		if (ModuleID < 0 || ModuleID >= M4U_PORT_UNKNOWN) {
			M4UMSG("MTK_M4U_T_POWER_ON, moduleid%d is invalid\n", ModuleID);
			return -EFAULT;
		}
		ret = m4u_power_on(ModuleID);
		break;

	case MTK_M4U_T_POWER_OFF:
		ret = copy_from_user(&ModuleID, (void *)arg, sizeof(unsigned int));
		if (ret) {
			M4UMSG("MTK_M4U_T_POWER_OFF,copy_from_user failed,%d\n", ret);
			return -EFAULT;
		}
		if (ModuleID < 0 || ModuleID >= M4U_PORT_UNKNOWN) {
			M4UMSG("MTK_M4U_T_POWER_Off, moduleid%d is invalid\n", ModuleID);
			return -EFAULT;
		}
		ret = m4u_power_off(ModuleID);
		break;

	case MTK_M4U_T_ALLOC_MVA:
		ret = copy_from_user(&m4u_module, (void *)arg, sizeof(struct M4U_MOUDLE_STRUCT));
		if (ret) {
			M4UMSG("MTK_M4U_T_ALLOC_MVA,copy_from_user failed:%d\n", ret);
			return -EFAULT;
		}
		if (m4u_module.port < 0 || m4u_module.port >= M4U_PORT_UNKNOWN) {
			M4UMSG("MTK_M4U_T_ALLOC_MVA, port%d is invalid\n", m4u_module.port);
			return -EFAULT;
		}
		ret = m4u_alloc_mva(client, m4u_module.port, m4u_module.BufAddr, NULL,
				m4u_module.BufSize, m4u_module.prot, m4u_module.flags,
				    &(m4u_module.MVAStart));

		if (ret)
			return ret;

		ret = copy_to_user(&(((struct M4U_MOUDLE_STRUCT *) arg)->MVAStart),
				&(m4u_module.MVAStart), sizeof(unsigned int));
		if (ret) {
			M4UMSG("MTK_M4U_T_ALLOC_MVA,copy_from_user failed:%d\n", ret);
			return -EFAULT;
		}
		break;

	case MTK_M4U_T_DEALLOC_MVA:
		{
			ret = copy_from_user(&m4u_module, (void *)arg, sizeof(struct M4U_MOUDLE_STRUCT));
			if (ret) {
				M4UMSG("MTK_M4U_T_DEALLOC_MVA,copy_from_user failed:%d\n", ret);
				return -EFAULT;
			}
			if (m4u_module.port < 0 || m4u_module.port >= M4U_PORT_UNKNOWN) {
				M4UMSG("MTK_M4U_T_DEALLOC_MVA, port%d is invalid\n", m4u_module.port);
				return -EFAULT;
			}
			if (m4u_module.MVAStart <= 0 ||
				(m4u_module.MVAStart >> MVA_BLOCK_SIZE_ORDER) > MVA_MAX_BLOCK_NR) {
				M4UMSG("MTK_M4U_T_DEALLOC_MVA, mva %d is invalid\n", m4u_module.MVAStart);
				return -EFAULT;
			}
			ret = m4u_dealloc_mva(client, m4u_module.port, m4u_module.MVAStart);
			if (ret)
				return ret;
		}
		break;

	case MTK_M4U_T_DUMP_INFO:
		ret = copy_from_user(&ModuleID, (void *)arg, sizeof(unsigned int));
		if (ret) {
			M4UMSG("MTK_M4U_Invalid_TLB_Range,copy_from_user failed,%d\n", ret);
			return -EFAULT;
		}
		if (ModuleID < 0 || ModuleID >= M4U_PORT_UNKNOWN) {
			M4UMSG("MTK_M4U_Invalid_TLB_Range, port%d is invalid\n", ModuleID);
			return -EFAULT;
		}
		break;

	case MTK_M4U_T_CACHE_SYNC:
		ret = copy_from_user(&m4u_cache_data, (void *)arg, sizeof(struct M4U_CACHE_STRUCT));
		if (ret) {
			M4UMSG("m4u_cache_sync,copy_from_user failed:%d\n", ret);
			return -EFAULT;
		}
		if (m4u_cache_data.port < 0 || m4u_cache_data.port >= M4U_PORT_UNKNOWN) {
			M4UMSG("MTK_M4U_T_CACHE_SYNC, port%d is invalid\n", m4u_cache_data.port);
			return -EFAULT;
		}
		if (m4u_cache_data.mva < 0 || (m4u_cache_data.mva >> MVA_BLOCK_SIZE_ORDER) > MVA_MAX_BLOCK_NR) {
			M4UMSG("MTK_M4U_T_CACHE_SYNC, mva %d is invalid\n", m4u_cache_data.mva);
			return -EFAULT;
		}

		ret = m4u_cache_sync(client, m4u_cache_data.port, m4u_cache_data.va,
				     m4u_cache_data.size, m4u_cache_data.mva,
				     m4u_cache_data.eCacheSync);
		break;

	case MTK_M4U_T_DMA_OP:
		ret = copy_from_user(&m4u_dma_data, (void *) arg,
				sizeof(struct M4U_DMA_STRUCT));
		if (ret) {
			M4UMSG("m4u dma map/unmap area,copy_from_user failed:%d\n", ret);
			return -EFAULT;
		}
		if (m4u_dma_data.port < 0 || m4u_dma_data.port >= M4U_PORT_UNKNOWN) {
			M4UMSG("MTK_M4U_T_DMA_OP, port%d is invalid\n", m4u_dma_data.port);
			return -EFAULT;
		}
		if (m4u_dma_data.mva < 0 || (m4u_dma_data.mva >> MVA_BLOCK_SIZE_ORDER) > MVA_MAX_BLOCK_NR) {
			M4UMSG("MTK_M4U_T_DMA_OP, mva %d is invalid\n", m4u_dma_data.mva);
			return -EFAULT;
		}

		ret = m4u_dma_op(client, m4u_dma_data.port, m4u_dma_data.va,
				m4u_dma_data.size, m4u_dma_data.mva,
				m4u_dma_data.eDMAType, m4u_dma_data.eDMADir);
		break;

	case MTK_M4U_T_CONFIG_PORT:
		ret = copy_from_user(&m4u_port, (void *)arg, sizeof(M4U_PORT_STRUCT));
		if (ret) {
			M4UMSG("MTK_M4U_T_CONFIG_PORT,copy_from_user failed:%d\n", ret);
			return -EFAULT;
		}
		if (m4u_port.ePortID < 0 || m4u_port.ePortID >= M4U_PORT_UNKNOWN) {
			M4UMSG("MTK_M4U_T_CONFIG_PORT, port%d is invalid\n", m4u_port.ePortID);
			return -EFAULT;
		}
		ret = m4u_config_port(&m4u_port);
		break;
	case MTK_M4U_T_MONITOR_START:
		ret = copy_from_user(&PortID, (void *)arg, sizeof(unsigned int));
		if (ret) {
			M4UMSG("MTK_M4U_T_MONITOR_START,copy_from_user failed,%d\n", ret);
			return -EFAULT;
		}
		if (PortID < 0 || PortID >= M4U_PORT_UNKNOWN) {
			M4UMSG("MTK_M4U_T_MONITOR_START, port%d is invalid\n", PortID);
			return -EFAULT;
		}
		m4u_id = m4u_port_2_m4u_id(PortID);
		if (m4u_id < 0) {
			M4UMSG("MTK_M4U_T_MONITOR_START, portID is invalid\n");
			return -EFAULT;
		}
		ret = m4u_monitor_start(m4u_id);

		break;
	case MTK_M4U_T_MONITOR_STOP:
		ret = copy_from_user(&PortID, (void *)arg, sizeof(unsigned int));
		if (ret) {
			M4UMSG("MTK_M4U_T_MONITOR_STOP,copy_from_user failed,%d\n", ret);
			return -EFAULT;
		}
		if (PortID < 0 || PortID >= M4U_PORT_UNKNOWN) {
			M4UMSG("MTK_M4U_T_MONITOR_STOP, port%d is invalid\n", PortID);
			return -EFAULT;
		}
		m4u_id = m4u_port_2_m4u_id(PortID);
		if (m4u_id < 0) {
			M4UMSG("MTK_M4U_T_MONITOR_STOP, portID is invalid\n");
			return -EFAULT;
		}

		ret = m4u_monitor_stop(m4u_id);
		break;
	case MTK_M4U_T_CACHE_FLUSH_ALL:
		m4u_dma_cache_flush_all();
		break;

	case MTK_M4U_T_CONFIG_PORT_ARRAY:
		{
			struct m4u_port_array port_array;

			ret = copy_from_user(&port_array, (void *)arg, sizeof(struct m4u_port_array));
			if (ret) {
				M4UMSG("MTK_M4U_T_CONFIG_PORT,copy_from_user failed:%d\n", ret);
				return -EFAULT;
			}
			ret = m4u_config_port_array(&port_array);
		}
		break;
	case MTK_M4U_T_CONFIG_TF:
		{
			struct M4U_TF_STRUCT rM4UTF;

			ret = copy_from_user(&rM4UTF, (void *)arg, sizeof(struct M4U_TF_STRUCT));
			if (ret) {
				M4UMSG("MTK_M4U_T_CONFIG_TF,copy_from_user failed:%d\n", ret);
				return -EFAULT;
			}
			if (rM4UTF.port < 0 || rM4UTF.port >= M4U_PORT_UNKNOWN) {
				M4UMSG("MTK_M4U_T_CONFIG_TF, port%d is invalid\n", rM4UTF.port);
				return -EFAULT;
			}

			ret = m4u_enable_tf(rM4UTF.port, rM4UTF.fgEnable);
		}
		break;
#ifdef M4U_TEE_SERVICE_ENABLE
	case MTK_M4U_T_SEC_INIT:
		{
			M4UMSG("MTK M4U ioctl : MTK_M4U_T_SEC_INIT command!! 0x%x\n", cmd);
			/*ret = m4u_sec_init();*/
		}
		break;
#endif
	default:
		/* M4UMSG("MTK M4U ioctl:No such command!!\n"); */
		ret = -EINVAL;
		break;
	}

	return ret;
}

#if IS_ENABLED(CONFIG_COMPAT)

typedef struct {
	compat_uint_t port;
	compat_ulong_t BufAddr;
	compat_uint_t BufSize;
	compat_uint_t prot;
	compat_uint_t MVAStart;
	compat_uint_t MVAEnd;
	compat_uint_t flags;
} COMPAT_M4U_MOUDLE_STRUCT;

typedef struct {
	compat_uint_t port;
	compat_uint_t eCacheSync;
	compat_ulong_t va;
	compat_uint_t size;
	compat_uint_t mva;
} COMPAT_M4U_CACHE_STRUCT;

typedef struct {
	compat_uint_t port;
	compat_uint_t eDMAType;
	compat_uint_t eDMADir;
	compat_ulong_t va;
	compat_uint_t size;
	compat_uint_t mva;
} COMPAT_M4U_DMA_STRUCT;

#define COMPAT_MTK_M4U_T_ALLOC_MVA	  _IOWR(MTK_M4U_MAGICNO, 4, int)
#define COMPAT_MTK_M4U_T_DEALLOC_MVA  _IOW(MTK_M4U_MAGICNO, 5, int)
#define COMPAT_MTK_M4U_T_CACHE_SYNC   _IOW(MTK_M4U_MAGICNO, 10, int)
#define COMPAT_MTK_M4U_T_DMA_OP		_IOW(MTK_M4U_MAGICNO, 29, int)


static int compat_get_m4u_module_struct(COMPAT_M4U_MOUDLE_STRUCT __user *data32,
					struct M4U_MOUDLE_STRUCT __user *data)
{
	compat_uint_t u;
	compat_ulong_t l;
	int err;

	err = get_user(u, &(data32->port));
	err |= put_user(u, &(data->port));
	err |= get_user(l, &(data32->BufAddr));
	err |= put_user(l, &(data->BufAddr));
	err |= get_user(u, &(data32->BufSize));
	err |= put_user(u, &(data->BufSize));
	err |= get_user(u, &(data32->prot));
	err |= put_user(u, &(data->prot));
	err |= get_user(u, &(data32->MVAStart));
	err |= put_user(u, &(data->MVAStart));
	err |= get_user(u, &(data32->MVAEnd));
	err |= put_user(u, &(data->MVAEnd));
	err |= get_user(u, &(data32->flags));
	err |= put_user(u, &(data->flags));

	return err;
}

static int compat_put_m4u_module_struct(COMPAT_M4U_MOUDLE_STRUCT __user *data32,
					struct M4U_MOUDLE_STRUCT __user *data)
{
	compat_uint_t u;
	compat_ulong_t l;
	int err;

	err = get_user(u, &(data->port));
	err |= put_user(u, &(data32->port));
	err |= get_user(l, &(data->BufAddr));
	err |= put_user(l, &(data32->BufAddr));
	err |= get_user(u, &(data->BufSize));
	err |= put_user(u, &(data32->BufSize));
	err |= get_user(u, &(data->prot));
	err |= put_user(u, &(data32->prot));
	err |= get_user(u, &(data->MVAStart));
	err |= put_user(u, &(data32->MVAStart));
	err |= get_user(u, &(data->MVAEnd));
	err |= put_user(u, &(data32->MVAEnd));
	err |= get_user(u, &(data->flags));
	err |= put_user(u, &(data32->flags));

	return err;
}

static int compat_get_m4u_cache_struct(COMPAT_M4U_CACHE_STRUCT __user *data32,
				       struct M4U_CACHE_STRUCT __user *data)
{
	compat_uint_t u;
	compat_ulong_t l;
	int err;

	err = get_user(u, &(data32->port));
	err |= put_user(u, &(data->port));
	err |= get_user(u, &(data32->eCacheSync));
	err |= put_user(u, &(data->eCacheSync));
	err |= get_user(l, &(data32->va));
	err |= put_user(l, &(data->va));
	err |= get_user(u, &(data32->size));
	err |= put_user(u, &(data->size));
	err |= get_user(u, &(data32->mva));
	err |= put_user(u, &(data->mva));

	return err;
}

static int compat_get_m4u_dma_struct(
			COMPAT_M4U_DMA_STRUCT __user *data32,
			struct M4U_DMA_STRUCT __user *data)
{
	compat_uint_t u;
	compat_ulong_t l;
	int err;

	err = get_user(u, &(data32->port));
	err |= put_user(u, &(data->port));
	err |= get_user(u, &(data32->eDMAType));
	err |= put_user(u, &(data->eDMAType));
	err |= get_user(u, &(data32->eDMADir));
	err |= put_user(u, &(data->eDMADir));
	err |= get_user(l, &(data32->va));
	err |= put_user(l, &(data->va));
	err |= get_user(u, &(data32->size));
	err |= put_user(u, &(data->size));
	err |= get_user(u, &(data32->mva));
	err |= put_user(u, &(data->mva));

	return err;
}

long MTK_M4U_COMPAT_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_MTK_M4U_T_ALLOC_MVA:
	{
		COMPAT_M4U_MOUDLE_STRUCT __user *data32;
		struct M4U_MOUDLE_STRUCT __user *data;
		int err;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(struct M4U_MOUDLE_STRUCT));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_m4u_module_struct(data32, data);
		if (err)
			return err;

		ret = filp->f_op->unlocked_ioctl(filp, MTK_M4U_T_ALLOC_MVA, (unsigned long)data);

		err = compat_put_m4u_module_struct(data32, data);

		if (err)
			return err;

		return ret;
	}
	case COMPAT_MTK_M4U_T_DEALLOC_MVA:
	{
		COMPAT_M4U_MOUDLE_STRUCT __user *data32;
		struct M4U_MOUDLE_STRUCT __user *data;
		int err;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(struct M4U_MOUDLE_STRUCT));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_m4u_module_struct(data32, data);
		if (err)
			return err;

		return filp->f_op->unlocked_ioctl(filp, MTK_M4U_T_DEALLOC_MVA,
				(unsigned long)data);
	}
	case COMPAT_MTK_M4U_T_CACHE_SYNC:
	{
		COMPAT_M4U_CACHE_STRUCT __user *data32;
		struct M4U_CACHE_STRUCT __user *data;
		int err;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(struct M4U_CACHE_STRUCT));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_m4u_cache_struct(data32, data);
		if (err)
			return err;

		return filp->f_op->unlocked_ioctl(filp, MTK_M4U_T_CACHE_SYNC,
				(unsigned long)data);
	}
	case COMPAT_MTK_M4U_T_DMA_OP:
	{
		COMPAT_M4U_DMA_STRUCT __user *data32;
		struct M4U_DMA_STRUCT __user *data;
		int err;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(struct M4U_DMA_STRUCT));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_m4u_dma_struct(data32, data);
		if (err)
			return err;

		return filp->f_op->unlocked_ioctl(filp, MTK_M4U_T_DMA_OP,
								(unsigned long)data);
}
	case MTK_M4U_T_POWER_ON:
	case MTK_M4U_T_POWER_OFF:
	case MTK_M4U_T_DUMP_INFO:
	case MTK_M4U_T_CONFIG_PORT:
	case MTK_M4U_T_MONITOR_START:
	case MTK_M4U_T_MONITOR_STOP:
	case MTK_M4U_T_CACHE_FLUSH_ALL:
	case MTK_M4U_T_CONFIG_PORT_ARRAY:
	case MTK_M4U_T_SEC_INIT:
		return filp->f_op->unlocked_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
	default:
		return -ENOIOCTLCMD;
	}
}

#else

#define MTK_M4U_COMPAT_ioctl  NULL

#endif

static const struct file_operations m4u_fops = {
	.owner = THIS_MODULE,
	.open = MTK_M4U_open,
	.release = MTK_M4U_release,
	.flush = MTK_M4U_flush,
	.unlocked_ioctl = MTK_M4U_ioctl,
	.compat_ioctl = MTK_M4U_COMPAT_ioctl,
	/* .mmap = NULL; */
};

static int m4u_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;

	M4UINFO("m4u_probe 0\n");

	if (pdev->dev.of_node) {
		int err;

		err = of_property_read_u32(node, "cell-index", &pdev->id);
		if (err)
			M4UMSG("[DTS] get m4u platform_device id fail!!\n");
	}
	M4UINFO("m4u_probe 1, pdev id = %d name = %s\n", pdev->id, pdev->name);

	gM4uDev->pDev[pdev->id] = &pdev->dev;
	gM4uDev->m4u_base[pdev->id] = (unsigned long)of_iomap(node, 0);
	gM4uDev->irq_num[pdev->id] = irq_of_parse_and_map(node, 0);

	M4UINFO("m4u_probe 2, of_iomap: 0x%lx, irq_num: %d, pDev: %p\n",
		gM4uDev->m4u_base[pdev->id], gM4uDev->irq_num[pdev->id], gM4uDev->pDev[pdev->id]);

	if (pdev->id >= TOTAL_M4U_NUM) {
		M4UMSG("m4u_probe id(%d) is error...\n", pdev->id);
		return 0;
	}

	if (pdev->id == 0) {
		m4u_domain_init(gM4uDev, &gMvaNode_unknown);

#ifdef M4U_TEE_SERVICE_ENABLE
		if (!m4u_tee_en) {
			struct m4u_buf_info_t *pMvaInfo;
			unsigned int mva;
			unsigned int securitymemsize = 0;
			int ret = 0;

			m4u_session_init();
			ret = m4u_sec_init(m4u_get_domain_by_id(0)->pgd_pa, 1, &securitymemsize);
			if (ret)
				return ret;

			pMvaInfo = m4u_alloc_buf_info();
			if (pMvaInfo) {
				securitymemsize = ((securitymemsize + SZ_1M - 1) & (~(SZ_1M - 1)));
				pMvaInfo->port = M4U_PORT_UNKNOWN;
				pMvaInfo->size = securitymemsize;
				mva = m4u_do_mva_alloc(0, securitymemsize, pMvaInfo);
				pMvaInfo->mva = mva;
				M4UMSG("Reserve sec mva: 0x%x,secmvasz %dM\n",
				       mva, securitymemsize / 1024 / 1024);
			}
		}
#endif
	}

	m4u_hw_init(gM4uDev, pdev->id);

	M4UINFO("m4u_probe 3 finish...\n");

	return 0;
}

static int m4u_remove(struct platform_device *pdev)
{
	m4u_hw_deinit(gM4uDev, pdev->id);

#ifndef __M4U_USE_PROC_NODE
	misc_deregister(&(gM4uDev->dev));
#else
	if (gM4uDev->m4u_dev_proc_entry)
		proc_remove(gM4uDev->m4u_dev_proc_entry);
#endif

	return 0;
}

static int m4u_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	m4u_reg_backup();
	M4UINFO("M4U backup in suspend\n");

	return 0;
}

static int m4u_resume(struct platform_device *pdev)
{
	m4u_reg_restore();
	M4UINFO("M4U restore in resume\n");
	return 0;
}

/*---------------------------------------------------------------------------*/
#ifdef CONFIG_PM
/*---------------------------------------------------------------------------*/
static int m4u_pm_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);

	if (pdev == NULL)
		M4UERR("pdev is NULL!\n");

	return m4u_suspend(pdev, PMSG_SUSPEND);
}

static int m4u_pm_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);

	if (pdev == NULL)
		M4UERR("pdev is NULL!\n");

	return m4u_resume(pdev);
}

static int m4u_pm_restore_noirq(struct device *device)
{
	int i;

	for (i = 0; i < TOTAL_M4U_NUM; i++)
		irq_set_irq_type(gM4uDev->irq_num[i], IRQF_TRIGGER_LOW);


	return 0;
}

/*---------------------------------------------------------------------------*/
#else				/*CONFIG_PM */
/*---------------------------------------------------------------------------*/
#define m4u_pm_suspend NULL
#define m4u_pm_resume NULL
#define m4u_pm_restore_noirq NULL
/*---------------------------------------------------------------------------*/
#endif				/*CONFIG_PM */
/*---------------------------------------------------------------------------*/
static const struct of_device_id iommu_of_ids[] = {
	{.compatible = "mediatek,m4u",},
	{.compatible = "mediatek,perisys_iommu",},
	{}
};

const struct dev_pm_ops m4u_pm_ops = {
	.suspend = m4u_pm_suspend,
	.resume = m4u_pm_resume,
	.freeze = m4u_pm_suspend,
	.thaw = m4u_pm_resume,
	.poweroff = m4u_pm_suspend,
	.restore = m4u_pm_resume,
	.restore_noirq = m4u_pm_restore_noirq,
};

static struct platform_driver m4uDrv = {
	.probe = m4u_probe,
	.remove = m4u_remove,
	.suspend = m4u_suspend,
	.resume = m4u_resume,
	.driver = {
		   .name = "m4u",
		   .of_match_table = iommu_of_ids,
#ifdef CONFIG_PM
		   .pm = &m4u_pm_ops,
#endif
		   .owner = THIS_MODULE,
		   }
};

#if 0
static u64 m4u_dmamask = ~(u32) 0;

static struct platform_device mtk_m4u_dev = {
	.name = M4U_DEV_NAME,
	.id = 0,
	.dev = {
		.dma_mask = &m4u_dmamask,
		.coherent_dma_mask = 0xffffffffUL}
};
#endif

#define __M4U_USE_PROC_NODE

static int __init MTK_M4U_Init(void)
{
	int ret = 0;

	gM4uDev = kzalloc(sizeof(struct m4u_device), GFP_KERNEL);

	M4UINFO("MTK_M4U_Init kzalloc: %p\n", gM4uDev);

	if (!gM4uDev) {
		M4UMSG("kmalloc for m4u_device fail\n");
		return -ENOMEM;
	}
#ifndef __M4U_USE_PROC_NODE
	gM4uDev->dev.minor = MISC_DYNAMIC_MINOR;
	gM4uDev->dev.name = M4U_DEV_NAME;
	gM4uDev->dev.fops = &m4u_fops;
	gM4uDev->dev.parent = NULL;

	ret = misc_register(&(gM4uDev->dev));
	M4UINFO("misc_register, minor: %d\n", gM4uDev->dev.minor);
	if (ret) {
		M4UMSG("failed to register misc device.\n");
		return ret;
	}
#else
	gM4uDev->m4u_dev_proc_entry = proc_create("m4u", 0, NULL, &m4u_fops);
	if (!(gM4uDev->m4u_dev_proc_entry)) {
		M4UMSG("m4u:failed to register m4u in proc/m4u_device.\n");
		return ret;
	}
#endif

	m4u_debug_init(gM4uDev);

	M4UINFO("M4U platform_driver_register start\n");

	if (platform_driver_register(&m4uDrv)) {
		M4UMSG("failed to register M4U driver");
		return -ENODEV;
	}
	M4UINFO("M4U platform_driver_register finsish\n");

#if 0
	retval = platform_device_register(&mtk_m4u_dev);
	if (retval != 0)
		return retval;
#endif

#ifdef M4U_PROFILE
	m4u_profile_init();
#endif

	return 0;
}

static int __init mtk_m4u_late_init(void)
{
	return 0;
}

static void __exit MTK_M4U_Exit(void)
{
	platform_driver_unregister(&m4uDrv);
}

subsys_initcall(MTK_M4U_Init);
late_initcall(mtk_m4u_late_init);
module_exit(MTK_M4U_Exit);

MODULE_DESCRIPTION("MTKM4Udriver");
MODULE_AUTHOR("MTK80347 <Xiang.Xu@mediatek.com>");
MODULE_LICENSE("GPL");
