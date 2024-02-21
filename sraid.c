#ifndef CONFIG_BLOCK
#define CONFIG_BLOCK
#endif

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/genhd.h>
#include <linux/hdreg.h>
#include <linux/kthread.h>

#include <trace/events/block.h>

#define DEVICE_NAME "sraids"
#define KERNEL_SECTOR_SIZE 512
#define BDEV_MODE (FMODE_READ)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lai Liang-Wei");
MODULE_DESCRIPTION("ScaleRAID Simulate in one physical disk");
MODULE_VERSION("0.1");

static DECLARE_WAIT_QUEUE_HEAD(req_event);

static int sraids_open(struct block_device *, fmode_t);
static int sraids_release(struct gendisk *, fmode_t);
static int sraids_rw_page(struct block_device *,
                          sector_t,
                          struct page *,
                          unsigned int);
static int sraids_ioctl(struct block_device *,
                        fmode_t,
                        unsigned,
                        unsigned long);

/* System var*/
static int majorNum;
static struct device *sraidsDevice = NULL;
static int opend_times = 0;
module_param(opend_times, int, S_IRUGO);

static int LOGICAL_BLOCK_SIZE = 512;
module_param(LOGICAL_BLOCK_SIZE, int, 0);

/* Simulator var */
static int region_size;
static int datadisk_num;

static struct sraidsbd_t {
    sector_t sectorSize;
    struct gendisk *gd;
    spinlock_t lock;
    /* BIO list management for use by remapping drivers (e.g. DM or MD) and
     * loop. */
    struct bio_list blist;
    struct task_struct *thread;
    int stats;
    struct block_device *raw_dev;
    struct request_queue *queue;
} sraidsbd;

static int sraids_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
    long long size;
    size = sraidsbd.sectorSize * (LOGICAL_BLOCK_SIZE / KERNEL_SECTOR_SIZE);
    geo->cylinders = (size & ~0x3f) >> 6;
    geo->heads = 4;
    geo->sectors = 16;
    geo->start = 0;
    return 0;
}

static struct block_device_operations bdops = {
    .owner = THIS_MODULE,
    //.open = sraids_open,
    //.release = sraids_release,
    .rw_page = sraids_rw_page,
    .ioctl = sraids_ioctl,
    .getgeo = sraids_getgeo,
};
static int sraids_open(struct block_device *bdev, fmode_t mode)
{
    printk(KERN_INFO "SRAIDS: Device has been opened \n");
    opend_times++;
    return 0;
}
static int sraids_release(struct gendisk *gd, fmode_t mode)
{
    printk(KERN_INFO "SRAIDS: Device successfully closed\n");
    opend_times--;
    return 0;
}

static int sraids_rw_page(struct block_device *bdev,
                          sector_t sector,
                          struct page *page,
                          unsigned int mode)
{
}

static void make_request(struct request_queue *rq, struct bio *b)
{
    printk(KERN_INFO
           "SRAIDS: Make request \"%-5s\" #pages: %-4hu #total-size: "
           "%-10u\n",
           (bio_data_dir(b) == WRITE) ? "write" : "read", b->bi_sector,
           b->bi_vcnt, b->bi_size);
    spin_lock_irq(&sraidsbd.lock);
    if (!sraidsbd.raw_dev) {
        printk("SRAIDS: Request before raw_dev is ready, aborting\n");
        goto abort;
    }
    if (!sraidsbd.stats) {
        printk("SRAIDS: Device not active yet, aborting\n");
        goto abort;
    }
    bio_list_add(&sraidsbd.blist, b);
    wake_up(&req_event);
    spin_unlock_irq(&sraidsbd.lock);

    return;

abort:
    spin_unlock_irq(&sraidsbd.lock);
    printk("<%p> Abort request\n\n", b);
    bio_io_error(b);
}

static struct block_device *sraids_bdev_open(struct sraidsbd_t *dev,
                                             char dev_path[])
{
    /* Open underlying device */
    struct block_device *bdev_raw = lookup_bdev(dev_path);

    if (IS_ERR(bdev_raw)) {
        printk("SRAIDS: error opening raw device <%lu>\n", PTR_ERR(bdev_raw));
        return NULL;
    }
    printk("Opened %s\n", dev_path);

    if (!bdget(bdev_raw->bd_dev)) {
        printk("SRAIDS: error bdget()\n");
        return NULL;
    }

    if (blkdev_get(bdev_raw, BDEV_MODE, dev)) {
        printk("SRAIDS: error blkdev_get()\n");
        bdput(bdev_raw);
        return NULL;
    }
    return bdev_raw;
}

static void stackbd_io_fn(struct sraidsbd_t *dev, struct bio *bio)
{
    bio->bi_bdev = dev->raw_dev;

    trace_block_bio_remap(bdev_get_queue(dev->raw_dev), bio,
                          bio->bi_bdev->bd_dev, bio->bi_sector);

    /* No need to call bio_endio() */
    generic_make_request(bio);
}

static int sraids_threadfn(void *data)
{
    struct bio *bio;
    struct sraidsbd_t *dev = (struct sraidsbd_t *) data;
    set_user_nice(current, -20);

    while (!kthread_should_stop()) {
        /* wake_up() is after adding bio to list. No need for condition */
        wait_event_interruptible(
            req_event, kthread_should_stop() || !bio_list_empty(&dev->blist));

        spin_lock_irq(&dev->lock);
        if (bio_list_empty(&dev->blist)) {
            spin_unlock_irq(&dev->lock);
            continue;
        }

        bio = bio_list_pop(&dev->blist);
        spin_unlock_irq(&dev->lock);

        stackbd_io_fn(dev, bio);
    }

    return 0;
}

static int sraids_start(struct sraidsbd_t *dev, char dev_path[])
{
    unsigned max_sectors;

    if (!(dev->raw_dev = sraids_bdev_open(dev, dev_path)))
        return -EFAULT;
    dev->sectorSize = get_capacity(dev->raw_dev->bd_disk);
    printk("SRAIDS: Device real capacity: %llu\n", dev->sectorSize);

    set_capacity(dev->gd, dev->sectorSize);

    max_sectors = queue_max_hw_sectors(bdev_get_queue(dev->raw_dev));
    blk_queue_max_hw_sectors(dev->queue, max_sectors);
    printk("SRAIDS: Max sectors: %u\n", max_sectors);

    dev->thread = kthread_create(sraids_threadfn, dev, dev->gd->disk_name);
    if (IS_ERR(dev->thread)) {
        printk("SRAIDS: error kthread_create <%lu>\n", PTR_ERR(dev->thread));
        goto error_after_bdev;
    }

    printk("SRAIDS: initialization successfully\n");
    dev->stats = 1;
    wake_up_process(dev->thread);

    return 0;

error_after_bdev:
    blkdev_put(dev->raw_dev, BDEV_MODE);
    bdput(dev->raw_dev);

    return -EFAULT;
}

static int sraids_ioctl(struct block_device *blkdev,
                        fmode_t mode,
                        unsigned cmd,
                        unsigned long arg)
{
    char dev_path[80];
    void __user *argp = (void __user *) arg;
    switch (cmd) {
    case 0x0:
        printk("\n request io :0 \n\n");
        if (copy_from_user(dev_path, argp, sizeof(dev_path)))
            return -EFAULT;
        return sraids_start(&sraidsbd, dev_path);
        break;
    case 0x1:
        break;
    default:
        return -ENOTTY;
        break;
    }
}



static int create_block_device(struct sraidsbd_t *dev)
{
    /* init request queue */
    if (!(dev->queue = blk_alloc_queue(GFP_KERNEL))) {
        printk(KERN_ALERT "SRAIDS: failed to init blk queue\n");
        return -ENOMEM;
    }
    blk_queue_logical_block_size(dev->queue, KERNEL_SECTOR_SIZE);
    blk_queue_make_request(dev->queue, make_request);
    dev->queue->queuedata = dev;

    /* init dev gendisk */
    dev->gd = alloc_disk(1);
    if (!dev->gd) {
        printk(KERN_ALERT "SRAIDS: alloc_disk failure\n");
        return -ENOMEM;
    }
    dev->gd->major = majorNum;
    dev->gd->first_minor = 0;
    dev->gd->fops = &bdops;
    dev->gd->queue = dev->queue;
    snprintf(dev->gd->disk_name, 32, "sraids");
    dev->gd->private_data = dev;

    add_disk(dev->gd);
    return 0;
}
static int delete_block_device(struct sraidsbd_t *dev)
{
    if (dev->stats) {
        kthread_stop(dev->thread);
        blkdev_put(dev->raw_dev, BDEV_MODE);
        bdput(dev->raw_dev);
    }

    if (dev->gd) {
        del_gendisk(dev->gd);
    }
    if (dev->queue) {
        blk_cleanup_queue(dev->queue);
    }
}
static int __init sraids_init(void)
{
    int status;
    printk(KERN_INFO "SRAIDS: Load ScaleRAID Simulator LKM\n");
    spin_lock_init(&sraidsbd.lock);

    majorNum = register_blkdev(0, DEVICE_NAME);
    if (majorNum < 0) {
        printk(KERN_ALERT "SRAIDS: failed to register a major number\n");
        return -EBUSY;
    }

    status = create_block_device(&sraidsbd);
    if (status < 0) {
        printk(KERN_ALERT "SRAIDS: failed to rcreate_block_device\n");
        return status;
    }

    printk(KERN_INFO "SRAIDS: device created correctly\n");
    return 0;
}


static void __exit sraids_exit(void)
{
    unregister_blkdev(majorNum, DEVICE_NAME);
    delete_block_device(&sraidsbd);
    printk(KERN_INFO "SRAIDS: unload module\n");
}

module_init(sraids_init);
module_exit(sraids_exit);
