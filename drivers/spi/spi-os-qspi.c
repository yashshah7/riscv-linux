/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2019 SiFive, Inc. */
/* Open-Silicon QSPI controller driver */


#include <linux/clk.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/io.h>
#include <linux/spi/spi-mem.h>
#include <linux/iopoll.h>


#define OS_QSPI_DRIVER_NAME           "os_qspi"

/* register offsets */

#define OS_QSPI_CMD_REG 			0x00UL
#define OS_QSPI_ADDR_REG 			0x04UL
#define OS_QSPI_DUMMY_DLP_REG 			0x08UL
#define OS_QSPI_MODE_REG			0x0CUL
#define OS_QSPI_CMD_CFG				0x10UL
#define OS_QSPI_TX_DATA 			0x14UL
#define OS_QSPI_RX_DATA 			0x18UL
#define OS_QSPI_START_CMD_REG 			0x1CUL
#define OS_QSPI_CUSTOM_CMD_REG 			0x20UL
#define OS_QSPI_INTR_STAT_REG	 		0x24UL
#define OS_QSPI_INT_MASK_CLR 			0x28UL
#define OS_QSPI_BAUD_RATE_REG 			0x2CUL
#define OS_QSPI_BURST_CTRL_REG			0x30UL
#define OS_QSPI_SYS_STATUS_REG			0x34UL

/*
 * QSPI Custom command registers bit Masks
 */
#define OS_QSPI_OPCODE_PH_EN			(1U << 0)
#define OS_QSPI_ADDR_PH_EN			(1U << 1)
#define OS_QSPI_MODE_BITS_PH_EN			(1U << 2)
#define OS_QSPI_DUMMY_PH_EN			(1U << 3)
#define OS_QSPI_RX_DATA_PH_EN			(1U << 5)
#define OS_QSPI_TX_DATA_PH_EN			(1U << 6)

/*
 * QSPI interrupt stat bit Masks
 */
#define OS_QSPI_INTR_MASK_INTERRUPT		(1U << 0)
#define OS_QSPI_INTR_MASK_AXI_RX_FIFO_FULL	(1U << 1)
#define OS_QSPI_INTR_MASK_AXI_TX_FIFO_EMPTY	(1U << 2)
#define OS_QSPI_INTR_MASK_APB_RX_OK		(1U << 3)
#define OS_QSPI_INTR_MASK_APB_TX_OK		(1U << 4)
#define OS_QSPI_INTR_MASK_TX_BURST_OK		(1U << 5)

/*
 * QSPI Configuration Register bit Masks
 */

#define OS_QSPI_PH_SHIFT_BYPASS 		(1U << 24)
#define OS_QSPI_RX_DLY_HALFCLK			(1U << 31)
#define OS_QSPI_RW 				(1U << 17)
#define OS_QSPI_HOLD				(1U << 16)
#define OS_QSPI_WP				(1U << 15)
#define OS_QSPI_CPOL				(1U << 9)
#define OS_QSPI_ENDI				(1U << 8)
#define OS_QSPI_ADDR_LEN			(1U << 1)
#define OS_QSPI_TX_LEN_32			(3U << 1)
#define OS_QSPI_RX_LEN_32			(3U << 3)
#define OS_QSPI_DEVICE_0			(1U << 6)
#define OS_QSPI_PROTO_SINGLE      		(0U << 10)
#define OS_QSPI_PROTO_DUAL        		(1U << 10)
#define OS_QSPI_PROTO_QUAD        		(2U << 10)
#define OS_QSPI_PROTO_QUAD_DDR        		(3U << 10)
#define OS_QSPI_CMD_ADDR_SERIAL 		(0U << 12)
#define OS_QSPI_CMD_SERIAL 			(1U << 12)
#define OS_QSPI_APB_ACCESS 			(0U << 18)
#define OS_QSPI_SINGLE_BURST			(2U << 18)
#define OS_QSPI_0CLK				(0U << 20)
#define OS_QSPI_0PS				(0U << 22)

/*
 * QSPI baudrate bit
 */
#define OS_QSPI_DIV_1				0

/*
 * QSPI system status register state machine
 */
#define	OS_QSPI_IDLE_STATE			0
#define OS_QSPI_CMD_STATE			1
#define OS_QSPI_ADDR_STATE			2
#define OS_QSPI_MODE_STATE			3
#define	OS_QSPI_DUMMY_STATE			4
#define OS_QSPI_DLP_STATE			5
#define OS_QSPI_RXD_STATE			6
#define OS_QSPI_TXD_STATE			7
#define OS_QSPI_CFG_ERR				8

#define MODE_MM					0
#define MODE_APBW				1
#define MODE_APBR				2
#define OS_FIFO_TIMEOUT_US			100
#define OS_QSPI_DEFAULT_NUM_CS			0x2
#define OS_QSPI_DEFAULT_DEPTH			32

/**
 * struct os_qspi - Defines qspi driver instance
 * @regs:		Virtual address of the QSPI controller registers
 * @irq:                IRQ number
 * list_lock:		list synchronization
 * @fifo_depth:		fifo depth in words
 * burstmode:		SPI access mode
 * @done:		wake-up from interrupt
 */

struct os_qspi {
	void __iomem	*regs;
	void __iomem 	*mm_base;
	u32		irq;
	struct mutex	list_lock;
	u32		fifo_depth;
	u32		burstmode;
	struct		completion done;
};

/*
 * Inline functions for the QSPI controller read/write
 */

static void os_qspi_write(struct os_qspi *qspi, int offset, u32 value)
{
	writel(value, qspi->regs + offset);
}

static u32 os_qspi_read(struct os_qspi *qspi, int offset)
{
	return readl(qspi->regs + offset);
}

static void os_qspi_read_fifo(u32 *val, void __iomem *addr)
{
	*val = readl_relaxed(addr); //check which one should use
}

static void os_qspi_write_fifo(u32 *val, void __iomem *addr)
{
	writel_relaxed(*val, addr);
}

/**
 * os_qspi_setup - Initialize the controller
 * @qspi:      Pointer to the os_qspi structure
 *
 * This function performs the following actions
 *      - clear all the interrupts
 *      - Set the size of the word to be transferred and recived set as 32 bit
 *      - Set the size of the address as 32 bit
 *      - Set the MSB first
 *      - Set baudrate
 */
static int os_qspi_setup(struct spi_device *spi)
{
	struct spi_controller *ctrl = spi->master;
	struct os_qspi *qspi = spi_controller_get_devdata(ctrl);
	u32 cfg = 0;

	if (ctrl->busy)
		return -EBUSY;
#if 0
	TBD
	clk_enable(qspi->refclk);
	clk_enable(qspi->pclk);
#endif
	cfg = os_qspi_read(qspi, OS_QSPI_CMD_CFG);
	cfg |= ~OS_QSPI_PH_SHIFT_BYPASS | ~OS_QSPI_RX_DLY_HALFCLK | OS_QSPI_WP | OS_QSPI_HOLD | ~OS_QSPI_ENDI | OS_QSPI_TX_LEN_32 | OS_QSPI_RX_LEN_32 | OS_QSPI_DEVICE_0 | OS_QSPI_0CLK | OS_QSPI_0PS;
	os_qspi_write(qspi, OS_QSPI_CMD_CFG,cfg);

	/* Clear the interrupts */
	os_qspi_write(qspi, OS_QSPI_INT_MASK_CLR, ~0U);

	os_qspi_write(qspi, OS_QSPI_BAUD_RATE_REG, OS_QSPI_DIV_1);

	return 0;

}

/**
 * os_qspi_irq - Interrupt service routine of the QSPI controller
 * @irq:        IRQ number
 * @dev_id:     Pointer to the xqspi structure
 * Return:      IRQ_HANDLED when interrupt is handled; IRQ_NONE otherwise.
 */

static irqreturn_t os_qspi_irq(int irq, void *dev_id)
{
	struct os_qspi *qspi = (struct os_qspi *)dev_id;
	u32 sr;

	sr = os_qspi_read(qspi,OS_QSPI_INTR_STAT_REG);

	if (!(sr & OS_QSPI_INTR_MASK_APB_TX_OK)) {
		/* disable irq  TBD*/
		complete(&qspi->done);
	}

	return IRQ_HANDLED;
}


static int os_qspi_mmap_read(struct os_qspi *qspi, const struct spi_mem_op *op)
{
	u8 dummy_buf[1];

	/*dummy read*/
	memcpy_fromio(dummy_buf, qspi->mm_base + op->addr.val,1);

	memcpy_fromio(op->data.buf.in, qspi->mm_base + op->addr.val,
			op->data.nbytes);
	return 0;

}

static int os_qspi_wait_idle(struct os_qspi *qspi)
{
	u32 sr;

	return readl_relaxed_poll_timeout_atomic(qspi->regs + OS_QSPI_SYS_STATUS_REG, sr,
			!(sr | OS_QSPI_IDLE_STATE), 1, 100);
}


static int os_qspi_xfer(struct os_qspi *qspi,const struct spi_mem_op *op)
{
	void (*xfer_fifo)(u32 *val, void __iomem *addr);
	u32 len = op->data.nbytes,sr,mask;
	u32 *buf;
	int ret,reg;

	if (op->data.dir == SPI_MEM_DATA_IN) {
		xfer_fifo = os_qspi_read_fifo;
		buf = op->data.buf.in;
		reg = OS_QSPI_RX_DATA ;
		mask = OS_QSPI_INTR_MASK_APB_RX_OK;
	} else {
		xfer_fifo = os_qspi_write_fifo;
		buf = (u32 *)op->data.buf.out;
		reg = OS_QSPI_TX_DATA ;
		mask = OS_QSPI_INTR_MASK_APB_TX_OK;
	}


	os_qspi_write(qspi,OS_QSPI_START_CMD_REG,0x1);

	while (len--) {
		ret = readl_relaxed_poll_timeout_atomic(qspi->regs + OS_QSPI_INTR_STAT_REG,
				sr, (sr & mask), 1,
				OS_FIFO_TIMEOUT_US);
		if (ret) {
			printk(KERN_ERR "fifo timeout (len:%d stat:%#x)\n",
					len, sr);
			return ret;
		}
		os_qspi_write(qspi, OS_QSPI_INT_MASK_CLR, ~mask);
		xfer_fifo(buf++, qspi->regs + reg);
	}

	return 0;
}

static int os_qspi_xfer_mode(struct os_qspi *qspi, const struct spi_mem_op *op)
{
	if (!op->data.nbytes)
		return 0;

	if (qspi->burstmode == MODE_MM)
		return os_qspi_mmap_read(qspi, op);

	return os_qspi_xfer(qspi, op);
}

static int os_qspi_xfer_setup(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct os_qspi *qspi = spi_controller_get_devdata(mem->spi->master);
	u32 custom_cmd, cfg, addr_max;
	int err = 0;

	printk(KERN_DEBUG "cmd:%#x mode:%d.%d.%d.%d addr:%#llx len:%#x\n",
			op->cmd.opcode, op->cmd.buswidth, op->addr.buswidth,
			op->dummy.buswidth, op->data.buswidth,
			op->addr.val, op->data.nbytes);

	err = os_qspi_wait_idle(qspi);
	if (err)
		return -EBUSY;


	addr_max = op->addr.val + op->data.nbytes + 1;
	cfg = os_qspi_read(qspi, OS_QSPI_CMD_CFG);
	custom_cmd = os_qspi_read(qspi,OS_QSPI_CUSTOM_CMD_REG);

	if (op->data.dir == SPI_MEM_DATA_IN) {
		custom_cmd |= OS_QSPI_RX_DATA_PH_EN | ~OS_QSPI_TX_DATA_PH_EN;
		cfg |= OS_QSPI_RW;
		if (addr_max && op->addr.buswidth){ //TBDneed to check with flash size
			qspi->burstmode = MODE_MM;
			cfg |= OS_QSPI_SINGLE_BURST;
		}else{
			qspi->burstmode = MODE_APBR;
			cfg |= OS_QSPI_APB_ACCESS;
		}
	} else {
		custom_cmd |= OS_QSPI_TX_DATA_PH_EN | ~OS_QSPI_RX_DATA_PH_EN;
		qspi->burstmode = MODE_APBW;
		cfg |= OS_QSPI_APB_ACCESS | ~OS_QSPI_RW;
	}

	custom_cmd |= OS_QSPI_OPCODE_PH_EN;
	os_qspi_write(qspi,OS_QSPI_CMD_REG,op->cmd.opcode);

	if (op->addr.nbytes) {
		custom_cmd |= OS_QSPI_ADDR_PH_EN;
		os_qspi_write(qspi,OS_QSPI_ADDR_REG,op->addr.val);
		if(op->addr.nbytes == 3)
			cfg |= OS_QSPI_ADDR_LEN;
		else
			cfg |= ~OS_QSPI_ADDR_LEN;
	}

	if (op->dummy.nbytes){
		custom_cmd |= OS_QSPI_DUMMY_PH_EN;
		os_qspi_write(qspi,OS_QSPI_DUMMY_DLP_REG,(op->dummy.nbytes << 7));
	}

	switch (op->data.buswidth) {
		case SPI_NBITS_QUAD:
			cfg |= OS_QSPI_PROTO_QUAD;
			break;
		case SPI_NBITS_DUAL:
			cfg |= OS_QSPI_PROTO_DUAL;
			break;
		case SPI_NBITS_SINGLE:
			cfg |= OS_QSPI_PROTO_SINGLE;
			break;
	}

	os_qspi_write(qspi,OS_QSPI_CUSTOM_CMD_REG,custom_cmd);
	os_qspi_write(qspi, OS_QSPI_CMD_CFG,cfg);

	err = os_qspi_xfer_mode(qspi, op);

	return err;
}

/**
 * os_qspi_exec_mem_op() - Initiates the QSPI transfer
 * @mem: the SPI memory
 * @op: the memory operation to execute
 * Executes a memory operation.
 * This function first selects the chip and starts the memory operation.
 * Return: 0 in case of success, a negative error code otherwise.
 */
static int os_qspi_exec_mem_op(struct spi_mem *mem,
		const struct spi_mem_op *op)
{
	struct os_qspi *qspi = spi_master_get_devdata(mem->spi->master);
	int ret = 0;

	mutex_lock(&qspi->list_lock);
	ret = os_qspi_xfer_setup(mem, op);
	mutex_unlock(&qspi->list_lock);

	return ret;
}

static const struct spi_controller_mem_ops os_qspi_mem_ops = {
	.exec_op = os_qspi_exec_mem_op,
};

/**
 * os_qspi_probe - Probe method for the QSPI driver
 * @pdev:       Pointer to the platform_device structure
 * This function initializes the driver data structures and the hardware.
 * Return:      0 on success and error value on failure
 */
static int os_qspi_probe(struct platform_device *pdev)
{
	struct os_qspi *qspi;
	struct resource *res;
	int ret, irq;
	struct spi_master *master;

	master = spi_alloc_master(&pdev->dev, sizeof(struct os_qspi));
	if (!master) {
		dev_err(&pdev->dev, "out of memory\n");
		return -ENOMEM;
	}

	qspi = spi_master_get_devdata(master);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	qspi->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(qspi->regs)) {
		ret = PTR_ERR(qspi->regs);
		goto put_master;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "qspi_mm");
        qspi->mm_base = devm_ioremap_resource(&pdev->dev, res);
        if (IS_ERR(qspi->mm_base)) {
                ret = PTR_ERR(qspi->mm_base);
                goto put_master;
        }


	qspi->irq = platform_get_irq(pdev, 0);
	if (qspi->irq < 0) {
		dev_err(&pdev->dev, "Unable to find interrupt\n");
		ret = irq;
		goto put_master;
	}

	ret = devm_request_irq(&pdev->dev, qspi->irq, os_qspi_irq,
			0, dev_name(&pdev->dev), qspi);
	if (ret != 0) {
		ret = -ENXIO;
		dev_err(&pdev->dev, "request_irq failed\n");
		goto put_master;
	}

	init_completion(&qspi->done);

	ret = of_property_read_u32(pdev->dev.of_node, "sifive,fifo-depth",&qspi->fifo_depth);
	if (ret < 0)
		qspi->fifo_depth = OS_QSPI_DEFAULT_DEPTH;

	platform_set_drvdata(pdev, master);

	mutex_init(&qspi->list_lock);

	/* Define our master */
	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = pdev->id;
	master->setup = os_qspi_setup;
	master->num_chipselect = 2;
	master->mode_bits = SPI_MODE_0 | SPI_CS_HIGH | SPI_TX_DUAL | SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD;
	master->mem_ops = &os_qspi_mem_ops;

put_master:
	spi_master_put(master);

	return ret;
}

/**
 * os_qspi_remove:	Remove method for the QSPI driver
 * @pdev:	Pointer to the platform_device structure
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees all resources allocated to
 * the device.
 * Return:	0 Always
 */

static int os_qspi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct os_qspi *qspi = spi_master_get_devdata(master);

	mutex_destroy(&qspi->list_lock);
	//clk_disable_unprepare(qspi->clk); check and do
	spi_master_put(master);

	return 0;
}

static const struct of_device_id os_qspi_of_match[] = {
	{ .compatible = "open-silicon,qspi0", },
	{ /* End of table */ }
};
MODULE_DEVICE_TABLE(of, os_qspi_of_match);

static struct platform_driver os_qspi_driver = {
	.probe = os_qspi_probe,
	.remove = os_qspi_remove,
	.driver = {
		.name = OS_QSPI_DRIVER_NAME,
		.of_match_table = os_qspi_of_match,
	},
};
module_platform_driver(os_qspi_driver);

MODULE_AUTHOR("Open-silicon");
MODULE_DESCRIPTION("Open-Silicon QSPI driver");
MODULE_LICENSE("GPL");
