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

#define OS_QSPI_CMD_REG				0x00UL
#define OS_QSPI_ADDR_REG			0x04UL
#define OS_QSPI_DUMMY_DLP_REG			0x08UL
#define OS_QSPI_MODE_REG			0x0CUL
#define OS_QSPI_CMD_CFG				0x10UL
#define OS_QSPI_TX_DATA				0x14UL
#define OS_QSPI_RX_DATA				0x18UL
#define OS_QSPI_START_CMD_REG			0x1CUL
#define OS_QSPI_CUSTOM_CMD_REG			0x20UL
#define OS_QSPI_INTR_STAT_REG			0x24UL
#define OS_QSPI_INT_MASK_CLR			0x28UL
#define OS_QSPI_BAUD_RATE_REG			0x2CUL
#define OS_QSPI_BURST_CTRL_REG			0x30UL
#define OS_QSPI_SYS_STATUS_REG			0x34UL

/*
 * QSPI Custom command registers bit Masks
 */
#define OS_QSPI_OPCODE_PH_EN			BIT(0)
#define OS_QSPI_ADDR_PH_EN			BIT(1)
#define OS_QSPI_MODE_BITS_PH_EN			BIT(2)
#define OS_QSPI_DUMMY_PH_EN			BIT(3)
#define OS_QSPI_RX_DATA_PH_EN			BIT(5)
#define OS_QSPI_TX_DATA_PH_EN			BIT(6)

/*
 * QSPI interrupt stat bit Masks
 */
#define OS_QSPI_INT_MASK_INTERRUPT		BIT(0)
#define OS_QSPI_INT_MASK_AXI_RX_FIFO_FULL	BIT(1)
#define OS_QSPI_INT_MASK_AXI_TX_FIFO_EMPTY	BIT(2)
#define OS_QSPI_INT_MASK_APB_RX			BIT(3)
#define OS_QSPI_INT_MASK_APB_TX			BIT(4)
#define OS_QSPI_INT_MASK_BURST			BIT(5)

/*
 * QSPI Configuration Register bit Masks
 */

#define OS_QSPI_PH_SHIFT_BYPASS			BIT(24)
#define OS_QSPI_RX_DLY_HALFCLK			BIT(31)
#define OS_QSPI_RW				BIT(17)
#define OS_QSPI_HOLD				BIT(16)
#define OS_QSPI_WP				BIT(15)
#define OS_QSPI_CPOL				BIT(9)
#define OS_QSPI_ENDI				BIT(8)
#define OS_QSPI_ADDR_32BIT			BIT(0)
#define OS_QSPI_TX_LEN_16			BIT(1)
#define OS_QSPI_TX_LEN_24			BIT(2)
#define OS_QSPI_TX_LEN_32			GENMASK(2, 1)
#define OS_QSPI_RX_LEN_16			BIT(3)
#define OS_QSPI_RX_LEN_24			BIT(4)
#define OS_QSPI_RX_LEN_32			GENMASK(4, 3)
#define OS_QSPI_DEVICE_0			BIT(6)
#define OS_QSPI_PROTO_DUAL			BIT(10)
#define OS_QSPI_PROTO_QUAD			BIT(11)
#define OS_QSPI_PROTO_QUAD_DDR			GENMASK(11, 10)
#define OS_QSPI_CMD_SERIAL			BIT(12)
#define OS_QSPI_NONE_SERIAL			BIT(13)
#define OS_QSPI_SPEED_CLR_BITS			GENMASK(13, 12)
#define OS_QSPI_SINGLE_BURST			BIT(19)
#define OS_QSPI_CONT_BURST			GENMASK(19, 18)
#define OS_QSPI_0CLK				(0U << 20)
#define OS_QSPI_0PS				(0U << 22)

/*
 * QSPI baudrate bit
 */
#define OS_QSPI_DIV_1				0
#define OS_QSPI_DIV_2				1
#define OS_QSPI_DIV_4				2
#define OS_QSPI_DIV_8				3
#define OS_QSPI_DIV_16				4
#define OS_QSPI_DIV_32				5
#define OS_QSPI_DIV_64				6
#define OS_QSPI_DIV_128				7

/*
 * QSPI system status register state machine
 */
#define	OS_QSPI_IDLE_STATE_MASK			GENMASK(2, 0)
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

#define OS_QSPI_MAX_MMAP_SZ                     10000000

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
	void __iomem *regs;
	void __iomem *mm_base;
	struct mutex list_lock;
	resource_size_t mm_size;
	u32 irq;
	u32 fifo_depth;
	u32 burstmode;
	u32 clk_rate;
	struct clk *clk;
	struct completion done;
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
	udelay(1);//TODO fix this workaround
	*val = readl_relaxed(addr);
}

static void os_qspi_write_fifo(u32 *val, void __iomem *addr)
{
	writel_relaxed(*val, addr);
}

static int get_baud_div(u32 div)
{
	switch (div) {
	case 1:
		return OS_QSPI_DIV_1;
	case 2:
		return OS_QSPI_DIV_2;
	case 3 ... 4:
		return OS_QSPI_DIV_4;
	case 5 ... 8:
		return OS_QSPI_DIV_8;
	case 9 ... 16:
		return OS_QSPI_DIV_16;
	case 17 ... 32:
		return OS_QSPI_DIV_32;
	case 33 ... 64:
		return OS_QSPI_DIV_64;
	case 65 ... 128:
		return OS_QSPI_DIV_128;
	default:
		return -1;
	}
}

/* Make use of os_qspi_wait when the interrupt support is enabled */
#if 0
static int os_qspi_wait(struct os_qspi *qspi)
{
	reinit_completion(&qspi->done);
	if (!wait_for_completion_timeout(&qspi->done,
					 msecs_to_jiffies(1000)))
		return -ETIMEDOUT;

	return 0;
}
#endif

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
	u32 cfg = 0, div, clk_div;

	if (ctrl->busy)
		return -EBUSY;

	if (!spi->max_speed_hz)
		return -EINVAL;

	div = DIV_ROUND_UP(qspi->clk_rate, spi->max_speed_hz);
	clk_div = get_baud_div(div);
	if (clk_div < 0)
		return -EINVAL;

	cfg = os_qspi_read(qspi, OS_QSPI_CMD_CFG);
	cfg |= OS_QSPI_WP | OS_QSPI_HOLD | OS_QSPI_TX_LEN_32 |
	       OS_QSPI_DEVICE_0 | OS_QSPI_RX_LEN_32;
	cfg &= ~(OS_QSPI_PH_SHIFT_BYPASS | OS_QSPI_RX_DLY_HALFCLK |
		 OS_QSPI_ENDI);
	os_qspi_write(qspi, OS_QSPI_CMD_CFG,cfg);

	/* Clear the interrupts */
	os_qspi_write(qspi, OS_QSPI_INT_MASK_CLR, ~0U);
	os_qspi_write(qspi, OS_QSPI_BAUD_RATE_REG, clk_div);

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
	if ((sr & OS_QSPI_INT_MASK_APB_RX) ||
	    (sr & OS_QSPI_INT_MASK_APB_TX)) {
		sr = os_qspi_read(qspi, OS_QSPI_INT_MASK_CLR);
		sr |= 0x01;
		os_qspi_write(qspi, OS_QSPI_INT_MASK_CLR, sr);
	}
	complete(&qspi->done);
	return IRQ_HANDLED;
}

static int os_qspi_mmap_read(struct os_qspi *qspi, const struct spi_mem_op *op)
{
	u32 tmp, sr;

	if (op->data.dir == SPI_MEM_DATA_IN) {
		memcpy_fromio(op->data.buf.in, qspi->mm_base + op->addr.val,
			      op->data.nbytes);
	} else {
		memcpy_toio(qspi->mm_base + op->addr.val, op->data.buf.out,
			    op->data.nbytes);
		/* TODO make use of os_qspi_wait() instead of manual polling */
		if (readl_relaxed_poll_timeout_atomic(qspi->regs +
						      OS_QSPI_INTR_STAT_REG, sr,
						      (sr &
						      OS_QSPI_INT_MASK_BURST),
						      1, 1000))
			printk(KERN_ERR "OS_QSPI: AXI write timeout\n");

		os_qspi_write(qspi, OS_QSPI_BURST_CTRL_REG, 0x01);
		tmp = os_qspi_read(qspi, OS_QSPI_INT_MASK_CLR);
		tmp |= 0x01;
		os_qspi_write(qspi, OS_QSPI_INT_MASK_CLR, tmp);
	}

	return 0;
}

static int os_qspi_wait_idle(struct os_qspi *qspi)
{
	u32 sr;
	int ret;

	ret = readl_relaxed_poll_timeout_atomic(qspi->regs +
						OS_QSPI_SYS_STATUS_REG, sr,
						!(sr &
						OS_QSPI_IDLE_STATE_MASK), 1,
						100);
	if (ret)
		printk(KERN_ERR "OS_QSPI: busy timeout (stat:%#x)\n", sr);

	return ret;
}

static int os_qspi_xfer(struct os_qspi *qspi,const struct spi_mem_op *op)
{
	void (*xfer_fifo)(u32 *val, void __iomem *addr);
	u32 mask;
	u32 *buf;
	int reg;

	if (op->data.dir == SPI_MEM_DATA_IN) {
		xfer_fifo = os_qspi_read_fifo;
		buf = op->data.buf.in;
		reg = OS_QSPI_RX_DATA ;
		mask = OS_QSPI_INT_MASK_APB_RX;
	} else {
		xfer_fifo = os_qspi_write_fifo;
		buf = (u32 *)op->data.buf.out;
		reg = OS_QSPI_TX_DATA ;
		mask = OS_QSPI_INT_MASK_APB_TX;
	}

	if (op->data.dir == SPI_MEM_DATA_IN)
		os_qspi_write(qspi, OS_QSPI_START_CMD_REG, 0x1);

	xfer_fifo(buf++, qspi->regs + reg);

	if (op->data.dir == SPI_MEM_DATA_OUT)
		os_qspi_write(qspi, OS_QSPI_START_CMD_REG, 0x1);

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
	u32 custom_cmd = 0, cfg, addr_max, dummy_clocks;
	int err = 0;

	pr_debug("os-qspi: cmd:%#x mode:%d.%d.%d.%d addr:%#llx len:%#x\n",
		 op->cmd.opcode, op->cmd.buswidth, op->addr.buswidth,
		 op->dummy.buswidth, op->data.buswidth, op->addr.val,
		 op->data.nbytes);

	err = os_qspi_wait_idle(qspi);
	if (err)
		return -EBUSY;

	/* Enable all interrupts */
	os_qspi_write(qspi, OS_QSPI_INT_MASK_CLR, 0x0);

	cfg = os_qspi_read(qspi, OS_QSPI_CMD_CFG);
	os_qspi_write(qspi, OS_QSPI_CUSTOM_CMD_REG, 0);
	os_qspi_write(qspi, OS_QSPI_ADDR_REG, op->addr.val);
	addr_max = op->addr.val + op->data.nbytes + 1;

	if (op->cmd.opcode) {
		custom_cmd |= OS_QSPI_OPCODE_PH_EN;
		os_qspi_write(qspi, OS_QSPI_CMD_REG, op->cmd.opcode);
	}

	if (op->data.dir == SPI_MEM_DATA_IN) {
		cfg |= OS_QSPI_RW;
		if (op->data.nbytes) {
			custom_cmd |= OS_QSPI_RX_DATA_PH_EN;
			custom_cmd &= ~OS_QSPI_TX_DATA_PH_EN;
		}
		if (addr_max < qspi->mm_size && op->addr.buswidth &&
		    op->data.nbytes){
			qspi->burstmode = MODE_MM;
			cfg &= ~OS_QSPI_CONT_BURST;
			cfg |= OS_QSPI_SINGLE_BURST;
			cfg |= OS_QSPI_TX_LEN_32 | OS_QSPI_RX_LEN_32;
		}else{
			qspi->burstmode = MODE_APBR;
			/* configure APB access mode */
			cfg &= ~OS_QSPI_CONT_BURST;
		}
	} else {
		cfg &= ~OS_QSPI_RW;
		if (op->data.nbytes) {
			custom_cmd |= OS_QSPI_TX_DATA_PH_EN;
			custom_cmd &= ~OS_QSPI_RX_DATA_PH_EN;
		}
		if (addr_max < qspi->mm_size && op->addr.buswidth &&
		    op->data.nbytes){
			qspi->burstmode = MODE_MM;
			cfg |= OS_QSPI_CONT_BURST;
			cfg |= OS_QSPI_TX_LEN_32 | OS_QSPI_RX_LEN_32;
		} else {
			qspi->burstmode = MODE_APBW;
			/* configure APB access mode */
			cfg &= ~OS_QSPI_CONT_BURST;
		}
	}

	if (op->addr.nbytes) {
		custom_cmd |= OS_QSPI_ADDR_PH_EN;
		os_qspi_write(qspi, OS_QSPI_ADDR_REG, op->addr.val);
		if(op->addr.nbytes == 3)
			cfg &= ~OS_QSPI_ADDR_32BIT;
		else
			cfg |= OS_QSPI_ADDR_32BIT;
	}

	if (op->dummy.nbytes){
		custom_cmd |= OS_QSPI_DUMMY_PH_EN;
		dummy_clocks = op->dummy.nbytes * 8 / op->dummy.buswidth;
		if (op->data.dtr)
			dummy_clocks /= 2;

		dummy_clocks -= 1;
		dummy_clocks &= 0x0F;
		os_qspi_write(qspi, OS_QSPI_DUMMY_DLP_REG, (dummy_clocks << 8));
	}

	/* Clear cfg op_mode bits */
	cfg &= ~OS_QSPI_PROTO_QUAD_DDR;
	switch (op->data.buswidth) {
	case SPI_NBITS_SINGLE:
		cfg &= ~OS_QSPI_PROTO_QUAD_DDR;
		break;
	case SPI_NBITS_DUAL:
		cfg |= OS_QSPI_PROTO_DUAL;
		break;
	case SPI_NBITS_QUAD:
		cfg |= OS_QSPI_PROTO_QUAD;
		break;
	}

	if (op->data.dtr)
		cfg |= OS_QSPI_PROTO_QUAD_DDR;

	/* Clear cfg speed_mode bits */
	cfg &= ~OS_QSPI_SPEED_CLR_BITS;

	if (op->cmd.buswidth == 1 && op->addr.buswidth == 1)
		cfg &= ~OS_QSPI_SPEED_CLR_BITS;

	if (op->cmd.buswidth == 1 && op->addr.buswidth != 1)
		cfg |= OS_QSPI_CMD_SERIAL;

	if (op->cmd.buswidth != 1)
		cfg |= OS_QSPI_NONE_SERIAL;

	if ((qspi->burstmode == MODE_APBR || qspi->burstmode == MODE_APBW) &&
	    op->data.nbytes) {
		cfg &= ~(OS_QSPI_TX_LEN_32 | OS_QSPI_RX_LEN_32);
		switch (op->data.nbytes) {
		case 1:
			cfg &= ~(OS_QSPI_TX_LEN_32 | OS_QSPI_RX_LEN_32);
			break;
		case 2:
			cfg |= OS_QSPI_TX_LEN_16 | OS_QSPI_RX_LEN_16;
			break;
		case 3:
			cfg |= OS_QSPI_TX_LEN_24 | OS_QSPI_RX_LEN_24;
			break;
		default:
			cfg |= OS_QSPI_TX_LEN_32 | OS_QSPI_RX_LEN_32;
			break;
		}
	}

	trace_printk("YS: cfg=%x cust_cmd=%x\n", cfg, custom_cmd);
	os_qspi_write(qspi, OS_QSPI_CUSTOM_CMD_REG, custom_cmd);
	os_qspi_write(qspi, OS_QSPI_CMD_CFG,cfg);

	if ((qspi->burstmode == MODE_APBR || qspi->burstmode == MODE_APBW) &&
	    !op->data.nbytes)
		os_qspi_write(qspi, OS_QSPI_START_CMD_REG, 0x01);

	err = os_qspi_xfer_mode(qspi, op);

	return err;
}

static int os_qspi_check_buswidth(struct spi_mem *mem, u8 buswidth, bool tx)
{
	u32 mode = mem->spi->mode;

	switch (buswidth) {
	case 1:
		return 0;

	case 2:
		if ((tx && (mode & (SPI_TX_DUAL | SPI_TX_QUAD))) ||
		    (!tx && (mode & (SPI_RX_DUAL | SPI_RX_QUAD))))
			return 0;

		break;

	case 4:
		if ((tx && (mode & SPI_TX_QUAD)) ||
		    (!tx && (mode & SPI_RX_QUAD)))
			return 0;

		break;

	case 8:
		if ((tx && (mode & SPI_TX_OCTAL)) ||
		    (!tx && (mode & SPI_RX_OCTAL)))
			return 0;

		break;

	default:
		break;
	}

	return -ENOTSUPP;
}

bool os_qspi_mem_supports_op(struct spi_mem *mem,
			     const struct spi_mem_op *op)
{
	if (os_qspi_check_buswidth(mem, op->cmd.buswidth, true))
		return false;

	if (op->addr.nbytes &&
	    os_qspi_check_buswidth(mem, op->addr.buswidth, true))
		return false;

	if (op->dummy.nbytes &&
	    os_qspi_check_buswidth(mem, op->dummy.buswidth, true))
		return false;

	if (op->data.dir != SPI_MEM_NO_DATA &&
	    os_qspi_check_buswidth(mem, op->data.buswidth,
				   op->data.dir == SPI_MEM_DATA_OUT))
		return false;

	return true;
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
	.supports_op = os_qspi_mem_supports_op,
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

	qspi->mm_size = resource_size(res);
	if (qspi->mm_size > OS_QSPI_MAX_MMAP_SZ)
		return -EINVAL;

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

	qspi->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(qspi->clk)) {
		ret = PTR_ERR(qspi->clk);
		dev_err(&pdev->dev, "Cannot get clk err:%d\n", ret);
		goto put_master;
	}

	qspi->clk_rate = clk_get_rate(qspi->clk);
	if (!qspi->clk_rate) {
		ret = -EINVAL;
		goto put_master;
	}

	ret = clk_prepare_enable(qspi->clk);
	if (ret) {
		dev_err(&pdev->dev, "Cannot enable the clock\n");
		goto put_master;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "sifive,fifo-depth",
				   &qspi->fifo_depth);
	if (ret < 0)
		qspi->fifo_depth = OS_QSPI_DEFAULT_DEPTH;

	platform_set_drvdata(pdev, master);

	mutex_init(&qspi->list_lock);

	/* Define our master */
	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = pdev->id;
	master->setup = os_qspi_setup;
	master->num_chipselect = 2;
	master->mode_bits = SPI_MODE_0 | SPI_CS_HIGH | SPI_TX_DUAL |
			    SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD;
	master->mem_ops = &os_qspi_mem_ops;

	ret = devm_spi_register_master(&pdev->dev, master);
	if (!ret) {
		dev_dbg(&pdev->dev, "os-qspi driver probed\n");
		return 0;
	}
put_master:
	clk_disable_unprepare(qspi->clk);
	spi_master_put(master);

	return ret;
}

#ifdef CONFIG_PM
static int os_qspi_runtime_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct os_qspi *qspi = spi_master_get_devdata(master);

	clk_disable_unprepare(qspi->clk);

	return 0;
}

static int os_qspi_runtime_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct os_qspi *qspi = spi_master_get_devdata(master);

	return clk_prepare_enable(qspi->clk);
}
#endif

#ifdef CONFIG_PM_SLEEP
static int os_qspi_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	int ret;

	ret = spi_master_suspend(master);
	if (ret)
		return ret;

	return pm_runtime_force_suspend(dev);
}

static int os_qspi_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct os_qspi *qspi = spi_master_get_devdata(master);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;

	ret = spi_master_resume(master);
	if (ret)
		clk_disable_unprepare(qspi->clk);

	return ret;
}
#endif

static const struct dev_pm_ops os_qspi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(os_qspi_suspend, os_qspi_resume)
	SET_RUNTIME_PM_OPS(os_qspi_runtime_suspend,
			   os_qspi_runtime_resume, NULL)
};


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
	clk_disable_unprepare(qspi->clk);
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
		.pm = &os_qspi_pm_ops,
		.of_match_table = os_qspi_of_match,
	},
};
module_platform_driver(os_qspi_driver);

MODULE_AUTHOR("Open-silicon");
MODULE_DESCRIPTION("Open-Silicon QSPI driver");
MODULE_LICENSE("GPL");
