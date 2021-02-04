// SPDX-License-Identifier: GPL-2.0
/**
 * Driver for the Texas Instruments DS90UB960-Q1 video deserializer
 *
 * Copyright (c) 2019 Luca Ceresoli <luca@lucaceresoli.net>
 * Copyright (c) 2021 Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/i2c-atr.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <linux/regmap.h>
#include <media/v4l2-event.h>

#define EMBED_HACK_MODE 3

#define DS90_FPD_RX_NPORTS	4  /* Physical FPD-link RX ports */
#define DS90_CSI_TX_NPORTS	2  /* Physical CSI-2 TX ports */
#define DS90_NPORTS		(DS90_FPD_RX_NPORTS + DS90_CSI_TX_NPORTS)

#define DS90_NUM_GPIOS		7  /* Physical GPIO pins */
#define DS90_NUM_BC_GPIOS	4  /* Max GPIOs on Back Channel */

#define DS90_NUM_SLAVE_ALIASES	8
#define DS90_MAX_POOL_ALIASES	(DS90_FPD_RX_NPORTS * DS90_NUM_SLAVE_ALIASES)

/* XXX: always use CSI port 0 */
#define HACK_CSI_PORT		0

static inline bool ds90_pad_is_sink(u32 pad)
{
	return pad < 4;
}

static inline bool ds90_pad_is_source(u32 pad)
{
	return pad == 4 || pad == 5;
}

/*
 * Register map
 *
 * 0x00-0x32   Shared (DS90_SR)
 * 0x33-0x3A   CSI-2 TX (per-port paged on DS90UB960, shared on 954) (DS90_TR)
 * 0x4C        Shared (DS90_SR)
 * 0x4D-0x7F   FPD-Link RX, per-port paged (DS90_RR)
 * 0xB0-0xBF   Shared (DS90_SR)
 * 0xD0-0xDF   FPD-Link RX, per-port paged (DS90_RR)
 * 0xF0-0xF5   Shared (DS90_SR)
 * 0xF8-0xFB   Shared (DS90_SR)
 * All others  Reserved
 *
 * Register defines prefixes:
 * DS90_SR_* = Shared register
 * DS90_RR_* = FPD-Link RX, per-port paged register
 * DS90_TR_* = CSI-2 TX, per-port paged register
 * DS90_XR_* = Reserved register
 * DS90_IR_* = Indirect register
 */

#define DS90_SR_I2C_DEV_ID		0x00
#define DS90_SR_RESET			0x01
#define DS90_SR_GEN_CONFIG		0x02
#define DS90_SR_REV_MASK		0x03
#define DS90_SR_DEVICE_STS		0x04
#define DS90_SR_PAR_ERR_THOLD_HI	0x05
#define DS90_SR_PAR_ERR_THOLD_LO	0x06
#define DS90_SR_BCC_WDOG_CTL		0x07
#define DS90_SR_I2C_CTL1		0x08
#define DS90_SR_I2C_CTL2		0x09
#define DS90_SR_SCL_HIGH_TIME		0x0A
#define DS90_SR_SCL_LOW_TIME		0x0B
#define DS90_SR_RX_PORT_CTL		0x0C
#define DS90_SR_IO_CTL			0x0D
#define DS90_SR_GPIO_PIN_STS		0x0E
#define DS90_SR_GPIO_INPUT_CTL		0x0F
#define DS90_SR_GPIO_PIN_CTL(n)		(0x10 + (n)) /* n < DS90_NUM_GPIOS */
#define DS90_SR_FS_CTL			0x18
#define DS90_SR_FS_HIGH_TIME_1		0x19
#define DS90_SR_FS_HIGH_TIME_0		0x1A
#define DS90_SR_FS_LOW_TIME_1		0x1B
#define DS90_SR_FS_LOW_TIME_0		0x1C
#define DS90_SR_MAX_FRM_HI		0x1D
#define DS90_SR_MAX_FRM_LO		0x1E
#define DS90_SR_CSI_PLL_CTL		0x1F

#define DS90_SR_FWD_CTL1		0x20
#define DS90_SR_FWD_CTL1_PORT_DIS(n)		BIT((n)+4)

#define DS90_SR_FWD_CTL2		0x21
#define DS90_SR_FWD_STS			0x22

#define DS90_SR_INTERRUPT_CTL		0x23
#define DS90_SR_INTERRUPT_CTL_INT_EN		BIT(7)
#define DS90_SR_INTERRUPT_CTL_IE_CSI_TX0	BIT(4)
#define DS90_SR_INTERRUPT_CTL_IE_RX(n)		BIT((n)) /* rxport[n] IRQ */
#define DS90_SR_INTERRUPT_CTL_ALL		0x83 // TODO 0x93 to enable CSI

#define DS90_SR_INTERRUPT_STS		0x24
#define DS90_SR_INTERRUPT_STS_INT		BIT(7)
#define DS90_SR_INTERRUPT_STS_IS_CSI_TX0	BIT(4)
#define DS90_SR_INTERRUPT_STS_IS_RX(n)		BIT((n)) /* rxport[n] IRQ */

#define DS90_SR_TS_CONFIG		0x25
#define DS90_SR_TS_CONTROL		0x26
#define DS90_SR_TS_LINE_HI		0x27
#define DS90_SR_TS_LINE_LO		0x28
#define DS90_SR_TS_STATUS		0x29
#define DS90_SR_TIMESTAMP_P0_HI		0x2A
#define DS90_SR_TIMESTAMP_P0_LO		0x2B
#define DS90_SR_TIMESTAMP_P1_HI		0x2C
#define DS90_SR_TIMESTAMP_P1_LO		0x2D

#define DS90_SR_CSI_PORT_SEL		0x32

#define DS90_TR_CSI_CTL			0x33
#define DS90_TR_CSI_CTL_CSI_CAL_EN		BIT(6)
#define DS90_TR_CSI_CTL_CSI_ENABLE		BIT(0)

#define DS90_TR_CSI_CTL2		0x34
#define DS90_TR_CSI_STS			0x35
#define DS90_TR_CSI_TX_ICR		0x36

#define DS90_TR_CSI_TX_ISR		0x37
#define DS90_TR_CSI_TX_ISR_IS_CSI_SYNC_ERROR	BIT(3)
#define DS90_TR_CSI_TX_ISR_IS_CSI_PASS_ERROR	BIT(1)

#define DS90_TR_CSI_TEST_CTL		0x38
#define DS90_TR_CSI_TEST_PATT_HI	0x39
#define DS90_TR_CSI_TEST_PATT_LO	0x3A

#define DS90_XR_AEQ_CTL1		0x42
#define DS90_XR_AEQ_ERR_THOLD		0x43

#define DS90_RR_BCC_ERR_CTL		0x46
#define DS90_RR_BCC_STATUS		0x47

#define DS90_RR_FPD3_CAP		0x4A
#define DS90_RR_RAW_EMBED_DTYPE		0x4B

#define DS90_SR_FPD3_PORT_SEL		0x4C

#define DS90_RR_RX_PORT_STS1		0x4D
#define DS90_RR_RX_PORT_STS1_BCC_CRC_ERROR	BIT(5)
#define DS90_RR_RX_PORT_STS1_LOCK_STS_CHG	BIT(4)
#define DS90_RR_RX_PORT_STS1_BCC_SEQ_ERROR	BIT(3)
#define DS90_RR_RX_PORT_STS1_PARITY_ERROR	BIT(2)
#define DS90_RR_RX_PORT_STS1_PORT_PASS		BIT(1)
#define DS90_RR_RX_PORT_STS1_LOCK_STS		BIT(0)

#define DS90_RR_RX_PORT_STS2		0x4E
#define DS90_RR_RX_PORT_STS2_LINE_LEN_UNSTABLE	BIT(7)
#define DS90_RR_RX_PORT_STS2_LINE_LEN_CHG	BIT(6)
#define DS90_RR_RX_PORT_STS2_FPD3_ENCODE_ERROR	BIT(5)
#define DS90_RR_RX_PORT_STS2_BUFFER_ERROR	BIT(4)
#define DS90_RR_RX_PORT_STS2_CSI_ERROR		BIT(3)
#define DS90_RR_RX_PORT_STS2_FREQ_STABLE	BIT(2)
#define DS90_RR_RX_PORT_STS2_CABLE_FAULT	BIT(1)
#define DS90_RR_RX_PORT_STS2_LINE_CNT_CHG	BIT(0)

#define DS90_RR_RX_FREQ_HIGH		0x4F
#define DS90_RR_RX_FREQ_LOW		0x50
#define DS90_RR_SENSOR_STS_0		0x51
#define DS90_RR_SENSOR_STS_1		0x52
#define DS90_RR_SENSOR_STS_2		0x53
#define DS90_RR_SENSOR_STS_3		0x54
#define DS90_RR_RX_PAR_ERR_HI		0x55
#define DS90_RR_RX_PAR_ERR_LO		0x56
#define DS90_RR_BIST_ERR_COUNT		0x57

#define DS90_RR_BCC_CONFIG		0x58
#define DS90_RR_BCC_CONFIG_I2C_PASS_THROUGH	BIT(6)

#define DS90_RR_DATAPATH_CTL1		0x59
#define DS90_RR_DATAPATH_CTL2		0x5A
#define DS90_RR_SER_ID			0x5B
#define DS90_RR_SER_ALIAS_ID		0x5C

/* For these two register sets: n < DS90_NUM_SLAVE_ALIASES */
#define DS90_RR_SLAVE_ID(n)		(0x5D + (n))
#define DS90_RR_SLAVE_ALIAS(n)		(0x65 + (n))

#define DS90_RR_PORT_CONFIG		0x6D
#define DS90_RR_BC_GPIO_CTL(n)		(0x6E + (n)) /* n < 2 */
#define DS90_RR_RAW10_ID		0x70
#define DS90_RR_RAW12_ID		0x71
#define DS90_RR_CSI_VC_MAP		0x72
#define DS90_RR_LINE_COUNT_HI		0x73
#define DS90_RR_LINE_COUNT_LO		0x74
#define DS90_RR_LINE_LEN_1		0x75
#define DS90_RR_LINE_LEN_0		0x76
#define DS90_RR_FREQ_DET_CTL		0x77
#define DS90_RR_MAILBOX_1		0x78
#define DS90_RR_MAILBOX_2		0x79

#define DS90_RR_CSI_RX_STS		0x7A
#define DS90_RR_CSI_RX_STS_LENGTH_ERR		BIT(3)
#define DS90_RR_CSI_RX_STS_CKSUM_ERR		BIT(2)
#define DS90_RR_CSI_RX_STS_ECC2_ERR		BIT(1)
#define DS90_RR_CSI_RX_STS_ECC1_ERR		BIT(0)

#define DS90_RR_CSI_ERR_COUNTER	0x7B
#define DS90_RR_PORT_CONFIG2		0x7C
#define DS90_RR_PORT_PASS_CTL		0x7D
#define DS90_RR_SEN_INT_RISE_CTL	0x7E
#define DS90_RR_SEN_INT_FALL_CTL	0x7F

#define DS90_XR_REFCLK_FREQ		0xA5

#define DS90_SR_IND_ACC_CTL		0xB0
#define DS90_SR_IND_ACC_CTL_IA_AUTO_INC		BIT(1)

#define DS90_SR_IND_ACC_ADDR		0xB1
#define DS90_SR_IND_ACC_DATA		0xB2
#define DS90_SR_BIST_CONTROL		0xB3
#define DS90_SR_MODE_IDX_STS		0xB8
#define DS90_SR_LINK_ERROR_COUNT	0xB9
#define DS90_SR_FPD3_ENC_CTL		0xBA
#define DS90_SR_FV_MIN_TIME		0xBC
#define DS90_SR_GPIO_PD_CTL		0xBE

#define DS90_RR_PORT_DEBUG		0xD0
#define DS90_RR_AEQ_CTL2		0xD2
#define DS90_RR_AEQ_STATUS		0xD3
#define DS90_RR_AEQ_BYPASS		0xD4
#define DS90_RR_AEQ_MIN_MAX		0xD5
#define DS90_RR_PORT_ICR_HI		0xD8
#define DS90_RR_PORT_ICR_LO		0xD9
#define DS90_RR_PORT_ISR_HI		0xDA
#define DS90_RR_PORT_ISR_LO		0xDB
#define DS90_RR_FC_GPIO_STS		0xDC
#define DS90_RR_FC_GPIO_ICR		0xDD
#define DS90_RR_SEN_INT_RISE_STS	0xDE
#define DS90_RR_SEN_INT_FALL_STS	0xDF

#define DS90_SR_FPD3_RX_ID0		0xF0
#define DS90_SR_FPD3_RX_ID1		0xF1
#define DS90_SR_FPD3_RX_ID2		0xF2
#define DS90_SR_FPD3_RX_ID3		0xF3
#define DS90_SR_FPD3_RX_ID4		0xF4
#define DS90_SR_FPD3_RX_ID5		0xF5
#define DS90_SR_I2C_RX_ID(n)		(0xF8 + (n)) /* < DS90_FPD_RX_NPORTS */

/* DS90_IR_PGEN_*: Indirect Registers for Test Pattern Generator */

#define DS90_IR_PGEN_CTL		0x01
#define DS90_IR_PGEN_CTL_PGEN_ENABLE		BIT(0)

#define DS90_IR_PGEN_CFG		0x02
#define DS90_IR_PGEN_CSI_DI		0x03
#define DS90_IR_PGEN_LINE_SIZE1		0x04
#define DS90_IR_PGEN_LINE_SIZE0		0x05
#define DS90_IR_PGEN_BAR_SIZE1		0x06
#define DS90_IR_PGEN_BAR_SIZE0		0x07
#define DS90_IR_PGEN_ACT_LPF1		0x08
#define DS90_IR_PGEN_ACT_LPF0		0x09
#define DS90_IR_PGEN_TOT_LPF1		0x0A
#define DS90_IR_PGEN_TOT_LPF0		0x0B
#define DS90_IR_PGEN_LINE_PD1		0x0C
#define DS90_IR_PGEN_LINE_PD0		0x0D
#define DS90_IR_PGEN_VBP		0x0E
#define DS90_IR_PGEN_VFP		0x0F
#define DS90_IRT_PGEN_COLOR(n)		(0x10 + (n)) /* n < 15 */

enum ds90_rxport_mode {
	RXPORT_MODE_RAW10,
	RXPORT_MODE_RAW12_HF,
	RXPORT_MODE_RAW12_LF,
	RXPORT_MODE_CSI2,
};

struct ds90_rxport {
	/* Errors and anomalies counters */
	u64 bcc_crc_error_count;
	u64 bcc_seq_error_count;
	u64 line_len_unstable_count;
	u64 line_len_chg_count;
	u64 fpd3_encode_error_count;
	u64 buffer_error_count;
	u64 line_cnt_chg_count;
	u64 csi_rx_sts_length_err_count;
	u64 csi_rx_sts_cksum_err_count;
	u64 csi_rx_sts_ecc2_err_count;
	u64 csi_rx_sts_ecc1_err_count;
	u32 reported_height;
	u32 reported_width;

	struct gpio_chip        gpio_chip;
	char                    gpio_chip_name[64];

	struct i2c_client      *reg_client; /* for per-port local registers, for debugging */

	struct device_node     *remote_of_node; /* "remote-chip" OF node */
	struct i2c_client      *ser_client; /* remote serializer */
	unsigned short          ser_alias; /* ser i2c alias (lower 7 bits) */
	bool                    locked;

	struct ds90_data *priv;
	unsigned short nport; /* RX port number, and index in priv->rxport[] */

	struct v4l2_subdev *sd;		/* Connected subdev */
	struct fwnode_handle *fwnode;

	enum ds90_rxport_mode mode;

	u8 embed_mode;		// EMBED_DTYPE_EN (0 = off, 1-3 - num first long packets to embed)
	u8 embed_datatype;	// EMBED_DTYPE_ID DT for embedded data (e.g. 0x12)
};

struct ds90_asd {
	struct v4l2_async_subdev base;
	struct ds90_rxport *rxport;
};

static inline struct ds90_asd *to_ds90_asd(struct v4l2_async_subdev *asd)
{
	return container_of(asd, struct ds90_asd, base);
}



struct ds90_csitxport {
	u32 data_rate;		/* Nominal data rate (Gb/s) */
	u32 num_data_lanes;
	s64 link_freq[1];
};

struct ds90_data {
	struct i2c_client      *client;  /* for shared local registers */
	struct regmap          *regmap;
	struct gpio_desc       *reset_gpio;
	struct task_struct     *kthread;
	struct i2c_atr         *atr;
	struct ds90_rxport     *rxport[DS90_FPD_RX_NPORTS];
	struct ds90_csitxport   csitxport;

	struct v4l2_subdev          sd;
	struct media_pad            pads[DS90_NPORTS];

	struct v4l2_ctrl_handler    ctrl_handler;
	struct v4l2_async_notifier notifier;

	unsigned long               refclk;

	/* Address Translator alias-to-slave map table */
	size_t       atr_alias_num; /* Number of aliases configured */
	u16          atr_alias_id[DS90_MAX_POOL_ALIASES]; /* 0 = no alias */
	u16          atr_slave_id[DS90_MAX_POOL_ALIASES]; /* 0 = not in use */
	struct mutex alias_table_lock;

	u8 current_read_rxport;
	u8 current_write_rxport_mask;

	u8 current_read_csiport;
	u8 current_write_csiport_mask;

	struct v4l2_subdev_krouting routing;
	struct v4l2_subdev_stream_configs stream_configs;
};

static inline struct ds90_data *sd_to_ds90(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ds90_data, sd);
}

enum {
	TEST_PATTERN_DISABLED = 0,
	TEST_PATTERN_V_COLOR_BARS_1,
	TEST_PATTERN_V_COLOR_BARS_2,
	TEST_PATTERN_V_COLOR_BARS_4,
	TEST_PATTERN_V_COLOR_BARS_8,
};

static const char * const ds90_tpg_qmenu[] = {
	"Disabled",
	"1 vertical color bar",
	"2 vertical color bars",
	"4 vertical color bars",
	"8 vertical color bars",
};

/* -----------------------------------------------------------------------------
 * Basic device access
 */

static int ds90_read(const struct ds90_data *priv, u8 reg, u8 *val)
{
	struct device *dev = &priv->client->dev;
	unsigned int v;
	int ret;

	ret = regmap_read(priv->regmap, reg, &v);
	if (ret) {
		dev_err(dev, "%s: cannot read register 0x%02x (%d)!\n",
				__func__, reg, ret);
		return ret;
	}

	*val = v;

	return 0;
}

static int ds90_write(const struct ds90_data *priv, u8 reg, u8 val)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ret = regmap_write(priv->regmap, reg, val);
	if (ret < 0)
		dev_err(dev, "%s: cannot write register 0x%02x (%d)!\n",
			__func__, reg, ret);

	return ret;
}

static int ds90_update_bits_shared(const struct ds90_data *priv, u8 reg, u8 mask, u8 val)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ret = regmap_update_bits(priv->regmap, reg, mask, val);
	if (ret < 0)
		dev_err(dev, "%s: cannot update register 0x%02x (%d)!\n",
			__func__, reg, ret);

	return ret;
}

static int ds90_rxport_select(struct ds90_data *priv, int nport)
{
	struct device *dev = &priv->client->dev;
	int ret;

	if (priv->current_read_rxport == nport && priv->current_write_rxport_mask == BIT(nport))
		return 0;

	ret = regmap_write(priv->regmap, DS90_SR_FPD3_PORT_SEL, (nport << 4) | (1 << nport));
	if (ret) {
		dev_err(dev, "%s: cannot select rxport %d (%d)!\n",
			__func__, nport, ret);
		return ret;
	}

	priv->current_read_rxport = nport;
	priv->current_write_rxport_mask = BIT(nport);

	return 0;
}

static int ds90_rxport_read(struct ds90_data *priv, int nport, u8 reg, u8 *val)
{
	struct device *dev = &priv->client->dev;
	unsigned int v;
	int ret;

	ds90_rxport_select(priv, nport);

	ret = regmap_read(priv->regmap, reg, &v);
	if (ret) {
		dev_err(dev, "%s: cannot read register 0x%02x (%d)!\n",
			__func__, reg, ret);
		return ret;
	}

	*val = v;

	return 0;
}

static int ds90_rxport_write(struct ds90_data *priv, int nport, u8 reg, u8 val)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ds90_rxport_select(priv, nport);

	ret = regmap_write(priv->regmap, reg, val);
	if (ret)
		dev_err(dev, "%s: cannot write register 0x%02x (%d)!\n",
			__func__, reg, ret);

	return ret;
}

static int ds90_rxport_update_bits(struct ds90_data *priv, int nport,
				   u8 reg, u8 mask, u8 val)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ds90_rxport_select(priv, nport);

	ret = regmap_update_bits(priv->regmap, reg, mask, val);

	if (ret)
		dev_err(dev, "%s: cannot update register 0x%02x (%d)!\n",
			__func__, reg, ret);

	return ret;
}

static int ds90_csiport_select(struct ds90_data *priv, int nport)
{
	struct device *dev = &priv->client->dev;
	int ret;

	if (priv->current_read_csiport == nport && priv->current_write_csiport_mask == BIT(nport))
		return 0;

	ret = regmap_write(priv->regmap, DS90_SR_CSI_PORT_SEL, (nport << 4) | (1 << nport));
	if (ret) {
		dev_err(dev, "%s: cannot select csi port %d (%d)!\n",
			__func__, nport, ret);
		return ret;
	}

	priv->current_read_csiport = nport;
	priv->current_write_csiport_mask = BIT(nport);

	return 0;
}

static int ds90_csiport_read(struct ds90_data *priv, int nport, u8 reg, u8 *val)
{
	struct device *dev = &priv->client->dev;
	unsigned int v;
	int ret;

	ds90_csiport_select(priv, nport);

	ret = regmap_read(priv->regmap, reg, &v);
	if (ret) {
		dev_err(dev, "%s: cannot read register 0x%02x (%d)!\n",
			__func__, reg, ret);
		return ret;
	}

	*val = v;

	return 0;
}

static int ds90_csiport_write(struct ds90_data *priv, int nport, u8 reg, u8 val)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ds90_csiport_select(priv, nport);

	ret = regmap_write(priv->regmap, reg, val);
	if (ret)
		dev_err(dev, "%s: cannot write register 0x%02x (%d)!\n",
			__func__, reg, ret);

	return ret;
}

#if 0
static int ds90_csiport_update_bits(struct ds90_data *priv, int nport,
				   u8 reg, u8 mask, u8 val)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ds90_csiport_select(priv, nport);

	ret = regmap_update_bits(priv->regmap, reg, mask, val);

	if (ret)
		dev_err(dev, "%s: cannot update register 0x%02x (%d)!\n",
			__func__, reg, ret);

	return ret;
}
#endif

static int ds90_write_ind8(const struct ds90_data *priv, u8 reg, u8 val)
{
	int err;

	err = ds90_write(priv, DS90_SR_IND_ACC_ADDR, reg);
	if (!err)
		err = ds90_write(priv, DS90_SR_IND_ACC_DATA, val);
	return err;
}

/* Assumes IA_AUTO_INC is set in DS90_SR_IND_ACC_CTL */
static int ds90_write_ind16(const struct ds90_data *priv, u8 reg, u16 val)
{
	int err;

	err = ds90_write(priv, DS90_SR_IND_ACC_ADDR, reg);
	if (!err)
		err = ds90_write(priv, DS90_SR_IND_ACC_DATA, val >> 8);
	if (!err)
		err = ds90_write(priv, DS90_SR_IND_ACC_DATA, val & 0xff);
	return err;
}

static void ds90_reset(const struct ds90_data *priv, bool keep_reset)
{
	if (!priv->reset_gpio)
		return;

	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	usleep_range(3000, 6000); /* min 2 ms */

	if (!keep_reset) {
		gpiod_set_value_cansleep(priv->reset_gpio, 0);
		usleep_range(2000, 4000); /* min 1 ms */
	}
}

/* -----------------------------------------------------------------------------
 * I2C-ATR (address translator)
 */

static int ds90_atr_attach_client(struct i2c_atr *atr, u32 chan_id,
				  const struct i2c_board_info *info,
				  const struct i2c_client *client,
				  u16 *alias_id)
{
	struct ds90_data *priv = i2c_atr_get_clientdata(atr);
	struct ds90_rxport *rxport = priv->rxport[chan_id];
	struct device *dev = &priv->client->dev;
	u16 alias = 0;
	int reg_idx;
	int pool_idx;
	int err = 0;

	dev_dbg(dev, "rx%d: %s\n", chan_id, __func__);

	mutex_lock(&priv->alias_table_lock);

	/* Find unused alias in table */

	for (pool_idx = 0; pool_idx < priv->atr_alias_num; pool_idx++)
		if (priv->atr_slave_id[pool_idx] == 0)
			break;

	if (pool_idx == priv->atr_alias_num) {
		dev_warn(dev, "rx%d: alias pool exhausted\n", rxport->nport);
		err = -EADDRNOTAVAIL;
		goto out;
	}

	alias = priv->atr_alias_id[pool_idx];

	/* Find first unused alias register */

	for (reg_idx = 0; reg_idx < DS90_NUM_SLAVE_ALIASES; reg_idx++) {
		u8 regval;

		err = ds90_rxport_read(priv, chan_id,
				       DS90_RR_SLAVE_ALIAS(reg_idx), &regval);
		if (!err && regval == 0)
			break;
	}

	if (reg_idx == DS90_NUM_SLAVE_ALIASES) {
		dev_warn(dev, "rx%d: all aliases in use\n", rxport->nport);
		err = -EADDRNOTAVAIL;
		goto out;
	}

	/* Map alias to slave */

	ds90_rxport_write(priv, chan_id,
			  DS90_RR_SLAVE_ID(reg_idx), client->addr << 1);
	ds90_rxport_write(priv, chan_id,
			  DS90_RR_SLAVE_ALIAS(reg_idx), alias << 1);

	priv->atr_slave_id[pool_idx] = client->addr;

	*alias_id = alias; /* tell the atr which alias we chose */

	dev_dbg(dev, "rx%d: client 0x%02x mapped at alias 0x%02x (%s)\n",
		 rxport->nport, client->addr, alias, client->name);

out:
	mutex_unlock(&priv->alias_table_lock);
	return err;
}

static void ds90_atr_detach_client(struct i2c_atr *atr, u32 chan_id,
				   const struct i2c_client *client)
{
	struct ds90_data *priv = i2c_atr_get_clientdata(atr);
	struct ds90_rxport *rxport = priv->rxport[chan_id];
	struct device *dev = &priv->client->dev;
	u16 alias = 0;
	int reg_idx;
	int pool_idx;

	mutex_lock(&priv->alias_table_lock);

	/* Find alias mapped to this client */

	for (pool_idx = 0; pool_idx < priv->atr_alias_num; pool_idx++)
		if (priv->atr_slave_id[pool_idx] == client->addr)
			break;

	if (pool_idx == priv->atr_alias_num) {
		dev_err(dev, "rx%d: client 0x%02x is not mapped!\n",
			rxport->nport, client->addr);
		goto out;
	}

	alias = priv->atr_alias_id[pool_idx];

	/* Find alias register used for this client */

	for (reg_idx = 0; reg_idx < DS90_NUM_SLAVE_ALIASES; reg_idx++) {
		u8 regval;
		int err;

		err = ds90_rxport_read(priv, chan_id,
				       DS90_RR_SLAVE_ALIAS(reg_idx), &regval);
		if (!err && regval == (alias << 1))
			break;
	}

	if (reg_idx == DS90_NUM_SLAVE_ALIASES) {
		dev_err(dev,
			"rx%d: cannot find alias 0x%02x reg (client 0x%02x)!\n",
			rxport->nport, alias, client->addr);
		goto out;
	}

	/* Unmap */

	ds90_rxport_write(priv, chan_id, DS90_RR_SLAVE_ALIAS(reg_idx), 0);
	priv->atr_slave_id[pool_idx] = 0;

	dev_dbg(dev, "rx%d: client 0x%02x unmapped from alias 0x%02x (%s)\n",
		 rxport->nport, client->addr, alias, client->name);

out:
	mutex_unlock(&priv->alias_table_lock);
}

static const struct i2c_atr_ops ds90_atr_ops = {
	.attach_client = ds90_atr_attach_client,
	.detach_client = ds90_atr_detach_client,
};

/* -----------------------------------------------------------------------------
 * CSI ports
 */

static int ds90_csiport_probe_one(struct ds90_data *priv,
				  const struct device_node *np)
{
	struct device *dev = &priv->client->dev;
	struct ds90_csitxport *csitxport = &priv->csitxport;
	int ret;

	if (of_property_read_u32(np, "data-rate", &csitxport->data_rate) != 0) {
		dev_err(dev, "OF: %s: missing \"data-rate\" node\n",
			of_node_full_name(np));
		return -EINVAL;
	}

	if (csitxport->data_rate != 1600000000 &&
	    csitxport->data_rate !=  800000000 &&
	    csitxport->data_rate !=  400000000) {
		dev_err(dev, "OF: %s: invalid \"data-rate\" node\n",
			of_node_full_name(np));
		return -EINVAL;
	}

	csitxport->link_freq[0] = csitxport->data_rate / 2;

	dev_dbg(dev, "Nominal data rate: %u", csitxport->data_rate);

	ret = of_property_count_u32_elems(np, "data-lanes");

	if (ret <= 0) {
		dev_err(dev, "OF: %s: failed to parse data-lanes: %d\n",
		        of_node_full_name(np), ret);
		return ret;
	}

	csitxport->num_data_lanes = ret;

	return 0;
}

static void ds90_csi_handle_events(struct ds90_data *priv)
{
	struct device *dev = &priv->client->dev;
	u8 csi_tx_isr;
	int err;

	err = ds90_csiport_read(priv, HACK_CSI_PORT, DS90_TR_CSI_TX_ISR, &csi_tx_isr);

	if (!err) {
		if (csi_tx_isr & DS90_TR_CSI_TX_ISR_IS_CSI_SYNC_ERROR)
			dev_warn(dev, "CSI_SYNC_ERROR\n");

		if (csi_tx_isr & DS90_TR_CSI_TX_ISR_IS_CSI_PASS_ERROR)
			dev_warn(dev, "CSI_PASS_ERROR\n");
	}
}

/* -----------------------------------------------------------------------------
 * GPIO CHIP: control GPIOs on the remote side
 */

static int ds90_gpio_direction_out(struct gpio_chip *chip,
				   unsigned int offset, int value)
{
	struct ds90_rxport *rxport = gpiochip_get_data(chip);
	unsigned int reg_addr = DS90_RR_BC_GPIO_CTL(offset / 2);
	unsigned int reg_shift = (offset % 2) * 4;

	ds90_rxport_update_bits(rxport->priv, rxport->nport, reg_addr,
				0xf << reg_shift,
				(0x8 + !!value) << reg_shift);
	return 0;
}

static void ds90_gpio_set(struct gpio_chip *chip,
			  unsigned int offset, int value)
{
	ds90_gpio_direction_out(chip, offset, value);
}

static int ds90_gpio_of_xlate(struct gpio_chip *chip,
			      const struct of_phandle_args *gpiospec,
			      u32 *flags)
{
	struct ds90_rxport *rxport = gpiochip_get_data(chip);
	u32 bank = gpiospec->args[0];

	if (bank != rxport->nport)
		return -EINVAL;

	if (flags)
		*flags = gpiospec->args[2];

	return gpiospec->args[1];
}

static int ds90_gpiochip_probe(struct ds90_data *priv, int nport)
{
	struct ds90_rxport *rxport = priv->rxport[nport];
	struct gpio_chip *gc = &rxport->gpio_chip;
	struct device *dev = &priv->client->dev;
	int err;

	scnprintf(rxport->gpio_chip_name, sizeof(rxport->gpio_chip_name),
		  "%s:rx%d", dev_name(dev), nport);

	gc->label               = rxport->gpio_chip_name;
	gc->parent              = dev;
	gc->owner               = THIS_MODULE;
	gc->base                = -1;
	gc->can_sleep           = 1;
	gc->ngpio               = DS90_NUM_BC_GPIOS;
	gc->direction_output    = ds90_gpio_direction_out;
	gc->set                 = ds90_gpio_set;
	gc->of_xlate            = ds90_gpio_of_xlate;
	gc->of_node             = priv->client->dev.of_node;
	gc->of_gpio_n_cells     = 3;

	err = gpiochip_add_data(gc, rxport);
	if (err) {
		dev_err(dev, "rx%d: Failed to add GPIOs: %d\n", nport, err);
		return err;
	}

	return 0;
}

static void ds90_gpiochip_remove(struct ds90_data *priv, int nport)
{
	struct ds90_rxport *rxport = priv->rxport[nport];
	struct gpio_chip *gc = &rxport->gpio_chip;

	gpiochip_remove(gc);
}

/* -----------------------------------------------------------------------------
 * RX ports
 */

static ssize_t locked_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf);
static ssize_t status_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf);

static struct device_attribute dev_attr_locked[] = {
	__ATTR_RO(locked),
	__ATTR_RO(locked),
	__ATTR_RO(locked),
	__ATTR_RO(locked),
};

static struct device_attribute dev_attr_status[] = {
	__ATTR_RO(status),
	__ATTR_RO(status),
	__ATTR_RO(status),
	__ATTR_RO(status),
};

static struct attribute *ds90_rxport0_attrs[] = {
	&dev_attr_locked[0].attr,
	&dev_attr_status[0].attr,
	NULL
};

static struct attribute *ds90_rxport1_attrs[] = {
	&dev_attr_locked[1].attr,
	&dev_attr_status[1].attr,
	NULL
};

static struct attribute *ds90_rxport2_attrs[] = {
	&dev_attr_locked[2].attr,
	&dev_attr_status[2].attr,
	NULL
};

static struct attribute *ds90_rxport3_attrs[] = {
	&dev_attr_locked[3].attr,
	&dev_attr_status[3].attr,
	NULL
};

static ssize_t locked_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ds90_data *priv = sd_to_ds90(sd);
	int nport = (attr - dev_attr_locked);
	const struct ds90_rxport *rxport = priv->rxport[nport];

	return scnprintf(buf, PAGE_SIZE, "%d", rxport->locked);
}

static ssize_t status_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ds90_data *priv = sd_to_ds90(sd);
	int nport = (attr - dev_attr_status);
	const struct ds90_rxport *rxport = priv->rxport[nport];

	return scnprintf(buf, PAGE_SIZE,
			 "bcc_crc_error_count = %llu\n"
			 "bcc_seq_error_count = %llu\n"
			 "line_len_unstable_count = %llu\n"
			 "line_len_chg_count = %llu\n"
			 "fpd3_encode_error_count = %llu\n"
			 "buffer_error_count = %llu\n"
			 "line_cnt_chg_count = %llu\n"
			 "csi_rx_sts_length_err_count = %llu\n"
			 "csi_rx_sts_cksum_err_count = %llu\n"
			 "csi_rx_sts_ecc2_err_count = %llu\n"
			 "csi_rx_sts_ecc1_err_count = %llu\n"
			 "width = %u\n"
			 "height = %u\n",
			 rxport->bcc_crc_error_count,
			 rxport->bcc_seq_error_count,
			 rxport->line_len_unstable_count,
			 rxport->line_len_chg_count,
			 rxport->fpd3_encode_error_count,
			 rxport->buffer_error_count,
			 rxport->line_cnt_chg_count,
			 rxport->csi_rx_sts_length_err_count,
			 rxport->csi_rx_sts_cksum_err_count,
			 rxport->csi_rx_sts_ecc2_err_count,
			 rxport->csi_rx_sts_ecc1_err_count,
			 rxport->reported_width,
			 rxport->reported_height
			 );
}

static struct attribute_group ds90_rxport_attr_group[] = {
	{ .name = "rx0", .attrs = ds90_rxport0_attrs },
	{ .name = "rx1", .attrs = ds90_rxport1_attrs },
	{ .name = "rx2", .attrs = ds90_rxport2_attrs },
	{ .name = "rx3", .attrs = ds90_rxport3_attrs },
};

/*
 * Instantiate serializer and i2c adapter for the just-locked remote
 * end.
 *
 * @note Must be called with priv->alias_table_lock not held! The added i2c
 * adapter will probe new slaves, which can request i2c transfers, ending
 * up in calling ds90_atr_attach_client() where the lock is taken.
 */
static int ds90_rxport_add_serializer(struct ds90_data *priv, int nport)
{
	struct ds90_rxport *rxport = priv->rxport[nport];
	struct device *dev = &priv->client->dev;
	struct i2c_board_info ser_info = { /*.type = "ds90ub953-q1",*/
					   .of_node = rxport->remote_of_node,
					  };
	int err = 0;

	/*
	 * Adding the serializer under rxport->adap would be cleaner,
	 * but it would need tweaks to bypass the alias table. Adding
	 * to the upstream adapter is way simpler.
	 */
	ser_info.addr = rxport->ser_alias;
	rxport->ser_client = i2c_new_client_device(priv->client->adapter, &ser_info);
	if (!rxport->ser_client) {
		dev_err(dev, "rx%d: cannot add %s i2c device",
			nport, ser_info.type);
		return -EIO;
	}

	dev_dbg(dev, "rx%d: remote serializer at alias 0x%02x\n",
		 nport, rxport->ser_client->addr);

	return err;
}

static void ds90_rxport_remove_serializer(struct ds90_data *priv, int nport)
{
	struct ds90_rxport *rxport = priv->rxport[nport];

	if (rxport->ser_client) {
		i2c_unregister_device(rxport->ser_client);
		rxport->ser_client = NULL;
	}
}

/*
 * Return the local alias for a given remote serializer.
 * Get it from devicetree, if absent fallback to the default.
 */
static int
ds90_of_get_reg(struct device_node *np, const char *serializer_name)
{
	u32 alias;
	int ret;
	int idx;

	if (!np)
		return -ENODEV;

	idx = of_property_match_string(np, "reg-names", serializer_name);
	if (idx < 0)
		return idx;

	ret = of_property_read_u32_index(np, "reg", idx, &alias);
	if (ret)
		return ret;

	return alias;
}

static int ds90_rxport_probe_one(struct ds90_data *priv,
				 const struct device_node *np,
				 unsigned int nport)
{
	const char *rxport_names[DS90_FPD_RX_NPORTS] = { "rxport0", "rxport1", "rxport2", "rxport3" };
	const char *ser_names[DS90_FPD_RX_NPORTS] = { "ser0", "ser1", "ser2", "ser3" };
	struct device *dev = &priv->client->dev;
	struct ds90_rxport *rxport;
	int ret;

	if (priv->rxport[nport]) {
		dev_err(dev, "OF: %s: reg value %d is duplicated\n",
			of_node_full_name(np), nport);
		return -EADDRINUSE;
	}

	rxport = kzalloc(sizeof(*rxport), GFP_KERNEL);
	if (!rxport)
		return -ENOMEM;

	priv->rxport[nport] = rxport;

	rxport->nport = nport;
	rxport->priv = priv;
	rxport->mode = RXPORT_MODE_RAW10;

#ifdef EMBED_HACK_MODE
	rxport->embed_mode = EMBED_HACK_MODE;
	rxport->embed_datatype = 0x12; // Embedded 8-bit non Image Data
#endif

	ret = ds90_of_get_reg(priv->client->dev.of_node, ser_names[nport]);
	if (ret < 0)
		goto err_free_rxport;

	rxport->ser_alias = ret;

	rxport->remote_of_node = of_get_child_by_name(np, "remote-chip");
	if (!rxport->remote_of_node) {
		dev_err(dev, "OF: %s: missing remote-chip child\n",
			of_node_full_name(np));
		ret = -EINVAL;
		goto err_free_rxport;
	}

	rxport->fwnode = fwnode_graph_get_remote_endpoint(of_fwnode_handle(np));
	if (!rxport->fwnode) {
		dev_err(dev, "No remote endpoint for rxport%d\n", nport);
		ret = -ENODEV;
		goto err_node_put;
	}


	/* Initialize access to local registers */
	ret = ds90_of_get_reg(priv->client->dev.of_node, rxport_names[nport]);
	if (ret < 0)
		goto err_node_put;

	rxport->reg_client = i2c_new_dummy_device(priv->client->adapter, ret);

	if (IS_ERR(rxport->reg_client)) {
		ret = PTR_ERR(rxport->reg_client);
		goto err_node_put;
	}

	ds90_write(priv, DS90_SR_I2C_RX_ID(nport),
			  rxport->reg_client->addr << 1);

	dev_dbg(dev, "rx%d: at alias 0x%02x\n",
		 nport, rxport->reg_client->addr);


	// Override FREQ_SELECT from the strap
	// FREQ_SELECT: 000: 2.5 Mbps (default for DS90UB913A-Q1 / DS90UB933-Q1 compatibility)
	ds90_rxport_update_bits(priv, nport, DS90_RR_BCC_CONFIG, 0x7, 0);

	switch (rxport->mode) {
	case RXPORT_MODE_RAW10:
		/* FPD3_MODE = RAW10 Mode (DS90UB913A-Q1 / DS90UB933-Q1 compatible) */
		ds90_rxport_update_bits(priv, nport, DS90_RR_PORT_CONFIG, 0x3, 0x3);

		// RAW10_8BIT_CTL = 0b11 : 8-bit processing using lower 8 bits
		// 0b10 : 8-bit processing using upper 8 bits
		ds90_rxport_update_bits(priv, nport, DS90_RR_PORT_CONFIG2, 0x3<<6, 0x2<<6);

		// datatype 0x1e = YUV422 8-bit, VC=nport
		ds90_rxport_write(priv, nport, DS90_RR_RAW10_ID, 0x1e | (nport << 6));

		ds90_rxport_write(priv, nport, DS90_RR_RAW_EMBED_DTYPE,
		                  (rxport->embed_mode << 6) |
		                  rxport->embed_datatype);

		break;

	default:
		BUG();
	}

	// LV_POLARITY & FV_POLARITY
	ds90_rxport_update_bits(priv, nport, DS90_RR_PORT_CONFIG2, 0x3, 0x1);

#if 0
	/*
	 * Changing FREQ_SELECT will result in some errors on the back channel
	 * for a short period of time. Clear the status bits to ignore the errors.
	 */

	msleep(10);

	{
		u8 v1, v2, v3, v4;
		ds90_rxport_read(priv, nport, DS90_RR_BCC_STATUS, &v1);
		ds90_rxport_read(priv, nport, DS90_RR_RX_PORT_STS1, &v2);
		ds90_rxport_read(priv, nport, DS90_RR_RX_PORT_STS2, &v3);
		ds90_rxport_read(priv, nport, DS90_RR_CSI_RX_STS, &v4);
	}


	// XXX sleep a bit, and print the status again
	{
		u8 v1, v2, v3, v4;
		msleep(10);
		ds90_rxport_read(priv, nport, DS90_RR_BCC_STATUS, &v1);
		ds90_rxport_read(priv, nport, DS90_RR_RX_PORT_STS1, &v2);
		ds90_rxport_read(priv, nport, DS90_RR_RX_PORT_STS2, &v3);
		ds90_rxport_read(priv, nport, DS90_RR_CSI_RX_STS, &v4);
		printk("%x, %x, %x, %x\n", v1, v2, v3, v4);
	}
#endif

	ret = ds90_gpiochip_probe(priv, nport);
	if (ret)
		goto err_unreg_i2c_dev;

	/* Enable all interrupt sources from this port */
	ds90_rxport_write(priv, nport, DS90_RR_PORT_ICR_HI, 0x07);
	ds90_rxport_write(priv, nport, DS90_RR_PORT_ICR_LO, 0x7f);

	/* Set pass-through, but preserve BC_FREQ_SELECT strapping options */
	ds90_rxport_update_bits(priv, nport, DS90_RR_BCC_CONFIG,
				DS90_RR_BCC_CONFIG_I2C_PASS_THROUGH, ~0);

	/* Enable I2C communication to the serializer via the alias addr */
	ds90_rxport_write(priv, nport,
			  DS90_RR_SER_ALIAS_ID, rxport->ser_alias << 1);

	dev_dbg(dev, "ser%d: at alias 0x%02x\n",
		 nport, rxport->ser_alias);

	// XXX not sure if we need to delay before accessing the Ser
	// I sometimes get an error when accessing the first reg in Ser
	msleep(10);

	ret = sysfs_create_group(&dev->kobj, &ds90_rxport_attr_group[nport]);
	if (ret) {
		dev_err(dev, "rx%d: failed creating sysfs group", nport);
		goto err_remove_gpiochip;
	}

	ret = i2c_atr_add_adapter(priv->atr, nport);
	if (ret) {
		dev_err(dev, "rx%d: cannot add adapter", nport);
		goto err_remove_sysfs;
	}

	return 0;

err_remove_sysfs:
	sysfs_remove_group(&dev->kobj, &ds90_rxport_attr_group[nport]);
err_remove_gpiochip:
	ds90_gpiochip_remove(priv, nport);
err_unreg_i2c_dev:
	i2c_unregister_device(rxport->reg_client);
err_node_put:
	of_node_put(rxport->remote_of_node);
err_free_rxport:
	priv->rxport[nport] = NULL;
	kfree(rxport);
	return ret;
}

static void ds90_rxport_remove_one(struct ds90_data *priv, int nport)
{
	struct ds90_rxport *rxport = priv->rxport[nport];
	struct device *dev = &priv->client->dev;

	i2c_atr_del_adapter(priv->atr, nport);
	ds90_rxport_remove_serializer(priv, nport);
	ds90_gpiochip_remove(priv, nport);
	i2c_unregister_device(rxport->reg_client);
	sysfs_remove_group(&dev->kobj, &ds90_rxport_attr_group[nport]);
	of_node_put(rxport->remote_of_node);
	kfree(rxport);
}

static int ds90_atr_probe(struct ds90_data *priv)
{
	struct i2c_adapter *parent_adap = priv->client->adapter;
	struct device *dev = &priv->client->dev;

	priv->atr = i2c_atr_new(parent_adap, dev, &ds90_atr_ops,
				DS90_FPD_RX_NPORTS);
	if (IS_ERR(priv->atr))
		return PTR_ERR(priv->atr);

	i2c_atr_set_clientdata(priv->atr, priv);

	return 0;
}

static void ds90_atr_remove(struct ds90_data *priv)
{
	i2c_atr_delete(priv->atr);
	priv->atr = NULL;
}

static void ds90_rxport_handle_events(struct ds90_data *priv, int nport)
{
	struct ds90_rxport *rxport = priv->rxport[nport];
	struct device *dev = &priv->client->dev;
	u8 rx_port_sts1;
	u8 rx_port_sts2;
	u8 csi_rx_sts;
	u8 bcc_sts;
	bool locked;
	int err = 0;

	/* Read interrupts (also clears most of them) */
	if (!err)
		err = ds90_rxport_read(priv, nport, DS90_RR_RX_PORT_STS1, &rx_port_sts1);
	if (!err)
		err = ds90_rxport_read(priv, nport, DS90_RR_RX_PORT_STS2, &rx_port_sts2);
	if (!err)
		err = ds90_rxport_read(priv, nport, DS90_RR_CSI_RX_STS, &csi_rx_sts);
	if (!err)
		err = ds90_rxport_read(priv, nport, DS90_RR_BCC_STATUS, &bcc_sts);

	if (err)
		return;

	dev_dbg(dev, "Handle RX%d events: STS: %x, %x, %x, BCC %x\n", nport,
	       rx_port_sts1, rx_port_sts2, csi_rx_sts, bcc_sts);

	if (bcc_sts)
		dev_err(dev, "BCC error: %#02x\n", bcc_sts);

	if (rx_port_sts1 & DS90_RR_RX_PORT_STS1_BCC_CRC_ERROR)
		rxport->bcc_crc_error_count++;

	if (rx_port_sts1 & DS90_RR_RX_PORT_STS1_BCC_SEQ_ERROR)
		rxport->bcc_seq_error_count++;

	if (rx_port_sts2 & DS90_RR_RX_PORT_STS2_LINE_LEN_UNSTABLE)
		rxport->line_len_unstable_count++;

	if (rx_port_sts2 & DS90_RR_RX_PORT_STS2_LINE_LEN_CHG) {
		u8 h, l;

		rxport->line_len_chg_count++;

		err = ds90_rxport_read(priv, nport, DS90_RR_LINE_LEN_1, &h);
		err = ds90_rxport_read(priv, nport, DS90_RR_LINE_LEN_0, &l);

		rxport->reported_width = (h << 8) | l;

		dev_dbg(dev, "RX%d: PIXELS %u\n", nport, (h << 8) | l);
	}

	if (rx_port_sts2 & DS90_RR_RX_PORT_STS2_FPD3_ENCODE_ERROR)
		rxport->fpd3_encode_error_count++;

	if (rx_port_sts2 & DS90_RR_RX_PORT_STS2_BUFFER_ERROR)
		rxport->buffer_error_count++;

	if (rx_port_sts2 & DS90_RR_RX_PORT_STS2_LINE_CNT_CHG) {
		u8 h, l;

		rxport->line_cnt_chg_count++;

		err = ds90_rxport_read(priv, nport, DS90_RR_LINE_COUNT_HI, &h);
		err = ds90_rxport_read(priv, nport, DS90_RR_LINE_COUNT_LO, &l);

		rxport->reported_height = (h << 8) | l;
		dev_dbg(dev, "RX%d: LINES %u\n", nport, (h << 8) | l);
	}

	if (csi_rx_sts & DS90_RR_CSI_RX_STS_LENGTH_ERR)
		rxport->csi_rx_sts_length_err_count++;

	if (csi_rx_sts & DS90_RR_CSI_RX_STS_CKSUM_ERR)
		rxport->csi_rx_sts_cksum_err_count++;

	if (csi_rx_sts & DS90_RR_CSI_RX_STS_ECC2_ERR)
		rxport->csi_rx_sts_ecc2_err_count++;

	if (csi_rx_sts & DS90_RR_CSI_RX_STS_ECC1_ERR)
		rxport->csi_rx_sts_ecc1_err_count++;

	/* Update locked status */
	locked = rx_port_sts1 & DS90_RR_RX_PORT_STS1_LOCK_STS;
	if (locked && !rxport->locked) {
		dev_dbg(dev, "rx%d: LOCKED\n", nport);
		/* See note about locking in ds90_rxport_add_serializer()! */
		ds90_rxport_add_serializer(priv, nport);
		sysfs_notify(&dev->kobj,
			     ds90_rxport_attr_group[nport].name, "locked");
	} else if (!locked && rxport->locked) {
		dev_dbg(dev, "rx%d: NOT LOCKED\n", nport);
		ds90_rxport_remove_serializer(priv, nport);
		sysfs_notify(&dev->kobj,
			     ds90_rxport_attr_group[nport].name, "locked");
	}
	rxport->locked = locked;
}

/* -----------------------------------------------------------------------------
 * V4L2
 */

static int ds90_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ds90_data *priv = sd_to_ds90(sd);
	unsigned int csi_ctl = DS90_TR_CSI_CTL_CSI_ENABLE;
	unsigned int speed_select = 3;
	int ret;
	int i;

	if (enable) {
		/* Start all cameras. */
		for (i = 0; i < DS90_FPD_RX_NPORTS; ++i) {
			struct ds90_rxport *rxport = priv->rxport[i];

			if (!rxport)
				continue;

			ret = v4l2_subdev_call(rxport->sd, video, s_stream, 1);
			if (ret) {
				for (; i >= 0; --i) {
					rxport = priv->rxport[i];
					if (!rxport)
						continue;

					v4l2_subdev_call(rxport->sd, video, s_stream, 0);
				}

				return ret;
			}
		}

		switch (priv->csitxport.data_rate) {
		case 1600000000:
			speed_select = 0;
			break;
		case  800000000:
			speed_select = 2;
			break;
		case  400000000:
			speed_select = 3;
			break;
		}

		/*
		 * From the datasheet: "initial CSI Skew-Calibration
		 * sequence [...] should be set when operating at 1.6 Gbps"
		 */
		if (speed_select == 0)
			csi_ctl |= DS90_TR_CSI_CTL_CSI_CAL_EN;

		csi_ctl |= (4 - priv->csitxport.num_data_lanes) << 4;

		ds90_write(priv, DS90_SR_CSI_PLL_CTL, speed_select);
		ds90_csiport_write(priv, HACK_CSI_PORT, DS90_TR_CSI_CTL, csi_ctl);
	} else {
		ds90_csiport_write(priv, HACK_CSI_PORT, DS90_TR_CSI_CTL, 0);

		/* Stop all cameras. */
		for (i = 0; i < DS90_FPD_RX_NPORTS; ++i) {
			struct ds90_rxport *rxport = priv->rxport[i];

			if (!rxport)
				continue;

			v4l2_subdev_call(rxport->sd, video, s_stream, 0);
		}
	}

	return 0;
}

static const struct v4l2_subdev_video_ops ds90_video_ops = {
	.s_stream	= ds90_s_stream,
};

struct ds90_format_info {
	u32 code;
	u32 bpp;
	u8 datatype;
};

static const struct ds90_format_info ds90_formats[] = {
	{ .code = MEDIA_BUS_FMT_YUYV8_1X16, .bpp = 16, .datatype = 0x1e, },
	{ .code = MEDIA_BUS_FMT_UYVY8_1X16, .bpp = 16, .datatype = 0x1e, },
	{ .code = MEDIA_BUS_FMT_VYUY8_1X16, .bpp = 16, .datatype = 0x1e, },
	{ .code = MEDIA_BUS_FMT_YVYU8_1X16, .bpp = 16, .datatype = 0x1e, },

	{ .code = MEDIA_BUS_FMT_METADATA_8, .bpp = 8, .datatype = 0x12, },
	{ .code = MEDIA_BUS_FMT_METADATA_16, .bpp = 16, .datatype = 0x12, },

	/* Legacy */
	{ .code = MEDIA_BUS_FMT_YUYV8_2X8, .bpp = 16, .datatype = 0x1e, },
	{ .code = MEDIA_BUS_FMT_UYVY8_2X8, .bpp = 16, .datatype = 0x1e, },
	{ .code = MEDIA_BUS_FMT_VYUY8_2X8, .bpp = 16, .datatype = 0x1e, },
	{ .code = MEDIA_BUS_FMT_YVYU8_2X8, .bpp = 16, .datatype = 0x1e, },
};

static const struct ds90_format_info *ds90_find_format(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ds90_formats); ++i) {
		if (ds90_formats[i].code == code)
			return &ds90_formats[i];
	}

	return NULL;
}

static int ds90_get_routing(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state,
			    struct v4l2_subdev_krouting *routing)
{
	struct ds90_data *priv = sd_to_ds90(sd);
	struct v4l2_subdev_krouting *src;

	dev_dbg(&priv->client->dev, "ds90_get_routing\n");

	src = &priv->routing;

	return v4l2_subdev_cpy_routing(routing, src);
}

static struct v4l2_mbus_framefmt *
ds90_get_pad_format(struct ds90_data *priv,
			    struct v4l2_subdev_state *state,
			    unsigned int pad, u32 stream, u32 which)
{
	unsigned int i;

	if (WARN_ON(which != V4L2_SUBDEV_FORMAT_ACTIVE && which != V4L2_SUBDEV_FORMAT_TRY))
		return NULL;

	if (which == V4L2_SUBDEV_FORMAT_TRY) {
		if (stream > 0) {
			printk("V4L2_SUBDEV_FORMAT_TRY does not support streams\n");
			return NULL;
		}

		return v4l2_subdev_get_try_format(&priv->sd, state, pad);
	}


	for (i = 0; i < priv->stream_configs.num_configs; ++i) {
		if (priv->stream_configs.configs[i].pad == pad &&
		    priv->stream_configs.configs[i].stream == stream)
			return &priv->stream_configs.configs[i].fmt;
	}

	printk("ds90_get_pad_format: out of range");

	return NULL;
}

static void ds90_init_pad_formats(struct v4l2_subdev *sd)
{
	const struct v4l2_mbus_framefmt format = {
		.width = 640,
		.height = 480,
		.code = MEDIA_BUS_FMT_UYVY8_2X8,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.ycbcr_enc = V4L2_YCBCR_ENC_601,
		.quantization = V4L2_QUANTIZATION_LIM_RANGE,
		.xfer_func = V4L2_XFER_FUNC_SRGB,
	};

	struct ds90_data *priv = sd_to_ds90(sd);
	unsigned int i;

	for (i = 0; i < priv->stream_configs.num_configs; ++i)
		priv->stream_configs.configs[i].fmt = format;
}

static int ds90_set_routing(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state,
			    struct v4l2_subdev_krouting *routing)
{
	struct ds90_data *priv = sd_to_ds90(sd);
	int ret;

	if (WARN_ON(routing->which != V4L2_SUBDEV_FORMAT_ACTIVE))
		return -EINVAL;

	// XXX we can only support up to V4L2_FRAME_DESC_ENTRY_MAX, until frame desc is made
	// dynamically allocated
	if (routing->num_routes >= V4L2_FRAME_DESC_ENTRY_MAX)
		return -EINVAL;

	ret = v4l2_subdev_dup_routing(&priv->routing, routing);
	if (ret)
		return ret;

	ret = v4l2_init_stream_configs(&priv->stream_configs, &priv->routing);
	if (ret)
		return ret;

	ds90_init_pad_formats(sd);

	return 0;
}

static int ds90_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
			      struct v4l2_mbus_frame_desc *fd)
{
	struct ds90_data *priv = sd_to_ds90(sd);
	int i;

	dev_dbg(&priv->client->dev, "ds90_get_frame_desc for pad %d\n", pad);

	if (pad != 4) // XXX first csi port
		return -EINVAL;

	memset(fd, 0, sizeof(*fd));

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	for (i = 0; i < priv->routing.num_routes; ++i) {
		const struct v4l2_subdev_route *route = &priv->routing.routes[i];
		struct v4l2_mbus_framefmt *fmt;
		const struct ds90_format_info *ds90_fmt;

		if (!(route->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
			continue;

		if (route->source_pad != pad)
			continue;

		fmt = ds90_get_pad_format(priv, NULL, pad, route->source_stream, V4L2_SUBDEV_FORMAT_ACTIVE);

		if (!fmt)
			return -EINVAL;

		ds90_fmt = ds90_find_format(fmt->code);
		if (!ds90_fmt) {
			dev_err(&priv->client->dev, "Unable to find ds90 format\n");
			return -EINVAL;
		}

		fd->entry[fd->num_entries].stream = route->source_stream;

		fd->entry[fd->num_entries].flags = V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
		fd->entry[fd->num_entries].length = fmt->width * fmt->height * ds90_fmt->bpp / 8;
		fd->entry[fd->num_entries].pixelcode = fmt->code;

		// use the input RX channel as VC
		fd->entry[fd->num_entries].bus.csi2.vc = route->sink_pad;
		fd->entry[fd->num_entries].bus.csi2.dt = ds90_fmt->datatype;
		fd->num_entries++;

		dev_dbg(&priv->client->dev, "UB FD source %u/%u: %ux%u\n", route->sink_pad, route->sink_stream,
		       fmt->width, fmt->height);
	}

	return 0;
}

static int ds90_get_fmt(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_format *format)
{
	struct ds90_data *priv = sd_to_ds90(sd);
	struct v4l2_mbus_framefmt *fmt;

	//printk("ds90_get_fmt %u/%u\n", format->pad, format->stream);

	fmt = ds90_get_pad_format(priv, state, format->pad, format->stream, format->which);

	if (!fmt) {
		printk("ds90_get_fmt: pad/stream fmt not found\n");
		return -EINVAL;
	}

	format->format = *fmt;

	return 0;
}

static int ds90_set_fmt(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_format *format)
{
	struct ds90_data *priv = sd_to_ds90(sd);
	struct v4l2_mbus_framefmt *fmt;

	//printk("ds90_set_fmt %u/%u\n", format->pad, format->stream);

	fmt = ds90_get_pad_format(priv, state, format->pad, format->stream, format->which);

	if (!fmt) {
		printk("ds90_set_fmt: pad/stream fmt not found\n");
		return -EINVAL;
	}

	*fmt = format->format;

	return 0;
}


static const struct v4l2_subdev_pad_ops ds90_pad_ops = {
	.get_routing	= ds90_get_routing,
	.set_routing	= ds90_set_routing,
	.get_frame_desc	= ds90_get_frame_desc,

	.get_fmt = ds90_get_fmt,
	.set_fmt = ds90_set_fmt,
};

static const struct v4l2_subdev_core_ops ds90_subdev_core_ops = {
	.log_status		= v4l2_ctrl_subdev_log_status,
	.subscribe_event	= v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event	= v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops ds90_subdev_ops = {
	.core		= &ds90_subdev_core_ops,
	.video		= &ds90_video_ops,
	.pad		= &ds90_pad_ops,
};

static bool ds90_has_route(struct media_entity *entity, unsigned int pad0,
			  unsigned int pad1)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct ds90_data *priv = sd_to_ds90(sd);

	return v4l2_subdev_has_route(&priv->routing, pad0, pad1);
}

static const struct media_entity_operations ds90_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.has_route = ds90_has_route
};

static void ds90_set_tpg(struct ds90_data *priv, int tpg_num)
{
	/*
	 * Note: no need to write DS90_REG_IND_ACC_CTL: the only indirect
	 * registers target we use is "CSI-2 Pattern Generator & Timing
	 * Registers", which is the default
	 */

	/*
	 * TPG can only provide a single stream per CSI TX port. If
	 * multiple streams are currently enabled, only the first
	 * one will use the TPG, other streams will be halted.
	 */

	if (tpg_num == 0) {
		/* TPG off, enable forwarding from FPD-3 RX ports */
		ds90_write(priv, DS90_SR_FWD_CTL1, 0x00);

		ds90_write_ind8(priv, DS90_IR_PGEN_CTL, 0x00);
		return;
	} else {
		/* TPG on */
		struct v4l2_mbus_framefmt *fmt;
		u8 vbp, vfp;
		u16 blank_lines;
		u16 width;
		u16 height;

		u16 bytespp = 2; /* For MEDIA_BUS_FMT_UYVY8_1X16 */
		u8 cbars_idx = tpg_num - TEST_PATTERN_V_COLOR_BARS_1;
		u8 num_cbars = 1 << cbars_idx;

		u16 line_size;	/* Line size [bytes] */
		u16 bar_size;	/* cbar size [bytes] */
		u16 act_lpf;	/* active lines/frame */
		u16 tot_lpf;	/* tot lines/frame */
		u16 line_pd;	/* Line period in 10-ns units */

		vbp = 33;
		vfp = 10;
		blank_lines = vbp + vfp + 2; /* total blanking lines */

		fmt = ds90_get_pad_format(priv, NULL, 4, 0, V4L2_SUBDEV_FORMAT_ACTIVE);

		width  = fmt->width;
		height = fmt->height;

		line_size = width * bytespp;
		bar_size = line_size / num_cbars;
		act_lpf = height;
		tot_lpf = act_lpf + blank_lines;
		line_pd = 100000000 / 60 / tot_lpf;

		/* Disable forwarding from FPD-3 RX ports */
		ds90_write(priv, DS90_SR_FWD_CTL1,
			   DS90_SR_FWD_CTL1_PORT_DIS(0) |
				   DS90_SR_FWD_CTL1_PORT_DIS(1));

		/* Access Indirect Pattern Gen */
		ds90_write(priv, DS90_SR_IND_ACC_CTL,
			   DS90_SR_IND_ACC_CTL_IA_AUTO_INC | 0);

		ds90_write_ind8(priv, DS90_IR_PGEN_CTL,
				DS90_IR_PGEN_CTL_PGEN_ENABLE);

		/* YUV422 8bit: 2 bytes/block, CSI-2 data type 0x1e */
		ds90_write_ind8(priv, DS90_IR_PGEN_CFG, cbars_idx << 4 | 0x2);
		ds90_write_ind8(priv, DS90_IR_PGEN_CSI_DI, 0x1e);

		ds90_write_ind16(priv, DS90_IR_PGEN_LINE_SIZE1, line_size);
		ds90_write_ind16(priv, DS90_IR_PGEN_BAR_SIZE1,  bar_size);
		ds90_write_ind16(priv, DS90_IR_PGEN_ACT_LPF1,   act_lpf);
		ds90_write_ind16(priv, DS90_IR_PGEN_TOT_LPF1,   tot_lpf);
		ds90_write_ind16(priv, DS90_IR_PGEN_LINE_PD1,   line_pd);
		ds90_write_ind8(priv,  DS90_IR_PGEN_VBP,        vbp);
		ds90_write_ind8(priv,  DS90_IR_PGEN_VFP,        vfp);
	}
}

static int ds90_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ds90_data *priv = container_of(ctrl->handler, struct ds90_data,
					      ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		ds90_set_tpg(priv, ctrl->val);
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ds90_ctrl_ops = {
	.s_ctrl		= ds90_s_ctrl,
};

/* -----------------------------------------------------------------------------
 * Core
 */

static irqreturn_t ds90_handle_events(int irq, void *arg)
{
	struct ds90_data *priv = arg;
	u8 int_sts;
	int err;
	int i;

	// XXX needs mutex!

	err = ds90_read(priv, DS90_SR_INTERRUPT_STS, &int_sts);

	if (!err && int_sts) {
		u8 fwd_sts;

		dev_dbg(&priv->client->dev, "INTERRUPT_STS %x\n", int_sts);

		ds90_read(priv, DS90_SR_FWD_STS, &fwd_sts);

		dev_dbg(&priv->client->dev, "FWD_STS %#x\n", fwd_sts);

		if (int_sts & DS90_SR_INTERRUPT_STS_IS_CSI_TX0)
			ds90_csi_handle_events(priv);

		for (i = 0; i < DS90_FPD_RX_NPORTS; i++) {
			if (!priv->rxport[i])
				continue;

			if (int_sts & DS90_SR_INTERRUPT_STS_IS_RX(i))
				ds90_rxport_handle_events(priv, i);
		}
	}

	return IRQ_HANDLED;
}

static int ds90_run(void *arg)
{
	struct ds90_data *priv = arg;

	while (1) {
		if (kthread_should_stop())
			break;

		ds90_handle_events(0, priv);

		msleep(1000);
	}

	return 0;
}

static void ds90_remove_ports(struct ds90_data *priv)
{
	int i;

	for (i = 0; i < DS90_FPD_RX_NPORTS; i++)
		if (priv->rxport[i])
			ds90_rxport_remove_one(priv, i);

	/* CSI ports have no _remove_one(). No rollback needed. */
}

static int ds90_parse_dt(struct ds90_data *priv)
{
	struct device_node *np = priv->client->dev.of_node;
	struct device *dev = &priv->client->dev;
	int err = 0;
	int n;

	if (!np) {
		dev_err(dev, "OF: no device tree node!\n");
		return -ENOENT;
	}

	n = of_property_read_variable_u16_array(np, "i2c-alias-pool",
						priv->atr_alias_id,
						2, DS90_MAX_POOL_ALIASES);
	if (n < 0)
		dev_warn(dev,
			 "OF: no i2c-alias-pool, can't access remote I2C slaves");

	priv->atr_alias_num = n;

	dev_dbg(dev, "i2c-alias-pool has %zu aliases", priv->atr_alias_num);

	for (n = 0; n < DS90_NPORTS; ++n) {
		struct device_node *ep_np;

		ep_np = of_graph_get_endpoint_by_regs(np, n, 0);
		if (!ep_np)
			continue;

		if (n < DS90_FPD_RX_NPORTS)
			err = ds90_rxport_probe_one(priv, ep_np, n);
		else
			err = ds90_csiport_probe_one(priv, ep_np);

		of_node_put(ep_np);

		if (err)
			break;
	}

#if 0
	for_each_endpoint_of_node(np, ep_np) {
		struct of_endpoint ep;

		of_graph_parse_endpoint(ep_np, &ep);

		if (ep.port >= DS90_NPORTS) {
			dev_err(dev,
				"OF: %s: missing or invalid reg property\n",
				of_node_full_name(np));
			return -EINVAL;
		}

		if (ep.port < DS90_FPD_RX_NPORTS)
			err = ds90_rxport_probe_one(priv, ep_np, ep.port);
		else
			err = ds90_csiport_probe_one(priv, ep_np);

		if (err) {
			of_node_put(ep_np);
			break;
		}
	}
#endif

	if (err)
		ds90_remove_ports(priv);

	return err;
}



static int ds90_notify_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
	struct ds90_data *priv = sd_to_ds90(notifier->sd);
	struct ds90_rxport *rxport = to_ds90_asd(asd)->rxport;
	struct device *dev = &priv->client->dev;
	unsigned int nport = rxport->nport;
	unsigned int src_pad;
	int ret;
	int i;

	dev_dbg(dev, "Bind %s\n", subdev->name);

	ret = media_entity_get_fwnode_pad(&subdev->entity,
					  rxport->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(dev, "Failed to find pad for %s\n", subdev->name);
		return ret;
	}

	rxport->sd = subdev;
	src_pad = ret;

	ret = media_create_pad_link(&rxport->sd->entity, src_pad,
				    &priv->sd.entity, nport,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(dev, "Unable to link %s:%u -> %s:%u\n",
			rxport->sd->name, src_pad, priv->sd.name, nport);
		return ret;
	}

	dev_dbg(dev, "Bound %s pad: %u on index %u\n", subdev->name, src_pad, nport);

	for (i = 0; i < DS90_FPD_RX_NPORTS; ++i) {
		if (priv->rxport[i] && !priv->rxport[i]->sd) {
			dev_dbg(dev, "Waiting for more subdevs to be bound\n");
			return 0;
		}
	}

	dev_dbg(dev, "All subdevs bound\n");

#if 0
	/*
	 * We can only register v4l2_async_notifiers, which do not provide a
	 * means to register a complete callback. bound_sources allows us to
	 * identify when all remote serializers have completed their probe.
	 */
	if (priv->bound_sources != priv->source_mask)
		return 0;

	/*
	 * All enabled sources have probed and enabled their reverse control
	 * channels:
	 *
	 * - Verify all configuration links are properly detected
	 * - Disable auto-ack as communication on the control channel are now
	 *   stable.
	 */
	ds90_check_config_link(priv, priv->source_mask);

	/*
	 * Re-configure I2C with local acknowledge disabled after cameras have
	 * probed.
	 */
	ds90_configure_i2c(priv, false);

	return ds90_set_pixelrate(priv);
#endif
	return 0;
}

static void ds90_notify_unbind(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_subdev *asd)
{
	struct ds90_data *priv = sd_to_ds90(notifier->sd);
	struct ds90_rxport *rxport = to_ds90_asd(asd)->rxport;
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Unbind %s\n", subdev->name);

	rxport->sd = NULL;
}

static const struct v4l2_async_notifier_operations ds90_notify_ops = {
	.bound = ds90_notify_bound,
	.unbind = ds90_notify_unbind,
};

static int ds90_v4l2_notifier_register(struct ds90_data *priv)
{
	struct device *dev = &priv->client->dev;
	int ret;
	int i;

	v4l2_async_notifier_init(&priv->notifier);

	for (i = 0; i < DS90_FPD_RX_NPORTS; ++i) {
		struct ds90_rxport *rxport = priv->rxport[i];
		struct v4l2_async_subdev *asd;
		struct ds90_asd *ds90_asd;

		if (!rxport)
			continue;

		asd = v4l2_async_notifier_add_fwnode_subdev(&priv->notifier,
							    rxport->fwnode,
							    sizeof(struct ds90_asd));
		if (IS_ERR(asd)) {
			dev_err(dev, "Failed to add subdev for source %u: %ld",
				i, PTR_ERR(asd));
			v4l2_async_notifier_cleanup(&priv->notifier);
			return PTR_ERR(asd);
		}

		ds90_asd = to_ds90_asd(asd);
		ds90_asd->rxport = rxport;
	}

	priv->notifier.ops = &ds90_notify_ops;

	ret = v4l2_async_subdev_notifier_register(&priv->sd, &priv->notifier);
	if (ret) {
		dev_err(dev, "Failed to register subdev_notifier");
		v4l2_async_notifier_cleanup(&priv->notifier);
		return ret;
	}

	return 0;
}

static void ds90_v4l2_notifier_unregister(struct ds90_data *priv)
{
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Unregister async notif\n");

	v4l2_async_notifier_unregister(&priv->notifier);
	v4l2_async_notifier_cleanup(&priv->notifier);
}

static int ds90_create_subdev(struct ds90_data *priv)
{
	struct device *dev = &priv->client->dev;
	int ret;
	int i;

	v4l2_i2c_subdev_init(&priv->sd, priv->client, &ds90_subdev_ops);
	v4l2_ctrl_handler_init(&priv->ctrl_handler,
			       ARRAY_SIZE(ds90_tpg_qmenu) - 1);
	priv->sd.ctrl_handler = &priv->ctrl_handler;

	v4l2_ctrl_new_std_menu_items(&priv->ctrl_handler, &ds90_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ds90_tpg_qmenu) - 1, 0, 0,
				     ds90_tpg_qmenu);

	v4l2_ctrl_new_int_menu(&priv->ctrl_handler, NULL, V4L2_CID_LINK_FREQ,
	                       ARRAY_SIZE(priv->csitxport.link_freq) - 1, 0,
	                       priv->csitxport.link_freq);

	if (priv->ctrl_handler.error) {
		ret = priv->ctrl_handler.error;
		goto err_free_ctrl;
	}

	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS |
		V4L2_SUBDEV_FL_MULTIPLEXED;
	priv->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	priv->sd.entity.ops = &ds90_entity_ops;

	for (i = 0; i < DS90_NPORTS; i++) {
		priv->pads[i].flags = (i < DS90_FPD_RX_NPORTS) ?
			MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;
	}

	ret = media_entity_pads_init(&priv->sd.entity, DS90_NPORTS, priv->pads);
	if (ret)
		goto err_free_ctrl;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret) {
		dev_err(dev, "v4l2_async_register_subdev error: %d\n", ret);
		goto err_entity_cleanup;
	}

	ret = ds90_v4l2_notifier_register(priv);
	if (ret) {
		dev_err(dev, "v4l2 subdev notifier register failed: %d\n", ret);
		goto err_unreg_subdev;
	}

	return 0;

err_unreg_subdev:
	v4l2_async_unregister_subdev(&priv->sd);
err_entity_cleanup:
	media_entity_cleanup(&priv->sd.entity);
err_free_ctrl:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return ret;
}

static void ds90_destroy_subdev(struct ds90_data *priv)
{
	ds90_v4l2_notifier_unregister(priv);
	v4l2_async_unregister_subdev(&priv->sd);
	v4l2_subdev_free_routing(&priv->routing);

	v4l2_uninit_stream_configs(&priv->stream_configs);

	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
}

static const struct regmap_config ds90_regmap_config = {
	.name = "ds90ub960",

	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0xff,
};

static void ds90_sw_reset(struct ds90_data *priv)
{
	u8 v;
	int i;

	ds90_write(priv, DS90_SR_RESET, BIT(1));

	for (i = 0; i < 10; ++i) {
		ds90_read(priv, DS90_SR_RESET, &v);
		if (v == 0)
			break;
	}
}

static int ds90_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ds90_data *priv;
	struct clk *clk;
	u8 rev_mask;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;

	mutex_init(&priv->alias_table_lock);

	priv->regmap = devm_regmap_init_i2c(client, &ds90_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	/* get reset pin from DT */
	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio)) {
		err = PTR_ERR(priv->reset_gpio);
		if (err != -EPROBE_DEFER)
			dev_err(dev, "Cannot get reset GPIO (%d)", err);
		return err;
	}

	ds90_reset(priv, false);

	clk = clk_get(dev, NULL);
	if (IS_ERR(clk)) {
		err = PTR_ERR(clk);
		if (err != -EPROBE_DEFER)
			dev_err(dev, "Cannot get REFCLK (%d)", err);
		return err;
	}
	priv->refclk = clk_get_rate(clk);
	clk_put(clk);
	dev_dbg(dev, "REFCLK %lu", priv->refclk);

	ds90_sw_reset(priv);

	/* Runtime check register accessibility */
	err = ds90_read(priv, DS90_SR_REV_MASK, &rev_mask);
	if (err) {
		dev_err(dev, "Cannot read first register (%d), abort\n", err);
		goto err_reg_read;
	}

	err = ds90_atr_probe(priv);
	if (err)
		goto err_atr_probe;

	err = ds90_parse_dt(priv);
	if (err)
		goto err_parse_dt;

	/* V4L2 */

	err = ds90_create_subdev(priv);
	if (err)
		goto err_subdev;

	/* By default enable forwarding from all ports */
	{
		/* Enable forwarding to CSI-2 from RX ports 0, 1, 2, 3 */
		const u8 port_enable_mask = BIT(0) | BIT(1) | BIT(2) | BIT(3);
		/* Forward all RX ports to CSI port 0 */
		const u8 port_map_mask = 0;
		ds90_write(priv, DS90_SR_FWD_CTL1, port_map_mask | ((~port_enable_mask) & 0xf) << 4);
	}

	/* Kick off */
	{
		// Check the initial LOCK status before adding the IRQ handler.
		// If we already have a LOCK, we don't seem to get an irq (at least when polling)
		int i;

		dev_dbg(dev, "INITIAL CHECK\n");

		for (i = 0; i < DS90_FPD_RX_NPORTS; i++) {
			u8 rx_port_sts1;

			if (!priv->rxport[i])
				continue;

			err = ds90_rxport_read(priv, i, DS90_RR_RX_PORT_STS1, &rx_port_sts1);
			if (rx_port_sts1 & DS90_RR_RX_PORT_STS1_LOCK_STS) {
				dev_dbg(dev, "rx%d: INITIAL LOCKED\n", i);
				ds90_rxport_add_serializer(priv, i);
				priv->rxport[i]->locked = true;
			}
		}
	}

	if (client->irq) {
		dev_dbg(dev, "using IRQ %d\n", client->irq);

		err = devm_request_threaded_irq(dev, client->irq,
						NULL, ds90_handle_events,
						IRQF_ONESHOT, client->name,
						priv);
		if (err) {
			dev_err(dev, "Cannot enable IRQ (%d)\n", err);
			goto err_irq;
		}

		/* Disable GPIO3 as input */
		ds90_update_bits_shared(priv, DS90_SR_GPIO_INPUT_CTL,
					BIT(3), 0);
		/* Enable GPIO3 as output, active low interrupt */
		ds90_write(priv, DS90_SR_GPIO_PIN_CTL(3), 0xd1);

		ds90_write(priv, DS90_SR_INTERRUPT_CTL,
				  DS90_SR_INTERRUPT_CTL_ALL);
	} else {
		/* No IRQ, fallback to polling */

		priv->kthread = kthread_run(ds90_run, priv, dev_name(dev));
		if (IS_ERR(priv->kthread)) {
			err = PTR_ERR(priv->kthread);
			dev_err(dev, "Cannot create kthread (%d)\n", err);
			goto err_kthread;
		}
		dev_dbg(dev, "using polling mode\n");
	}

	dev_info(dev, "Successfully probed (rev/mask %02x)\n", rev_mask);

	return 0;

err_kthread:
err_irq:
	ds90_destroy_subdev(priv);
err_subdev:
	ds90_remove_ports(priv);
err_parse_dt:
	ds90_atr_remove(priv);
err_atr_probe:
err_reg_read:
	ds90_reset(priv, true);
	mutex_destroy(&priv->alias_table_lock);
	return err;
}

static int ds90_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ds90_data *priv = sd_to_ds90(sd);

	dev_dbg(&client->dev, "Removing\n");

	if (priv->kthread)
		kthread_stop(priv->kthread);
	ds90_destroy_subdev(priv);
	ds90_remove_ports(priv);
	ds90_atr_remove(priv);
	ds90_reset(priv, true);
	mutex_destroy(&priv->alias_table_lock);

	dev_dbg(&client->dev, "Remove done\n");

	return 0;
}

static const struct i2c_device_id ds90_id[] = {
	{ "ds90ub960-q1", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ds90_id);

#ifdef CONFIG_OF
static const struct of_device_id ds90_dt_ids[] = {
	{ .compatible = "ti,ds90ub960-q1", },
	{ }
};
MODULE_DEVICE_TABLE(of, ds90_dt_ids);
#endif

static struct i2c_driver ds90ub960_driver = {
	.probe_new	= ds90_probe,
	.remove		= ds90_remove,
	.id_table	= ds90_id,
	.driver = {
		.name	= "ds90ub960",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ds90_dt_ids),
	},
};

module_i2c_driver(ds90ub960_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Texas Instruments DS90UB960-Q1 CSI-2 dual deserializer driver");
MODULE_AUTHOR("Luca Ceresoli <luca@lucaceresoli.net>");
