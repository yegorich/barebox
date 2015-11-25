/*
 *  Atheros AR71xx built-in ethernet mac driver
 *
 *  Copyright (C) 2008-2010 Gabor Juhos <juhosg@openwrt.org>
 *  Copyright (C) 2008 Imre Kaloz <kaloz@openwrt.org>
 *
 *  Based on Atheros' AG7100 driver
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include <common.h>
#include <net.h>
#include <dma.h>
#include <init.h>
#include <io.h>
#include <linux/err.h>

#include "ag71xx.h"

#define AG71XX_DEFAULT_MSG_ENABLE	\
	(NETIF_MSG_DRV			\
	| NETIF_MSG_PROBE		\
	| NETIF_MSG_LINK		\
	| NETIF_MSG_TIMER		\
	| NETIF_MSG_IFDOWN		\
	| NETIF_MSG_IFUP		\
	| NETIF_MSG_RX_ERR		\
	| NETIF_MSG_TX_ERR)

static int ag71xx_msg_level = -1;

#define ETH_SWITCH_HEADER_LEN	2

static int ag71xx_probe_dt(struct ag71xx *priv)
{
	struct device_d *dev = priv->dev;
	struct device_node *np = dev->device_node, *child;
	int ret, i = 0;
/*
	ret = of_property_read_u32(np, "slaves", &priv->num_slaves);
	if (ret)
		return ret;

	priv->slaves = xzalloc(sizeof(struct cpsw_slave) * priv->num_slaves);

	for_each_child_of_node(np, child) {
		if (of_device_is_compatible(child, "ti,am3352-cpsw-phy-sel")) {
			ret = cpsw_phy_sel_init(priv, child);
			if (ret)
				return ret;
		}

		if (of_device_is_compatible(child, "ti,davinci_mdio")) {
			ret = of_pinctrl_select_state_default(child);
			if (ret)
				return ret;
		}

		if (i < priv->num_slaves && !strncmp(child->name, "slave", 5)) {
			struct cpsw_slave *slave = &priv->slaves[i];
			uint32_t phy_id[2];

			ret = of_property_read_u32_array(child, "phy_id", phy_id, 2);
			if (ret)
				return ret;

			slave->dev.device_node = child;
			slave->phy_id = phy_id[1];
			slave->phy_if = of_get_phy_mode(child);
			slave->slave_num = i;

			i++;
		}
	}

	for (i = 0; i < priv->num_slaves; i++) {
		struct cpsw_slave *slave = &priv->slaves[i];

		cpsw_gmii_sel_am335x(slave);
	}
*/
	return 0;
}

static inline u32 ag71xx_rr(struct ag71xx *dev, int reg)
{
	return __raw_readl(dev->regs + reg);
}

static inline void ag71xx_wr(struct ag71xx *dev, u32 val, int reg)
{
	__raw_writel(val, dev->regs + reg);
}

static int ag71xx_ether_mii_read(struct mii_bus *miidev, int addr, int reg)
{
	struct ag71xx *etdev = miidev->priv;
	struct device_d *dev = etdev->dev;

	dev_err(dev, "ag71xx_ether_mii_read\n");

	return 0;
}

static int ag71xx_ether_mii_write(struct mii_bus *miidev, int addr, int reg, u16 val)
{
	struct ag71xx *etdev = miidev->priv;
	struct device_d *dev = etdev->dev;

	dev_err(dev, "ag71xx_ether_mii_write\n");

	return 0;
}

static int ag71xx_ether_set_ethaddr(struct eth_device *edev, const unsigned char *adr)
{
	struct ag71xx *etdev = edev->priv;
	struct device_d *dev = etdev->dev;
	dev_err(dev, "ag71xx_ether_set_ethaddr\n");
	return 0;
}

static int ag71xx_ether_get_ethaddr(struct eth_device *edev, unsigned char *adr)
{
	struct ag71xx *etdev = edev->priv;
	struct device_d *dev = etdev->dev;
	dev_err(dev, "ag71xx_ether_get_ethaddr\n");
	/* We have no eeprom */
	return -1;
}

static void ag71xx_ether_halt (struct eth_device *edev)
{
	struct ag71xx *etdev = edev->priv;
	struct device_d *dev = etdev->dev;

	dev_err(dev, "ag71xx_ether_halt\n");

	ag71xx_wr(etdev, AG71XX_REG_RX_CTRL, 0);
	while (ag71xx_rr(etdev, AG71XX_REG_RX_CTRL))
		;
}

static int ag71xx_ether_rx(struct eth_device *edev)
{
	struct ag71xx *etdev = edev->priv;
	struct device_d *dev = etdev->dev;
	ag7240_desc_t *f = etdev->rx_ring;
	int length;

	dev_err(dev, "ag71xx_ether_rx\n");
#if 0
	for (;;) {
		f = etdev->fifo_rx[etdev->next_rx];
		/* TODO */
		/*if (ag7240_rx_owned_by_dma(f)) {
			break;
		}*/

		length = f->pkt_size;

		net_receive(edev, NetRxPackets[etdev->next_rx], length - 4);
		//flush_cache((u32)NetRxPackets[mac->next_rx], PKTSIZE_ALIGN);

		//ag7240_rx_give_to_dma(f);

		if (++etdev->next_rx >= NO_OF_RX_FIFOS) {
			etdev->next_rx = 0;
		}
	}

	if (!(ag71xx_rr(etdev, AG71XX_REG_RX_CTRL))) {
		ag71xx_wr(etdev, AG71XX_REG_RX_DESC, virt_to_phys(f));
		ag71xx_wr(etdev, AG71XX_REG_RX_CTRL, 1);
	}

#endif
	return 0;
}

static int ag71xx_ether_send(struct eth_device *edev, void *packet, int length)
{
	struct ag71xx *etdev = edev->priv;
	struct device_d *dev = etdev->dev;
#if 0
	ag7240_desc_t *f = etdev->fifo_tx[etdev->next_tx];
	int i;

	dev_err(dev, "ag71xx_ether_send\n");

	f->pkt_size = length;
	f->res1 = 0;
	f->pkt_start_addr = virt_to_phys(packet);

	/* TODO */
	//ag7240_tx_give_to_dma(f);
	//flush_cache((u32)packet, length);
	ag71xx_wr(etdev, AG71XX_REG_TX_DESC, virt_to_phys(f));
	ag71xx_wr(etdev, AG71XX_REG_TX_CTRL, TX_CTRL_TXE);

	for (i = 0; i < MAX_WAIT; i++) {
		udelay(10);
		/* TODO */
		/*if (!ag7240_tx_owned_by_dma(f)) {
			break;
		}*/
	}

	if (i == MAX_WAIT) {
		dev_err(dev, "Tx timed out\n");
	}

	f->pkt_start_addr = 0;
	f->pkt_size = 0;

	if (++etdev->next_tx >= NO_OF_TX_FIFOS) {
		etdev->next_tx = 0;
	}

#endif
	return 0;
}

static int ag71xx_ether_open(struct eth_device *edev)
{
	struct ag71xx *etdev = edev->priv;
	struct device_d *dev = etdev->dev;
	dev_err(dev, "ag71xx_ether_open\n");
	return 0;
}

static int ag71xx_ether_init(struct eth_device *edev)
{
	struct ag71xx *etdev = edev->priv;
	struct device_d *dev = etdev->dev;
	void *rx_buffer = etdev->rx_buffer;
	int i;
	unsigned long paddr, val = 0;
	ag7240_desc_t *fr;

	dev_err(dev, "ag71xx_ether_init\n");

	/*if (!ag7240_check_link(mac)) {
		return 0;
	}*/

	/* initialize DMA descriptors */
	etdev->rx_ring = etdev->tx_ring;
	for (i = 0; i < NO_OF_RX_FIFOS; i++) {
		etdev->rx_ring[i].addr = (unsigned long)rx_buffer;
		//etdev->rx_ring[i].size = 0;
		rx_buffer += MAX_RBUFF_SZ;
	}
	etdev->rx_ring[i - 1].addr |= AG71XX_RB_WRAP;

	dev_err(dev, "ag71xx_ether_init: enable RX\n");
	ag71xx_wr(etdev, AG71XX_REG_RX_DESC, etdev->rx_ring);
	ag71xx_wr(etdev, AG71XX_REG_RX_CTRL, RX_CTRL_RXE);
	/* rx start */

	dev_err(dev, "ag71xx_ether_init: done\n");

	return 1;
}

static int ag71xx_probe(struct device_d *dev)
{
	void __iomem *regs;
	struct mii_bus *miibus;
	struct eth_device *edev;
	struct ag71xx *ether_dev;
	u32 rd;

	dev_err(dev, "ag71xx_probe\n");

	regs = dev_request_mem_region(dev, 0);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	dev_err(dev, "ag71xx_probe: regs = %p\n", regs);

	ether_dev = xzalloc(sizeof(struct ag71xx));
	edev = &ether_dev->netdev;
	miibus = &ether_dev->miibus;
	edev->priv = ether_dev;

	edev->init = ag71xx_ether_init;
	edev->open = ag71xx_ether_open;
	edev->send = ag71xx_ether_send;
	edev->recv = ag71xx_ether_rx;
	edev->halt = ag71xx_ether_halt;
	edev->get_ethaddr = ag71xx_ether_get_ethaddr;
	edev->set_ethaddr = ag71xx_ether_set_ethaddr;

	ether_dev->dev = dev;
	ether_dev->regs = regs;

	miibus->read = ag71xx_ether_mii_read;
	miibus->write = ag71xx_ether_mii_write;
	miibus->priv = ether_dev;

#if 0
	/* enable switch core*/
	rd = __raw_readl(base + AR933X_ETHSW_CLOCK_CONTROL_REG) & ~(0x1f);
	rd |= 0x10;
	__raw_writel(base + AR933X_ETHSW_CLOCK_CONTROL_REG, rd);
#endif

        if(ath79_reset_rr(AR933X_RESET_REG_RESET_MODULE) != 0)
	    ath79_reset_wr(AR933X_RESET_REG_RESET_MODULE, 0);

	dev_err(dev, "ag71xx_probe: reset MAC and MDIO\n");

	/* reset GE0 MAC and MDIO */
	rd = ath79_reset_rr(AR933X_RESET_REG_RESET_MODULE);
	rd |= AR933X_RESET_GE0_MAC | AR933X_RESET_GE0_MDIO;
	ath79_reset_wr(AR933X_RESET_REG_RESET_MODULE, rd);
	mdelay(100);

	rd = ath79_reset_rr(AR933X_RESET_REG_RESET_MODULE);
	rd &= ~(AR933X_RESET_GE0_MAC | AR933X_RESET_GE0_MDIO);
	ath79_reset_wr(AR933X_RESET_REG_RESET_MODULE, rd);
	mdelay(100);

	dev_err(dev, "ag71xx_probe: config MAC\n");

	ag71xx_wr(ether_dev, AG71XX_REG_MAC_CFG1, (MAC_CFG1_SR | MAC_CFG1_TX_RST | MAC_CFG1_RX_RST));
	ag71xx_wr(ether_dev, AG71XX_REG_MAC_CFG1, (MAC_CFG1_RXE | MAC_CFG1_TXE));
	ag71xx_wr(ether_dev, AG71XX_REG_MAC_CFG2, (MAC_CFG2_PAD_CRC_EN | MAC_CFG2_LEN_CHECK | MAC_CFG2_IF_10_100));

	dev_err(dev, "ag71xx_probe: config FIFOs\n");

	/* config FIFOs */
	ag71xx_wr(ether_dev, AG71XX_REG_FIFO_CFG0, 0x1f00);
	//ag7240_mii_setup(mac);

	ag71xx_wr(ether_dev, AG71XX_REG_FIFO_CFG1, 0x10ffff);
	ag71xx_wr(ether_dev, AG71XX_REG_FIFO_CFG2, 0xAAA0555);

	ag71xx_wr(ether_dev, AG71XX_REG_FIFO_CFG4, 0x3ffff);
	ag71xx_wr(ether_dev, AG71XX_REG_FIFO_CFG5, 0x66b82);
	ag71xx_wr(ether_dev, AG71XX_REG_FIFO_CFG3, 0x1f00140);


	dev_err(dev, "ag71xx_probe: allocating DMA\n");
	ether_dev->rx_buffer = dma_alloc_coherent(NO_OF_RX_FIFOS * MAX_RBUFF_SZ, DMA_ADDRESS_BROKEN);

	ether_dev->tx_ring = dma_alloc_coherent(NO_OF_RX_FIFOS * sizeof(ag7240_desc_t), DMA_ADDRESS_BROKEN);
	//ag7240_setup_fifos(ag7240_macs[i]);

	dev_err(dev, "ag71xx_probe: register miibus and edev\n");

	mdiobus_register(miibus);
	eth_register(edev);

	return 0;
}

static __maybe_unused struct of_device_id ag71xx_dt_ids[] = {
	{
		.compatible = "qca,ar7100-gmac",
	}, {
		/* sentinel */
	}
};

static struct driver_d ag71xx_driver = {
	.name	= AG71XX_DRV_NAME,
	.probe		= ag71xx_probe,
	.of_compatible = DRV_OF_COMPAT(ag71xx_dt_ids),
};
device_platform_driver(ag71xx_driver);
