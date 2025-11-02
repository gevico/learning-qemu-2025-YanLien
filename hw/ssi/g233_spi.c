/*
 * QEMU model of the G233 SPI Controller
 *
 * Copyright (c) 2025 Learning QEMU
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qemu/fifo8.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "migration/vmstate.h"
#include "hw/ssi/g233_spi.h"

#define FIFO_CAPACITY   8

static void g233_spi_txfifo_reset(G233SPIState *s)
{
    fifo8_reset(&s->tx_fifo);
    s->regs[R_SPI_SR] |= SPI_SR_TXE;
}

static void g233_spi_rxfifo_reset(G233SPIState *s)
{
    fifo8_reset(&s->rx_fifo);
    s->regs[R_SPI_SR] &= ~SPI_SR_RXNE;
}

static void g233_spi_update_cs(G233SPIState *s)
{
    uint32_t csctrl = s->regs[R_SPI_CSCTRL];
    
    for (int i = 0; i < s->num_cs && i < 4; i++) {
        bool enabled = csctrl & (1 << i);
        bool active = csctrl & (1 << (i + 4));
        qemu_set_irq(s->cs_lines[i], (enabled && active) ? 0 : 1);
    }
}

static void g233_spi_update_irq(G233SPIState *s)
{
    uint32_t cr2 = s->regs[R_SPI_CR2];
    uint32_t sr = s->regs[R_SPI_SR];
    bool irq_level = false;
    
    /* TXE 中断 */
    if ((cr2 & SPI_CR2_TXEIE) && (sr & SPI_SR_TXE)) {
        irq_level = true;
    }
    
    /* RXNE 中断 */
    if ((cr2 & SPI_CR2_RXNEIE) && (sr & SPI_SR_RXNE)) {
        irq_level = true;
    }
    
    /* 错误中断 */
    if ((cr2 & SPI_CR2_ERRIE) && 
        (sr & (SPI_SR_OVERRUN | SPI_SR_UNDERRUN))) {
        irq_level = true;
    }
    
    qemu_set_irq(s->irq, irq_level);
}

static void g233_spi_reset(DeviceState *d)
{
    G233SPIState *s = G233_SPI(d);

    memset(s->regs, 0, sizeof(s->regs));
    
    s->regs[R_SPI_CR1] = 0x00000000;
    s->regs[R_SPI_CR2] = 0x00000000;
    s->regs[R_SPI_SR] = 0x00000002;    /* TXE = 1 */
    s->regs[R_SPI_DR] = 0x0000000C;
    s->regs[R_SPI_CSCTRL] = 0x00000000;

    g233_spi_txfifo_reset(s);
    g233_spi_rxfifo_reset(s);
    g233_spi_update_cs(s);
    g233_spi_update_irq(s);
}

static void g233_spi_flush_txfifo(G233SPIState *s)
{
    uint8_t tx, rx;

    /* 只有 SPI 使能且为主模式时才传输 */
    if (!(s->regs[R_SPI_CR1] & SPI_CR1_SPE) ||
        !(s->regs[R_SPI_CR1] & SPI_CR1_MSTR)) {
        return;
    }
    
    /* 设置忙标志 */
    s->regs[R_SPI_SR] |= SPI_SR_BSY;

    while (!fifo8_is_empty(&s->tx_fifo)) {
        tx = fifo8_pop(&s->tx_fifo);
        rx = ssi_transfer(s->spi, tx);

        /* ✅ 关键修复：检查 OVERRUN 条件 */
        if (s->regs[R_SPI_SR] & SPI_SR_RXNE) {
            /* 
             * RXNE 已经为 1，说明上一个接收的数据还未被读取
             * 现在又收到新数据，触发 OVERRUN
             */
            s->regs[R_SPI_SR] |= SPI_SR_OVERRUN;
            
            /* 丢弃新数据，保留旧数据（这是标准行为）*/
            qemu_log_mask(LOG_GUEST_ERROR,
                          "g233_spi: OVERRUN detected - "
                          "received new data while RXNE is set\n");
        } else {
            /* 正常接收数据 */
            if (!fifo8_is_full(&s->rx_fifo)) {
                fifo8_push(&s->rx_fifo, rx);
                s->regs[R_SPI_SR] |= SPI_SR_RXNE;
            } else {
                /* RX FIFO 真的满了 */
                s->regs[R_SPI_SR] |= SPI_SR_OVERRUN;
                qemu_log_mask(LOG_GUEST_ERROR,
                              "g233_spi: RX FIFO overflow\n");
            }
        }
    }
    
    /* 清除忙标志 */
    s->regs[R_SPI_SR] &= ~SPI_SR_BSY;
    
    /* 更新 TXE 标志 */
    if (fifo8_is_empty(&s->tx_fifo)) {
        s->regs[R_SPI_SR] |= SPI_SR_TXE;
    }
    
    /* 触发中断更新 */
    g233_spi_update_irq(s);
}

static uint64_t g233_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    G233SPIState *s = G233_SPI(opaque);
    uint32_t r = 0;

    if (addr >= (G233_SPI_REG_NUM << 2)) {
        qemu_log_mask(LOG_GUEST_ERROR, 
                      "%s: bad read at address 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        return 0;
    }

    addr >>= 2;
    switch (addr) {
    case R_SPI_CR1:
    case R_SPI_CR2:
    case R_SPI_CSCTRL:
        r = s->regs[addr];
        break;
        
    case R_SPI_SR:
        r = s->regs[R_SPI_SR];
        break;
        
    case R_SPI_DR:
        if (!fifo8_is_empty(&s->rx_fifo)) {
            r = fifo8_pop(&s->rx_fifo);
            
            /* 如果 FIFO 空了，清除 RXNE */
            if (fifo8_is_empty(&s->rx_fifo)) {
                s->regs[R_SPI_SR] &= ~SPI_SR_RXNE;
            }
        } else {
            r = 0;
        }
        break;

    default:
        r = s->regs[addr];
        break;
    }

    g233_spi_update_irq(s);
    return r;
}

static void g233_spi_write(void *opaque, hwaddr addr,
                           uint64_t val64, unsigned int size)
{
    G233SPIState *s = G233_SPI(opaque);
    uint32_t value = val64;

    if (addr >= (G233_SPI_REG_NUM << 2)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write at addr=0x%" HWADDR_PRIx " value=0x%x\n",
                      __func__, addr, value);
        return;
    }

    addr >>= 2;
    switch (addr) {
    case R_SPI_CR1:
        s->regs[R_SPI_CR1] = value & 0x44;
        
        if (!(value & SPI_CR1_SPE)) {
            g233_spi_txfifo_reset(s);
            g233_spi_rxfifo_reset(s);
        }
        break;

    case R_SPI_CR2:
        s->regs[R_SPI_CR2] = value & 0xF0;
        break;

    case R_SPI_SR:
        /* ✅ 写 1 清除错误标志 */
        if (value & SPI_SR_OVERRUN) {
            s->regs[R_SPI_SR] &= ~SPI_SR_OVERRUN;
        }
        if (value & SPI_SR_UNDERRUN) {
            s->regs[R_SPI_SR] &= ~SPI_SR_UNDERRUN;
        }
        /* 其他位只读，不处理 */
        break;

    case R_SPI_DR:
        if (!fifo8_is_full(&s->tx_fifo)) {
            fifo8_push(&s->tx_fifo, (uint8_t)value);
            
            if (fifo8_is_full(&s->tx_fifo)) {
                s->regs[R_SPI_SR] &= ~SPI_SR_TXE;
            }
            
            /* 立即刷新 TX FIFO */
            g233_spi_flush_txfifo(s);
        } else {
            /* TX FIFO 满，设置下溢错误 */
            s->regs[R_SPI_SR] |= SPI_SR_UNDERRUN;
        }
        break;

    case R_SPI_CSCTRL:
        s->regs[R_SPI_CSCTRL] = value & 0xFF;
        g233_spi_update_cs(s);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write to unknown register 0x%" HWADDR_PRIx "\n",
                      __func__, addr << 2);
        break;
    }

    g233_spi_update_irq(s);
}

static const MemoryRegionOps g233_spi_ops = {
    .read = g233_spi_read,
    .write = g233_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void g233_spi_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    G233SPIState *s = G233_SPI(dev);

    s->spi = ssi_create_bus(dev, "spi");
    sysbus_init_irq(sbd, &s->irq);

    if (s->num_cs > 4) {
        s->num_cs = 4;
    }
    s->cs_lines = g_new0(qemu_irq, s->num_cs);
    for (int i = 0; i < s->num_cs; i++) {
        sysbus_init_irq(sbd, &s->cs_lines[i]);
    }

    memory_region_init_io(&s->mmio, OBJECT(s), &g233_spi_ops, s,
                          TYPE_G233_SPI, 0x1000);
    sysbus_init_mmio(sbd, &s->mmio);

    fifo8_create(&s->tx_fifo, FIFO_CAPACITY);
    fifo8_create(&s->rx_fifo, FIFO_CAPACITY);
}

static const VMStateDescription vmstate_g233_spi = {
    .name = TYPE_G233_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, G233SPIState, G233_SPI_REG_NUM),
        VMSTATE_FIFO8(tx_fifo, G233SPIState),
        VMSTATE_FIFO8(rx_fifo, G233SPIState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property g233_spi_properties[] = {
    DEFINE_PROP_UINT32("num-cs", G233SPIState, num_cs, 4),
    // DEFINE_PROP_END_OF_LIST(),
};

static void g233_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = g233_spi_realize;
    dc->vmsd = &vmstate_g233_spi;
    device_class_set_props(dc, g233_spi_properties);
    device_class_set_legacy_reset(dc, g233_spi_reset);
}

static const TypeInfo g233_spi_info = {
    .name           = TYPE_G233_SPI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(G233SPIState),
    .class_init     = g233_spi_class_init,
};

static void g233_spi_register_types(void)
{
    type_register_static(&g233_spi_info);
}

type_init(g233_spi_register_types)
