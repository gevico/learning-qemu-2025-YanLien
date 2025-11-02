/*
 * QEMU model of the G233 SPI Controller
 *
 * Copyright (c) 2025 Learning QEMU
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 */

#ifndef HW_G233_SPI_H
#define HW_G233_SPI_H

#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qemu/fifo8.h"

#define TYPE_G233_SPI "g233.spi"

/* 寄存器索引 */
enum {
    R_SPI_CR1    = 0,   /* 0x00 - 控制寄存器 1 */
    R_SPI_CR2    = 1,   /* 0x04 - 控制寄存器 2 */
    R_SPI_SR     = 2,   /* 0x08 - 状态寄存器 */
    R_SPI_DR     = 3,   /* 0x0C - 数据寄存器 */
    R_SPI_CSCTRL = 4,   /* 0x10 - CS 控制寄存器 */
    G233_SPI_REG_NUM
};

/* CR1 位定义 */
#define SPI_CR1_SPE     (1 << 6)   /* SPI 使能 */
#define SPI_CR1_MSTR    (1 << 2)   /* 主模式选择 */

/* CR2 位定义 */
#define SPI_CR2_TXEIE   (1 << 7)   /* TXE 中断使能 */
#define SPI_CR2_RXNEIE  (1 << 6)   /* RXNE 中断使能 */
#define SPI_CR2_ERRIE   (1 << 5)   /* 错误中断使能 */
#define SPI_CR2_SSOE    (1 << 4)   /* 软件从设备选择输出使能 */

/* SR 位定义 */
#define SPI_SR_BSY      (1 << 7)   /* 忙标志 */
#define SPI_SR_OVERRUN  (1 << 3)   /* 溢出错误 */
#define SPI_SR_UNDERRUN (1 << 2)   /* 下溢错误 */
#define SPI_SR_TXE      (1 << 1)   /* 发送缓冲区空 */
#define SPI_SR_RXNE     (1 << 0)   /* 接收缓冲区非空 */

/* CSCTRL 位定义 */
#define SPI_CS0_EN      (1 << 0)   /* CS0 使能 */
#define SPI_CS1_EN      (1 << 1)   /* CS1 使能 */
#define SPI_CS2_EN      (1 << 2)   /* CS2 使能 */
#define SPI_CS3_EN      (1 << 3)   /* CS3 使能 */
#define SPI_CS0_ACT     (1 << 4)   /* CS0 激活 */
#define SPI_CS1_ACT     (1 << 5)   /* CS1 激活 */
#define SPI_CS2_ACT     (1 << 6)   /* CS2 激活 */
#define SPI_CS3_ACT     (1 << 7)   /* CS3 激活 */

#define G233_SPI(obj) OBJECT_CHECK(G233SPIState, (obj), TYPE_G233_SPI)

typedef struct G233SPIState G233SPIState;

struct G233SPIState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;

    uint32_t num_cs;
    qemu_irq *cs_lines;

    SSIBus *spi;

    Fifo8 tx_fifo;
    Fifo8 rx_fifo;

    uint32_t regs[G233_SPI_REG_NUM];
};

#endif /* HW_G233_SPI_H */