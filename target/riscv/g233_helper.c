#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/getpc.h"
#include "accel/tcg/cpu-ldst.h"

void helper_dma(CPURISCVState *env, target_ulong dst_addr,
                target_ulong src_addr, target_ulong size_code)
{
    uintptr_t ra = GETPC();

    int M, N;
    switch (size_code) {
        case 0: M = N = 8; break;
        case 1: M = N = 16; break;
        case 2: M = N = 32; break;
        default:
            riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, ra);
            return;
    }

    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {
            target_ulong src_elem_addr = src_addr + (i * N + j) * 4;
            uint32_t value = cpu_ldl_data_ra(env, src_elem_addr, ra);
            
            target_ulong dst_elem_addr = dst_addr + (j * M + i) * 4;
            cpu_stl_data_ra(env, dst_elem_addr, value, ra);
        }
    }
}

void helper_sort(CPURISCVState *env,
                 target_ulong array_addr,
                 target_ulong array_size,
                 target_ulong sort_count)
{
    uintptr_t ra = GETPC();
    
    // 参数验证
    if (array_size == 0 || sort_count == 0) {
        return;  // 空数组或不排序，直接返回
    }
    
    // sort_count 不能超过 array_size
    if (sort_count > array_size) {
        sort_count = array_size;
    }
    
    // 冒泡排序算法
    for (target_ulong i = 0; i < sort_count - 1; i++) {
        for (target_ulong j = 0; j < sort_count - 1 - i; j++) {
            // 计算两个相邻元素的地址
            target_ulong addr_j = array_addr + j * sizeof(int32_t);
            target_ulong addr_j1 = array_addr + (j + 1) * sizeof(int32_t);
            
            // 读取两个相邻元素（有符号 32 位整数）
            int32_t val_j = (int32_t)cpu_ldl_data_ra(env, addr_j, ra);
            int32_t val_j1 = (int32_t)cpu_ldl_data_ra(env, addr_j1, ra);
            
            // 如果前面的元素大于后面的，交换
            if (val_j > val_j1) {
                cpu_stl_data_ra(env, addr_j, (uint32_t)val_j1, ra);
                cpu_stl_data_ra(env, addr_j1, (uint32_t)val_j, ra);
            }
        }
    }
}

void helper_crush(CPURISCVState *env,
                  target_ulong dst_addr,
                  target_ulong src_addr,
                  target_ulong src_count)
{
    uintptr_t ra = GETPC();
    
    // 参数验证
    if (src_count == 0) {
        return;  // 空数组，直接返回
    }
    
    // 计算目标数组大小（向上取整）
    target_ulong dst_count = (src_count + 1) / 2;
    
    // 处理每对源元素
    for (target_ulong i = 0; i < dst_count; i++) {
        uint8_t dst_byte = 0;
        
        // 处理第一个元素（低 4 位）
        target_ulong src_idx1 = i * 2;
        if (src_idx1 < src_count) {
            target_ulong src_addr1 = src_addr + src_idx1;
            uint8_t src_byte1 = cpu_ldub_data_ra(env, src_addr1, ra);
            dst_byte = src_byte1 & 0x0F;  // 取低 4 位
        }
        
        // 处理第二个元素（高 4 位）
        target_ulong src_idx2 = i * 2 + 1;
        if (src_idx2 < src_count) {
            target_ulong src_addr2 = src_addr + src_idx2;
            uint8_t src_byte2 = cpu_ldub_data_ra(env, src_addr2, ra);
            dst_byte |= (src_byte2 & 0x0F) << 4;  // 取低 4 位并移到高位
        }
        
        // 写入目标数组
        target_ulong dst_addr_i = dst_addr + i;
        cpu_stb_data_ra(env, dst_addr_i, dst_byte, ra);
    }
}

void helper_expand(CPURISCVState *env, 
                   target_ulong dst_addr, 
                   target_ulong src_addr, 
                   target_ulong num)

{
    uintptr_t ra = GETPC();
    
    // 每个 8bit 元素拆分为 4bit 元素
    for (target_ulong i = 0; i < num; i++) {
        // 读取源数组的一个字节
        uint8_t src_val = cpu_ldub_data_ra(env, src_addr + i, ra);

        // 提取低 4 位
        uint8_t low4 = src_val & 0xF;
        // 提取高 4 位
        uint8_t high4 = (src_val >> 4) & 0xF;
        
        cpu_stb_data_ra(env, dst_addr + (i * 2), low4, ra);
        cpu_stb_data_ra(env, dst_addr + (i * 2 + 1), high4, ra);
    }
}
