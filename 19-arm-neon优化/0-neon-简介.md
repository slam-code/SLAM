1,ARM NEON 是适用于ARM Cortex-A系列处理器的一种128位SIMD(Single Instruction, Multiple Data,单指令、多数据)扩展结构。


2,相比于ARMv6或之前的架构，NEON结合了64-bit和128-bit的SIMD指令集，提供128-bit宽的向量运算(vector operations)。NEON技术从ARMv7开始被采用，目前可以在ARM Cortex-A和Cortex-R系列处理器中采用。

 

3,ARM NEON Intrinsics 


NEON指令是从Armv7架构开始引入的SIMD指令，其共有16个128位寄存器。
发展到最新的Arm64架构，其寄存器数量增加到32个，但是其长度仍然为最大128位，
因此操作上并没有发生显著的变化。对于这样的寄存器，因为可以同时存储并处理多组数据，称之为向量寄存器。

Intrinsics是使用C语言的方式对NEON寄存器进行操作，因为相比于传统的使用纯汇编语言，具有可读性强，开发速度快等优势。如果需要在代码中调用NEON Intrinsics函数，需要加入头文件"arm_neon.h"。
