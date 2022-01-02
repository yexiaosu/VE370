# Introduction to Computer

略

# Assembly Programming

## Operations and Operands

### Levels of Program Code

- High-level language (translator: compiler)
- Assembly language (translator: assembler)
- Machine language

### Processing Different Languages

![image-20211031220909107](.\image-20211031220909107.png)

High-level language -> **complier** -> 
Assembly language -> **assembler** -> 
machine code -> **linker** (link with other related files) -> 
executable machine code -> **formatter** -> 
machine code specific to a target microprocessor

### Assembly Language

Advantages:

- **Compilers** introduce **uncertainty** about execution time and size
- Use when **speed** and size **of** program are critical
- Can mix high-level language with assembly

Drawbacks:

- Can be very time consuming
- No assembler optimization
- Almost impossible to be portable
  - Different computers support different assembly languages that requires different assembler
  - Assembly languages are similar
- Hard to debug

#### Instruction Set (ISA)

- A collection of instructions that a computer understands.
- Different computers have different instruction sets
- Types:
  - Reduced Instruction Set Computer – RISC
  - Complex Instruction Set Computer – CISC

#### Arithmetic Operations

All arithmetic operations have three operands: **two sources** and **one destination**.

- operation destination source1 source2

#### Operands in RISC-V Assembly

##### Register operands

- Arithmetic instructions use register operands
- RISC-V RV32 has a 32 × 32-bit register file
  - x0: the constant value 0
  - x1 (ra): return address
  - x2 (sp): stack pointer
  - x3 (gp): global pointer
  - x4 (tp): thread pointer
  - x5 – x7, x28 – x31: temporaries
  - x8: frame pointer
  - x9, x18 – x27: saved registers
  - x10 – x11: function arguments/results
  - x12 – x17: function arguments

##### Memory operands

- Memory used mainly for composite data (Arrays, structures, dynamic data)
- Steps to use memory operands
  - **Load** values from memory **into registers**
  - Perform arithmetic **operations** with registers
  - **Store** result from register back to memory

![image-20211031231359248](.\image-20211031231359248.png)

- RISC-V memory is Little Endian
  - **Least-significant** byte at **smallest** byte address of a word
  - Big Endian: most-significant byte at smallest address

![image-20211031231437340](.\image-20211031231437340.png)

- Array Address: **Base Address+ Offset**
  - e.g.: Address of A[8] = 0xFFEE0000 + (8 × 4)

##### Registers vs. Memory

- Registers are **faster** to access than memory
- Operating on memory data requires loads and stores (**More instructions**)
- Compiler must **use registers** for variables **as much as possible**

##### Immediate operands (constant)

- e.g.: addi x22, x22, 4

#### Other RISC-V Instructions

##### Logical Operations

- Instructions for bitwise manipulation

![image-20211031232250381](.\image-20211031232250381.png)

- Shift Operations
  - `sll` or `slli`: shift left logical
    - Fill vacated bits with 0 bits
    - `sll` by $i$ bits = multiplies by $2i$
  - `srl` or `srli`: shift right logical
    - Fill vacated bits with 0 bits
    - `srl` by $i$ bits = divides by $2i$ (unsigned only)
  - `srai`: shift right arithmetic
    - Fill vacated bits with **sign bit**
- AND Operations
  - Useful to **mask bits** in a word
    - Select some bits, **clear others to 0**
    - e.g.: `and x0, x10, x11`

![image-20211031232733909](.\image-20211031232733909.png)

- OR Operations
  - Useful to **include bits** in a word
    - **Set some bits to 1** (<u>selected part</u>), leave others unchanged
    - e.g.: `or x9, x10, x11`

![image-20211031232833838](.\image-20211031232833838.png)

- XOR Operations
  - Differencing operation
    - Set some bits to 1, leave others unchanged
    - e.g.: `xor x9,x10,x12 # NOT operation`

![image-20211031232935076](.\image-20211031232935076.png)

##### Conditional Operations

- Each instruction can have a label (optional)
- Branch to a labeled instruction if a condition is true
  - Otherwise, continue sequentially
- `beq`, `bne`, `blt`, `bge`, ...

##### Load upper immediate instruction

- `lui rd, constant`: Copies **20-bit constant** to bits [31:12] of `rd`
- For the occasional 32-bit constant, use `addi` after `lui`.

##### Signed and unsigned variants of instructions

- <u>Bit 31 is sign bit</u>
- Signed comparison: `blt`, `bge`
- Unsigned comparison: `bltu`, `bgeu`
- Sign extension will add sign bits before.

## Function (Procedure) Call

### Stored Program

- Instructions are represented in binary, just like data
- Instructions and data are both stored in memory – stored program

### Program Counter (PC)

- Each instruction is stored as a 32-bit word in program memory
  - has an address
  - when labeled, the label is equal to the address
- PC holds **address of an instruction** to be executed
- PC is a special register in CPU

### Memory Layout

- **Text**: program code
- **Static data**: global/static variables
  - x3 (gp) initialized to the middle of this segment, 0x10008000 allowing ±offset
- **Dynamic data**: heap
- **Stack**: storage for temporary variable in functions
  - x2 (spr) initialized to 0x7ffffffc, growing towards low address

![image-20211101003227789](.\image-20211101003227789.png)

### Function Calling

- Steps for function calling operation

1. **Place parameters** in registers x10 to x17 (function arguments)
2. Transfer control to procedure
3. Acquire storage on stack for procedure
4. **Perform** procedure’s operations
5. **Place result** in register x10 and x11 for caller
6. **Return** to place of call (address in x1)

#### Function Call Instructions

##### Function call: jump and link

`jal x1, ProcedureLabel`

- **x1 <= PC + 4**, x1 is called return address reg.
- **PC <= ProcedureLabel**

##### Function return: jump and link register

`jalr x0, offset(x1)`

- x0 <= PC + 4 (x0≡0, nothing happens)
- **PC <= offset + return address stored in x1**, offset usually is 0 for function return
- Can also be used for computed jumps

#### Uses of Stack in Function Call

![image-20211101003621133](.\image-20211101003621133.png)

#### Leaf Function

- Functions that don’t call other functions.
  - e.g.: (string copy)

![](.\王澜\2021fall\VE370\mid\image-20211101003755255.png)

#### Non-Leaf Functions

- Functions that call other functions.
- For nested call, caller needs to **save on the stack** before calling another function:
  - Its **return address**
  - Any **argument registers**
  - **Temporary registers needed** after the call
- **Restore from the stack** after the call

![](.\image-20211101003953986.png)

# Instruction Encoding

## Representing Instructions

- 6 types (format): 
  - R-type (Register)
  - I-type (Immediate)
  - S-type (Store)
  - U-type (Load upper immediate)
  - B-type (Branch), a.k.a. SB-type
  - J-type (Jump), a.k.a. UJ-type

![image-20211101123650372](.\image-20211101123650372.png)

### R-type

![image-20211101123905452](.\image-20211101123905452.png)

### I-type

![image-20211101124034290](.\image-20211101124034290.png)

![image-20211101124100573](.\image-20211101124100573.png)

### S-type

![image-20211101124152914](.\image-20211101124152914.png)

### B-type

Branch Target address (Target PC) = **Current PC + immediate × 2**

![image-20211101124315205](.\image-20211101124315205.png)

### U-type

For `lui` and `auipc`.

![image-20211101124651893](.\image-20211101124651893.png)

### J-type

For `jal`:

- x1 <= PC + 4, x1 is called return address reg.
- **Target PC <= Current PC + immediate × 2**

![image-20211101124825283](.\image-20211101124825283.png)

## Performance Considerations

- Immediate bits are swirled around. This wil: 
  - Create difficulty for assemblers
  - But save hardware (muxes) on the critical path

# Single Cycle Processor

![image-20211101125429286](.\image-20211101125429286.png)

## ALU Control Unit

![image-20211101125528572](.\image-20211101125528572.png)

![image-20211101125553154](.\image-20211101125553154.png)

## Control Unit

Give different control signals when getting different opcode.

## ImmGen

Generate immediate and **extend it to 32 bits**.

## Performance Related Factors

- Algorithm
  - Determines number of operations executed
- Programming language, compiler, architecture
  - Determine number of machine instructions (lines of source code) executed per operation
- Processor and memory system
  - Determine how fast instructions are executed
- I/O system (including OS)
  - Determines how fast I/O operations are executed

### Measure Computer Performance -- Execution Time

#### Elapsed time – for System Performance

- Total execution time to complete a task, including: Processing, I/O, OS overhead, idle time, everything
- But doesn’t completely reflect computer’s performance if it focuses on better throughput

#### CPU time – for CPU Performance

- CPU execution time processing a task
  - Exclude I/O time, time spent for other tasks
- Comprises user CPU time and system CPU time because they are hard to differentiate
- Different programs run with different CPU performance and system performance

##### CPU Clocking

- Operation of digital hardware governed by a constant-rate clock

![image-20211101130545106](.\image-20211101130545106.png)

- Clock period: **duration** of a clock cycle, e.g., $250$ ps = $0.25$ ns = $250×10^{–12}$​ s
- Clock frequency (rate): **cycles per second**, e.g., $4.0$ GHz = $4000$ MHz = $4.0×10^9$ Hz

##### CPU Time

- Formula:
  - CPU Time = CPU Clock Cycles $\times$ Clock Cycle Time = $\frac{\text{CPU Clock Cycles}}{\text{Clock Rate}}$

- Performance improved by
  - Reducing number of clock cycles
  - Increasing clock rate
  - Hardware designer must often **trade off clock rate against cycle count**

![image-20211101132046877](.\image-20211101132046877.png)

##### Instruction Count and CPI

- Clock Cycles = Instruction Count (IC) $\times$​​ Cycles per Instruction (CPI)
- CPU Time = IC $\times$ CPI $\times$ Clock Cycle Time = $\frac{\text{IC $\times$ CPI}}{\text{Clock Rate}}$​
- Example:

![image-20211101133053931](.\image-20211101133053931.png)

##### Summary

CPU Time = $\frac{\text{Instructions}}{\text{Program}} \times \frac{\text{Clock cycles}}{\text{Instruction}} \times \frac{\text{Seconds}}{\text{Clock cycle}}$

- Algorithm: affects IC, possibly CPI
- Programming language: affects IC, CPI
- Compiler: affects IC, CPI
- Instruction set architecture: affects IC, CPI, Tc (clock period)

#### Clocking Methodology

- Combinational logic does the computation during clock cycles (**between clock edges**).
- Among all kinds of computations, **longest delay determines clock period**

# Pipelined Processor

- **Divide** the combinational logic into smaller pieces, so that each piece is finished in shorter time.
- Five stages, **one step per stage per cycle**:
  - **IF**: Instruction fetch from memory
  - **ID**: Instruction decode & register read
  - **EX**: Execute operation or calculate address
  - **MEM**: Access memory operand
  - **WB**: Write result back to register
- Need **registers between stages** to hold information produced in previous cycle.
- Pipelining improves performance by **increasing instruction throughput**
  - Executes multiple instructions in parallel
  - Each instruction has the same latency

## Pipelined Control

- Control signals derived from instruction
  - Passed along with corresponding instruction
  - Consumed in appropriate stages

![image-20211101163904542](.\image-20211101163904542.png)

## Hazards

- Situations that prevent starting the next instruction in the next cycle
  - **Data hazard**: Need to wait for previous instruction to complete its **data read/write**
  - **Control hazard**: Decision on **control** action **depends on previous instruction**
  - **Structure hazards**: A required resource is busy

### Data Hazards

![image-20211101165457546](.\image-20211101165457546.png)

- If an instruction depends on completion of a data by a previous instruction, then the instruction must be delayed by:
  - **Inserting bubbles or stalls**: Keep adding bubbles until the hazard is resolved
  - **Forwarding**/Bypassing

#### Forwarding

- Use result **as soon as it is available** instead of waiting for it to be stored in a register.
  - Requires extra hardware in the datapath
  - Should always **connect data from a pipeline register** instead connecting input and output directly.
  - Add mux to choose data

![image-20211101165747122](.\image-20211101165747122.png)

##### EX hazard

- Data hazard between two **adjacent** instructions
- Forwarding path from **EX/MEM** pipeline register to **ALU**

##### MEM hazard

- Data hazard between two instructions with **one more instruction in between**
- Forwarding path from **MEM/WB** pipeline register to **ALU**

![image-20211101165918656](.\image-20211101165918656.png)

##### Forwarding Paths

Forwarding paths are created between stage pipeline registers and ALU inputs

![image-20211101170614891](.\image-20211101170614891.png)

##### Detecting the Need to Forward

- ALU operand register numbers in EX stage are given by: ID/EX.RegisterRs1, ID/EX.RegisterRs2
- Data hazards when: 
  - EX: 
    - EX/MEM.RegisterRd = ID/EX.RegisterRs1
    - EX/MEM.RegisterRd = ID/EX.RegisterRs2
  - MEM:
    - MEM/WB.RegisterRd = ID/EX.RegisterRs1
    - MEM/WB.RegisterRd = ID/EX.RegisterRs2
  - And: forwarding instruction will write to a register, i.e.:
    - EX/MEM.RegWrite (EX hazard) or MEM/WB.RegWrite (MEM hazard)
  - And: Rd for that instruction is not x0, i.e.:
    - EX/MEM.RegisterRd ≠ 0 (EX hazard) or MEM/WB.RegisterRd ≠ 0 (MEM hazard)

- Design Forwarding Unit to judge the condition and output select signals (ForwardA, ForwardB) for mux.

##### Double Data Hazard

- Conditions for both hazards are satisfied

  - e.g.: 

  - ```assembly
    add x1 x1 x2
    add x1 x1 x3
    add x1 x1 x4
    ```

- Revise MEM hazard condition that **only forward if EX hazard condition isn’t true**

##### Load-Use Hazard Detection

- load instruction: needs stalls as well as forwarding
  - Indicated by: ID/EX.MemRead
- Load-use hazard when:
  - ID/EX.MemRead and ((ID/EX.RegisterRd = IF/ID.RegisterRs1) or (ID/EX.RegisterRd = IF/ID.RegisterRs2))
- **Hazard Detection unit** is added in ID stage. If detected, **stall and insert bubble**.

![image-20211101172640196](.\image-20211101172640196.png)

- To stall the pipeline:
  1. Force control signals in ID/EX pipeline register to 0’s
  2. Prevent updates of PC and IF/ID registers

![image-20211101172844283](.\image-20211101172844283.png)

