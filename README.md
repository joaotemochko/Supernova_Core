# Supernova Core (RV64G+S)

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](...)
[![License](https://img.shields.io/badge/license-MIT-blue)](...)
[![Language](https://img.shields.io/badge/language-SystemVerilog-purple)](...)

The Supernova Core is a 12-stage, 4-wide superscalar, **Out-of-Order (OoO)** RISC-V core designed for maximum single-thread performance. It functions as the "big" core in a cluster, acting as the primary "Master" thread and as a high-performance "worker" for the **STU (Speculative Threading Unit)**.

Supernova is designed to be fully Linux-capable, implementing the **RV64G+S** (IMAFD + Supervisor) instruction set and a full **MMU (TLB/PTW)**.

---

## 🚀 Microarchitecture (Out-of-Order)

Supernova uses an aggressive OoO design to break false dependencies and extract instruction-level parallelism (ILP). Its core components include:

* **Front-End:** A 4-wide Fetch stage (`supernova_fetch`) with a Branch Target Buffer (BTB) for branch prediction.
* **Rename:** A 4-wide Decode & Rename stage (`supernova_decode_rename`) that uses a **Register Alias Table (RAT)** and **Physical Register Files (PRF)** to eliminate WAR/WAW hazards.
* **Execution:** A unified **Reservation Station (RS)** schedules instructions for a pool of execution units as soon as their operands are ready. This includes:
    * `supernova_alu_unit` (Integer ALU)
    * `supernova_mdu_unit` (Multiply/Divide)
    * `supernova_fpu_unit` (Floating Point)
* **Memory:** A **Load/Store Queue (LSQ)** manages memory operations out-of-order, providing store-to-load forwarding and precise exception handling.
* **Back-End:** A **Reorder Buffer (ROB)** ensures instructions are "committed" (retired) in-order, updating the **Architectural Register File (ARF)**.

---

## 🏛️ STU (Speculative Threading Unit) Integration

Supernova is the primary "Master" core for the STU and a high-performance worker.

### Level 1: Conservative Parallelism (Ignored)

Supernova **ignores** Level 1 dispatches. Feeding its 4-wide, 128-entry OoO pipeline with only 2 pre-vetted instructions would "starve" it and severely degrade performance.

### Level 2: Optimistic Speculation (Master & Worker)

Supernova is designed to participate fully in Level 2 "thread forks".
* **As Master:** It acts as the "present" thread (Core 0), and its execution is monitored by the STU.
* **As Worker:**
    * **Context Management:** It exposes its **Architectural Register File (ARF)** (not its PRF) via `core_copy_*` signals for the `stu_context_manager` to copy.
    * **Forking:** When `l2_spec_start_in` is asserted, it triggers a **full pipeline flush** (`global_flush`), checkpoints its ARF, and restarts Fetch at the `l2_spec_pc_in`.
    * **Tracking:** Its LSQ (`supernova_lsq`) reports all **Physical Addresses (PAs)** (post-MMU) to the `stu_memory_tracker`.
    * **Status Reporting:** The `supernova_commit` stage reports exceptions (e.g., Page Faults from the LSQ/MMU) to the `stu_validator` via `spec_exception_out`.
    * **Verdict:** The core obeys the `SQUASH` (a full pipeline flush and checkpoint restore) and `COMMIT` (discards the checkpoint) signals from the STU.

## 🚧 Status

**Work in Progress (WIP)** - All sub-modules (`fetch`, `rename`, `issue`, `lsq`, `commit`, `rat`, `freelist`, `prf`, `arf`, `mdu`, `fpu`) and MMU components (`tlb`, `ptw`) have been designed. The final step is the integration and wiring of all these components inside the `supernova_core.sv` wrapper.
