/*
 * exact_sched.h — opt-in cycle-exact Mega Drive timing (GameSpec.exact_timing).
 *
 * The default runner schedules the recompiled 68000 per scanline with
 * heuristics (WaitForVBlank yields, atomic interrupt handlers on a private
 * stack, instantaneous DMA). That keeps games playable but makes the machine
 * state at a frame boundary only approximately the hardware's. A game that
 * must be frame-identical to a reference emulator opts into this mode
 * (`.exact_timing = 1` in its GameSpec, `exact_timing = true` in game.toml):
 *
 *   - Time is one master clock per frame (3420 per line, 7 per 68000 cycle,
 *     15 per Z80 T-state), with the frame laid out like Genesis Plus GX:
 *     line 261 first, then 0..260; H-int and DMA serviced at line starts;
 *     the VINT status flag 588 and the V-int 788 master clocks into the first
 *     vblank line.
 *   - Generated code starts every instruction with XI(pc, cost): a boundary
 *     where the slice may end (the game fiber parks) or an interrupt is taken,
 *     then the instruction's exact cost. Interrupts run hardware-style on the
 *     game fiber: PC and SR pushed on the live supervisor stack, 44 cycles,
 *     handler, RTE.
 *   - VDP port timing: status flags (VBLANK, VINT, HBLANK, DMA busy, FIFO),
 *     write FIFO stalls, DMA (68K bus transfers freeze the 68000; fill and
 *     copy run per line with DMA busy set), IRQ enable latency.
 *   - I/O pads (6-button TH counter with its line-based timeout), Z80 bus
 *     request / reset synchronisation, Z80 RAM write latency.
 *
 * Every rule here reproduces documented hardware behaviour as modelled by the
 * reference emulator the mode is verified against (BizHawk 2.4's Genesis Plus
 * GX); the per-frame RAM/VRAM/CRAM/VSRAM CRC comparison is the test.
 */
#ifndef GENESIS_EXACT_SCHED_H
#define GENESIS_EXACT_SCHED_H

#include <stdint.h>
#include "video/genesis_machine.h"

#ifdef __cplusplus
extern "C" {
#endif

extern int g_exact;   /* set once at startup from g_game_spec.exact_timing */

/* Frame driver (replaces machine_run_frame's body in exact mode). */
void exact_run_frame(GenesisScanlineSink sink, void *user);
void exact_reset(void);

/* 68000 bus in exact mode (glue.c's m68k_read/write route here). */
uint8_t  xbus_read8 (uint32_t a);
uint16_t xbus_read16(uint32_t a);
void     xbus_write8 (uint32_t a, uint8_t v);
void     xbus_write16(uint32_t a, uint16_t v);

/* YM2612 timer/status model (CPU-visible timing; t = master clock). */
uint8_t exact_ym_read(uint32_t t);
void    exact_ym_write(uint32_t t, unsigned port, uint8_t v);
/* Z80 time of the current access (genesis_machine.c, during a Z80 step). */
uint32_t machine_z80_now(void);

/* Host inspection (no side effects). */
uint16_t exact_peek_status(void);

#ifdef __cplusplus
}
#endif
#endif
