/*
 * genesis_runtime.h — Genesis/Mega Drive runtime interface.
 * Shared between runner/ and generated/ code.
 * Generated code includes this; runner implements it.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ---- CPU State ---- */
/* Tagged so headers that only need a pointer (e.g. crash_report.h) can
 * forward-declare `struct M68KState` and have it be the SAME type as this
 * typedef. clang treats an anonymous-struct typedef and a `struct M68KState`
 * tag as distinct (MSVC is laxer), which broke the crash_report prototypes. */
typedef struct M68KState {
    uint32_t D[8];   /* D0-D7 data registers */
    uint32_t A[8];   /* A0-A6 address registers, A7 = SSP (supervisor stack) */
    uint16_t SR;     /* Status register: T,S,I2,I1,I0,X,N,Z,V,C */
    uint32_t PC;     /* Program counter (for dynamic dispatch / debug) */
    uint32_t USP;    /* User Stack Pointer (shadow register, separate from A7) */
} M68KState;

extern M68KState g_cpu;
extern uint8_t   g_rom[0x400000];   /* 4MB ROM map */
extern uint8_t  *g_ram;             /* 64KB main RAM ($FF0000-$FFFFFF), as $00FFxxxx */
extern uint8_t  *g_ram_ffff;        /* the same RAM mapped again, as $FFFFxxxx (ram_views.c) */
void genesis_ram_init(void);        /* maps both views; first thing main() does */

/* Resolve the index value encoded by a 68000 brief extension word.
 * Bit 15 selects An/Dn and bit 11 selects the full long register or the
 * sign-extended low word. Keeping this shared prevents native code and the
 * Tier-3 interpreter from silently agreeing on the same wrong W/L behavior. */
static inline uint32_t m68k_brief_index_value(uint16_t extension,
                                              uint32_t dreg,
                                              uint32_t areg)
{
    uint32_t value = (extension & 0x8000u) ? areg : dreg;
    return (extension & 0x0800u)
        ? value
        : (uint32_t)(int32_t)(int16_t)value;
}

/* Widescreen (16:9) margin in extra pixels per side, set by the runner's
 * per-frame gate (0 = authentic 4:3). The recompiler's [[widescreen_site]]
 * injection reads this in the generated C to widen object-cull / tile-load /
 * ring-window bounds; 0 makes every injected adjustment a no-op, so 4:3 output
 * is byte-identical. Defined once in the runner (glue.c). */
extern int g_ws_margin;

/* ---- Memory Map ---- */
#define ROM_BASE    0x000000u
#define ROM_SIZE    0x400000u
#define RAM_BASE    0xFF0000u
#define RAM_SIZE    0x010000u
#define VDP_BASE    0xC00000u
#define Z80_BASE    0xA00000u
#define IO_BASE     0xA10000u

/* SR flag bits */
#define SR_C   (1u << 0)   /* Carry */
#define SR_V   (1u << 1)   /* Overflow */
#define SR_Z   (1u << 2)   /* Zero */
#define SR_N   (1u << 3)   /* Negative */
#define SR_X   (1u << 4)   /* Extend */
#define SR_I0  (1u << 8)   /* Interrupt mask bit 0 */
#define SR_I1  (1u << 9)   /* Interrupt mask bit 1 */
#define SR_I2  (1u << 10)  /* Interrupt mask bit 2 */
#define SR_S   (1u << 13)  /* Supervisor mode */
#define SR_T   (1u << 15)  /* Trace */

/* ---- Memory Interface ---- */
uint8_t  m68k_read8 (uint32_t addr);
uint16_t m68k_read16(uint32_t addr);
uint32_t m68k_read32(uint32_t addr);
void     m68k_write8 (uint32_t addr, uint8_t  val);
void     m68k_write16(uint32_t addr, uint16_t val);
void     m68k_write32(uint32_t addr, uint32_t val);

#include "genesis_host_mem.h"   /* glue_peek* / glue_poke*: host-side access */

/* ---- Dispatch ---- */
/* Called for JMP (An) and indexed jump tables — dispatch to the correct recompiled function */
void call_by_address(uint32_t addr);

/* Generated-code trampoline helpers. Branch/JMP edges that cross a generated
 * split-function boundary are 68K tail jumps, not C calls; these helpers keep
 * those chains off the host stack while preserving direct JSR/BSR calls. */
typedef void (*RecompFuncPtr)(void);
void recomp_tail_call(uint32_t addr);
void recomp_call_addr(uint32_t addr);
void recomp_call_func(RecompFuncPtr fn);
/* Tail-dispatch frame list (defined in the generated <prefix>_dispatch.c).
 * Each active recomp_call_func/recomp_call_addr owns one frame on the GAME
 * FIBER stack; the head pointer is therefore execution state that must travel
 * with a fiber snapshot (get/set). walk visits frames innermost-first with
 * their guest-visible fields only (pending flag, pending 24-bit target), so
 * digests can hash the chain without host addresses. */
void *recomp_tail_frame_get(void);
void  recomp_tail_frame_set(void *frame);
void  recomp_tail_frame_walk(void (*visit)(int pending, uint32_t addr, void *user),
                             void *user);
void recomp_push_return(uint32_t ret_addr);
uint32_t recomp_resolve_ram_trampoline(uint32_t addr);
/* Execute terminal one-instruction RAM stubs whose semantics are represented by
 * returning to the native dispatcher. Currently handles RTS; returns nonzero
 * when the address was consumed. */
int recomp_dispatch_ram_stub(uint32_t addr);

/* Logging for dispatch misses */
void genesis_log_dispatch_miss(uint32_t addr);
/* Rewrite the runtime evidence files (dispatch_misses.toml & co.) with the
 * current counters. Returns the number of dispatch-miss addresses recorded
 * for this build across all sessions (runner/runtime_evidence.h). */
int genesis_write_dispatch_miss_evidence(void);

/* JMP table interpreter fallback — runs interpreter from target_pc until RTS */
void hybrid_jmp_interpret(uint32_t target_pc);

/* call_by_address interpreter fallback — runs interpreter when dispatch misses */
void hybrid_call_interpret(uint32_t target_pc);

/* ---- Fiber yield (Step 2) ---- */
void glue_yield_for_vblank(void);
void glue_yield_for_interrupt_poll(void);

/* ---- Vectored exceptions (Phase 7A) ----
 * Real semantics for TRAP #N / TRAPV / RTR / RESET / ILLEGAL / A-line /
 * F-line. Generated code calls into these instead of emitting a comment.
 *
 * m68k_trap_vector(vec) — dispatches a vectored trap by 8-bit vector
 *   number. Vectors 0x20..0x2F are the TRAP #0..#15 family. Other slots
 *   used by Phase 7A: 4=ILLEGAL, 6=CHK (Phase 7C), 7=TRAPV, 10=A-line,
 *   11=F-line. Sonic 1 in steady-state takes none of these — the hook
 *   logs and halts to surface the regression loudly rather than silently
 *   continuing past corrupt CPU state.
 *
 * m68k_illegal_trap(pc, opcode) — convenience wrapper that picks the
 *   right vector from the offending opcode (0x4AFC→4, A-line→10,
 *   F-line→11) and forwards to m68k_trap_vector. */
void m68k_trap_vector(uint8_t vec);
void m68k_illegal_trap(uint32_t pc, uint16_t opcode);

/* RESET pulses the external /RESET line. Re-initialise external devices
 * (Z80, YM2612, PSG) here if needed. Sonic 1's boot path issues this
 * once at startup, before runtime_init() has finished setting up the
 * devices for the first time, so for that title the hook is a no-op. */
void genesis_reset_devices(void);

/* STOP #imm halts the 68K until an interrupt of priority > I-mask
 * arrives. Generated code calls this after writing imm into SR. The
 * runtime yields to the VBlank fiber so frame-pacing keeps moving and
 * the next service_vblank() raises VBlank/HBlank IRQs. */
void genesis_stop_until_interrupt(uint16_t sr_imm);

/* ---- VDP Interface ---- */
void vdp_write_data(uint16_t val);
void vdp_write_ctrl(uint16_t val);
uint16_t vdp_read_data(void);
uint16_t vdp_read_status(void);
void vdp_render_frame(uint32_t *framebuf);   /* 320x224 or 256x224 ARGB8888 */

/* ---- Runtime Init ---- */
void runtime_init(void);

/* ---- Frame sync ---- */
/* Called by the runner at 60Hz. Signals the game thread to service VBlank+HBlank
 * on its own stack (eliminating g_ram races), then blocks until complete. */
void runtime_request_vblank(void);

/* ---- Debug: function call counter (temporary) ---- */
extern int g_dbg_b64_count;   /* incremented at start of func_000B64 */
extern int g_dbg_b5e_count;   /* incremented at start of func_000B5E */
extern int g_dbg_b88_count;   /* incremented at start of func_000B88 */

/* ---- RTE propagation ---- */
/* Set to 1 by RTE, or to the number of discarded return slots by a
 * stack-skipping RTS. Each generated JSR/BSR caller consumes one level
 * before returning; tail dispatch preserves the count. Cleared by the
 * VBlank service routine after the handler chain completes. */
/* g_rte_pending is accessed via a pointer so the runner can redirect it
 * to a dummy variable during VBlank service, suppressing RTE propagation
 * inside the interrupt handler chain.  The handler's MOVEM + RTE stack
 * management is handled by force-restoring A7 in glue_service_vblank(). */
extern int *g_rte_pending_ptr;
#define g_rte_pending (*g_rte_pending_ptr)

/* Stack-slot accounting carried between generated functions joined by a
 * tail-dispatch edge. Generated code defines this alongside its dispatch
 * table. Runtime re-entry (most importantly an IRQ handler executed while the
 * game fiber is suspended) must isolate its own split chain and restore the
 * interrupted chain's value before resuming it. */
extern int g_split_sp_popped;

/* ---- Early return (addq.l #4,sp + rts pattern) ---- */
/* 68K code uses "addq.l #4,sp" to discard a return address, then "rts" to
 * return to the caller's caller (skipping the rest of the current routine).
 * In C, "return" always goes to the immediate caller.  g_early_return counts
 * how many extra return levels to propagate (legacy symbol). Generated
 * stack-skip accounting now transfers the full count to g_rte_pending. */
extern int g_early_return;

/* Sparse game-owned pre-instruction extension (only configured PCs call it). */
int genesis_game_instruction_hook(uint32_t pc);
/* Generated table lookup: nonzero when `pc` is a configured game_hook site.
 * The tier-3 interpreter uses it to run the same hooks as generated code. */
int game_instruction_hook_site(uint32_t pc);

/* ---- Frame counter ---- */
extern uint64_t g_frame_count;

/* ---- Instruction-count telemetry (Stage C) ----
 * Ticked by generated C once per decoded 68K instruction; defined in
 * runner/glue.c. 0 on oracle (generated functions don't execute). */
extern uint64_t g_native_insn_count;

/* ---- Contextual recompiler: cycle-accurate VBlank ---- */
/* Accumulated 68K cycles since frame start. When this reaches the VBlank
 * threshold (~109312 = scanline 224 × 488 cycles), glue_check_vblank()
 * fires the VBlank handler between instructions. */
extern uint32_t g_cycle_accumulator;
extern uint32_t g_vblank_threshold;
void glue_check_vblank(void);

/* Audio event-queue cycle stamp. Bumped per-instruction by the generator
 * (same cycle-table source as g_cycle_accumulator). Monotonic within a
 * wall frame; resets at end-of-wall-frame. Used to time-stamp FM/PSG
 * register writes onto the cycle-stamped event queue consumed by the
 * audio mixer (runner/audio/). Defined in runner/glue.c. */
extern uint32_t g_audio_cycle_counter;

/* ---- Differential co-simulation per-instruction checkpoint hook ----------
 * GEN_COSIM_TICK(n) is emitted per instruction by the generator
 * (emit_cycle_accounting) and mirrored in m68k_interp.c (interp_account_cycles).
 * It advances a MONOTONIC cycle axis that both backends bump by the same
 * per-instruction cost, and when that axis crosses the next checkpoint boundary
 * the guest hashes full architectural state and PARKS for the coordinator
 * (see cosim.c). Expands to nothing unless GENESIS_COSIM is defined, so normal
 * native/oracle builds are byte-for-byte unaffected. The stride between
 * checkpoints (env GENESIS_COSIM_STRIDE) subsumes frame-sync (large stride) and
 * cycle-exact (small stride) under one mechanism. */
#ifdef GENESIS_COSIM
extern uint64_t g_cosim_cycle;      /* monotonic; never resets (unlike g_audio) */
extern uint64_t g_cosim_next_cp;    /* next checkpoint boundary                 */
void cosim_checkpoint(void);        /* hash + park; advances g_cosim_next_cp    */
#define GEN_COSIM_TICK(n) do { g_cosim_cycle += (uint64_t)(n); \
    if (g_cosim_cycle >= g_cosim_next_cp) cosim_checkpoint(); } while (0)
#else
#define GEN_COSIM_TICK(n) ((void)0)
#endif

/* ---- Exact timing (game.toml exact_timing; runner/exact_sched.c) --------
 * Every generated instruction starts with XI(pc, cost): a boundary where the
 * scheduler may end the 68000's time slice or take an interrupt, then the
 * instruction's cost in master clocks (7 per 68000 cycle). XC adds a
 * data-dependent cost; XSR requests an interrupt check after an SR write. */
extern uint32_t g_xclk, g_xstart, g_xend, g_xpc;
extern uint32_t g_xrts;    /* where the last RTS returned (68000 stack), exact mode */
/* The 68000 PC's top byte (as bits 24-31), exact mode. The bus ignores it, but the PC keeps
 * it: a JMP/JSR through a pointer with a tag byte, an RTS/RTE popping one, or an exception
 * vector sets it, PC-relative branches keep it, and every pushed return address and
 * PC-relative LEA/PEA carries it, as on the reference (Musashi never masks the PC). */
extern uint32_t g_pchi;
/* Returns nobody waits for. A call site waits for its own return address
 * (XCALL_BEGIN/XCALL_END around the call) and consumes a matching RTS
 * (XRTS_TAKEN). An RTS that lands where no enclosing site waits, e.g. a
 * threaded-code handler returning into its dispatch loop (`pea loop(pc); jmp
 * handler`), is where the 68000 continues: the dispatcher runs it next
 * (x68k_resume, called by the generated tail-call drain). */
#define XAWAIT_MAX 4096
/* An await that takes any return: a C caller that runs one function and wants control back at
 * its RTS wherever it returns (a test harness comparing where it went; C calling 68000 code),
 * as every call did before returns nobody waits for. */
#define XAWAIT_ANY 0xFFFFFFFFu
extern uint32_t g_xawait[XAWAIT_MAX];
extern int      g_xawait_n;
extern int      g_xrts_new;  /* an RTS set g_xrts and no call site has taken it yet */
#define XCALL_BEGIN(r) do { if (g_xawait_n < XAWAIT_MAX) g_xawait[g_xawait_n] = (r); g_xawait_n++; } while (0)
#define XCALL_END() (g_xawait_n--)
#define XRTS_TAKEN() (g_xrts_new = 0)
int x68k_resume(uint32_t *addr);  /* 1: run *addr next (a fresh RTS nobody waits for) */
/* After an RTE popped SR and PC: inside an interrupt handler it returns to the interrupted
 * code (g_rte_pending); outside one (a context switch: push PC and SR, RTE) it is a jump to
 * the popped PC, which the dispatcher runs next like an RTS nobody waits for. */
void x68k_rte(void);
/* Exact mode, a dispatch miss in ROM code (a task the game resumes where an interrupt parked
 * it, mid-function): run it on the exact interpreter until an address generated code can be
 * entered at (has_entry), then tail-dispatch that. 1 = handled. */
int recomp_exact_rom_resume(uint32_t addr, int (*has_entry)(uint32_t));
/* Exact mode, a dispatch to RAM code: the exact interpreter from addr until ROM code generated
 * code can be entered at (has_entry), then tail-dispatch that. 1 = handled. */
int recomp_exact_ram_run(uint32_t addr, int (*has_entry)(uint32_t));
extern uint32_t g_xa7min;  /* lowest A7 at any instruction boundary (sync CRC dead stack) */
extern uint64_t g_xinsn;   /* instructions started (logical positions: interrupt since) */
extern int      g_xtrace;
void x68k_slice(uint32_t pc);
void x68k_sr_written(void);
void x68k_trace(uint32_t pc);
void x68k_divide_by_zero(uint32_t next_pc);
#define XI(pc, c) do { XI_BOUNDARY(pc); XI_CHARGE(pc, c); } while (0)
/* XI's two halves: the boundary (slice end / interrupt) and the start of the instruction with
 * its cost. The interpreter (RAM-resident code) decodes between them: an interrupt taken at
 * the boundary may patch the very instruction it interrupted, which hardware refetches. */
#define XI_BOUNDARY(pc) do { if (g_xclk >= g_xend) { x68k_slice(pc); \
    if (g_xabort) { g_xabort = 0; return; } } } while (0)
/* Set by the scheduler when an interrupt handler's RTE went to another PC than the
 * interrupted one (a task switch): the interrupted code is abandoned, so the generated
 * function returns at its boundary and the new PC (g_xrts, fresh) is routed like a return. */
extern int g_xabort;
#define XI_CHARGE(pc, c) do { g_xpc = (pc); \
    g_xstart = g_xclk; if (g_xtrace) x68k_trace(pc); g_xclk += (c); g_xinsn++; \
    if (g_cpu.A[7] < g_xa7min) g_xa7min = g_cpu.A[7]; } while (0)
/* game.toml entry_hooks: a game function entered at its own start (a call of
 * func_ADDR). Counts logical positions (sync points, replay; exact_sched.c).
 * Nonzero = the body already ran (a verification harness ran it): return. */
int x68k_entry(uint32_t addr);
#define XENTRY(addr) x68k_entry(addr)
/* Verification hooks (exact_sched.c; a shadow-test harness sets them):
 * g_xhw_hook sees every 68000 hardware access ($A00000-$A1FFFF, $C00000-$C0001F).
 *   kind 0 = read about to happen: return 1 with *v set to supply the value
 *            (no device access), 0 to read the device;
 *   kind 1 = read done (the device returned v);
 *   kind 2 = write: return 1 to swallow it (no device access).
 * g_xinsn_hook is called with every instruction's pc while set (via g_xtrace).
 * g_xirq_taken counts interrupts taken. */
extern int (*g_xhw_hook)(uint32_t addr, int size, int kind, uint16_t v, uint16_t *out);
extern void (*g_xinsn_hook)(uint32_t pc);
extern uint64_t g_xirq_taken;
/* While set, an SR write requests no interrupt check: a shadow test's untimed C run (game verify)
 * must not take an interrupt that fell due during the generated run it is compared with. */
extern int g_xirq_hold;
#define XC(c) (g_xclk += (uint32_t)(int32_t)(c))
#define XSR() x68k_sr_written()
/* Untimed code (stepped runner): before an instruction writes SR, so a logged interrupt due
 * now is taken while the old mask still allows it. */
void x68k_sr_pre(void);
#define XSRP() x68k_sr_pre()
/* The frame-synchronous stepped runner takes the V-int now (a wait for it with no sync point);
   any other runner: nothing (exact_sched.c). */
void x68k_wait_vint(uint32_t pc);

/* Operand-dependent MUL/DIV time in master clocks (the standard 68000 timing
 * algorithms: MULU 38+2n for n set bits, MULS 38+2n for n 01/10 pairs, DIVU
 * and DIVS per the shift-subtract microcode). */
static inline uint32_t x68k_mulu_mclk(uint16_t s)
{
    uint32_t c = 38;
    while (s) { c += (s & 1) * 2; s >>= 1; }
    return c * 7u;
}
static inline uint32_t x68k_muls_mclk(int16_t s)
{
    uint32_t c = 38, t = (((uint32_t)(uint16_t)s << 1) ^ (uint16_t)s) & 0xFFFFu;
    while (t) { c += (t & 1) * 2; t >>= 1; }
    return c * 7u;
}
static inline uint32_t x68k_divu_mclk(uint32_t dst, uint32_t src)
{
    uint32_t m = 38 * 7;
    src <<= 16;
    for (int i = 0; i < 15; i++) {
        if ((int32_t)dst < 0) { dst <<= 1; dst -= src; }
        else { dst <<= 1; m += 2 * 7; if (dst >= src) { dst -= src; m -= 7; } }
    }
    return m << 1;
}
static inline uint32_t x68k_divs_mclk(int32_t dst, int16_t src)
{
    uint32_t m = 6 * 7;
    uint32_t ad = dst < 0 ? (uint32_t)0 - (uint32_t)dst : (uint32_t)dst;
    uint32_t as = src < 0 ? (uint32_t)(-(int32_t)src) : (uint32_t)src;
    if (dst < 0) m += 7;
    if ((ad >> 16) < as) {
        uint32_t q = ad / as;
        m += 55 * 7;
        if (src >= 0) { if (dst >= 0) m -= 7; else m += 7; }
        for (int i = 0; i < 15; i++) { q >>= 1; if (!(q & 1)) m += 7; }
    } else {
        m += 2 * 7;
    }
    return m << 1;
}

/* ---- Controller ---- */
/* 3-button pad bitmask: bit5=C, bit4=B, bit3=A,
 *   bit2=Start, bit1=Up, bit0=Down (bit6=Left? depends on read mode)
 * Full 6-button pad uses an extended protocol — stub for now. */
extern uint8_t g_controller1_buttons;
extern uint8_t g_controller2_buttons;

/* ---- Dispatch miss monitor ---- */
extern uint32_t g_miss_count_any;
extern uint32_t g_miss_last_addr;
extern uint64_t g_miss_last_frame;
#define MAX_MISS_UNIQUE 64
extern uint32_t g_miss_unique_addrs[MAX_MISS_UNIQUE];
extern int      g_miss_unique_count;

/* ---- Logger ---- */
void log_on_change(const char *label, uint32_t value);
