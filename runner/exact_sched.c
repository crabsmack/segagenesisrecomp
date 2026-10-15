/*
 * exact_sched.c — opt-in cycle-exact Mega Drive timing. See exact_sched.h.
 *
 * State naming follows the hardware: `status` is the VDP status register,
 * `hint_pending`/`vint_pending` the VDP's interrupt latches, `int_level` the
 * 68000's IPL input. Time is in master clocks from the start of the frame
 * (`s_mline` = start of the current line).
 */
#include "exact_sched.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "game_spec.h"
#include "genesis_runtime.h"
#include "glue.h"
#include "video/genesis_bus.h"
#include "video/genesis_machine.h"
#include "video/genesis_vdp.h"

int g_exact = 0;

/* ---- clock (shared with generated code via genesis_runtime.h) ------------ */
uint32_t g_xclk   = 0;     /* 68000 time: end of the current instruction       */
uint32_t g_xstart = 0;     /* 68000 time at the start of the current instruction */
uint32_t g_xend   = 0;     /* boundary trigger: s_end, or 0 to force a check    */
uint32_t g_xpc    = 0;     /* address of the current instruction               */
uint32_t g_xrts   = 0;     /* 68000 return target of the last RTS               */
int      g_xtrace = 0;

static uint32_t s_end;     /* end of the current 68000 time slice              */
static uint32_t s_mline;   /* master clock at the start of the current line    */

#define MPL        3420u   /* master clocks per line                           */
#define LINES      262
#define MCLK_68K   7u

/* glue.c: fiber control for the game (68000) fiber. */
extern void glue_exact_resume(void);   /* run the game fiber until it parks   */
extern void glue_exact_park(void);     /* game fiber: give control back       */
extern int  glue_exact_running(void);

/* ---- 68000 interrupt input ------------------------------------------------ */
static unsigned s_int_level;     /* IPL level presented by the VDP (0/4/6)     */
static int      s_irq_latency;   /* one-instruction delay after an IRQ enable  */
static uint32_t s_irq_latency_pc;

/* ---- VDP timing state ----------------------------------------------------- */
static uint16_t s_status;        /* 0x200 FIFO empty, 0x100 full, 0x80 VINT,
                                    0x08 VBLANK, 0x02 DMA busy                 */
static uint8_t  s_hint_pending, s_vint_pending;
static int      s_hcounter;      /* line-interrupt counter                     */
static int      s_vcounter;      /* current line (261 = the frame's first)     */
static int      s_view_h = 192;  /* active lines this frame (mode 4 at reset)  */
static int      s_view_changed;
static uint8_t  s_pending;       /* control port: first word written           */
static uint16_t s_addr_latch;
static int32_t  s_cached_write = -1;
static uint32_t s_hvc_latch = 0x10000;
/* DMA */
static uint32_t s_dma_length;
static int      s_dma_type;      /* 0 68K->VRAM, 1 68K->CRAM/VSRAM, 2 fill, 3 copy */
static uint32_t s_dma_end;
static int      s_dmafill;
static uint32_t s_dma_src;       /* word address (68K) / byte address (copy)   */
/* FIFO */
static uint16_t s_fifo[4];
static int      s_fifo_idx;
static int      s_fifo_write_cnt, s_fifo_slots, s_fifo_byte_access = 1;
static uint32_t s_fifo_cycles;

/* ---- I/O ------------------------------------------------------------------- */
static uint8_t  s_io_reg[16];
static uint8_t  s_pad_state[2] = { 0x40, 0x40 };
static uint8_t  s_pad_counter[2], s_pad_timeout[2];

/* ---- Z80 ------------------------------------------------------------------ */
static uint32_t s_zclk;          /* Z80 time in master clocks                 */
static int      s_zstate;        /* bit0 reset released, bit1 bus requested   */
static int      s_zaccess;       /* 68000 may access the Z80 area             */
static int      s_zirq;          /* V-int line asserted to the Z80            */
extern void     machine_z80_exact_step(uint32_t *zclk, uint32_t target);
extern void     machine_z80_exact_reset(void);
extern void     machine_z80_exact_irq(int asserted);

#define V (&g_machine.vdp)
#define REG(n) (g_machine.vdp.reg[(n)])

/* ======================================================================== */
/* Trace (GENESIS_XTRACE=from,to,path): one line per executed instruction,
 * `frame,pc,mclk,sr,a7`, comparable with the reference's emu/cycle_trace.py. */
static FILE    *s_trace_f;
static uint64_t s_trace_from, s_trace_to;
static uint64_t s_frame_no;      /* 1-based number of the frame being run      */

void x68k_trace(uint32_t pc)
{
    /* The reference runs the instruction in an IRQ-latency slot inline, without
     * its per-instruction hook; leave it out here too so the traces align. */
    if (s_irq_latency == 2 && pc == s_irq_latency_pc) return;
    if (s_trace_f) {
        uint32_t h = 2166136261u;   /* word-wise FNV-1a over D0-D7/A0-A7 */
        for (int i = 0; i < 8; i++) h = (h ^ g_cpu.D[i]) * 16777619u;
        for (int i = 0; i < 8; i++) h = (h ^ g_cpu.A[i]) * 16777619u;
        fprintf(s_trace_f, "%llu,%06X,%u,%04X,%06X,%08X\n", (unsigned long long)s_frame_no,
                pc & 0xFFFFFFu, g_xstart, (unsigned)g_cpu.SR, g_cpu.A[7] & 0xFFFFFFu, h);
    }
}

static void trace_frame_begin(void)
{
    static int init;
    if (!init) {
        init = 1;
        const char *e = getenv("GENESIS_XTRACE");
        if (e && *e) {
            char path[512] = "";
            unsigned long long a = 0, b = 0;
            if (sscanf(e, "%llu,%llu,%511s", &a, &b, path) == 3) {
                s_trace_from = a; s_trace_to = b;
                s_trace_f = fopen(path, "w");
                if (s_trace_f) {
                    setvbuf(s_trace_f, NULL, _IOFBF, 1 << 22);
                    fprintf(s_trace_f, "frame,pc,mclk,sr,a7,rh\n");
                }
            }
        }
    }
    if (s_trace_f) fflush(s_trace_f);   /* exit paths may skip stdio teardown */
    g_xtrace = s_trace_f && s_frame_no >= s_trace_from && s_frame_no <= s_trace_to;
    if (s_trace_f && s_frame_no > s_trace_to) { fclose(s_trace_f); s_trace_f = NULL; g_xtrace = 0; }
}

/* ======================================================================== */
/* 68000 interrupts                                                          */

static void irq_ack(unsigned level)
{
    /* The VDP clears the acknowledged latch; V-int has priority. */
    if (s_vint_pending & REG(1)) {
        s_vint_pending = 0;
        s_status &= (uint16_t)~0x80;
        s_int_level = (s_hint_pending & REG(0)) ? 4 : 0;
    } else {
        s_hint_pending = 0;
        s_int_level = 0;
    }
    (void)level;
}

static void force_boundary(void) { g_xend = 0; }

static void set_irq(unsigned level)          { s_int_level = level; force_boundary(); }
static void update_irq(unsigned level)       { if (level > s_int_level) s_int_level = level; force_boundary(); }
static void set_irq_delay(unsigned level)
{
    /* Raised by a VDP register write enabling a pending interrupt: the 68000
     * finishes the writing instruction and executes ONE more before taking it,
     * unless the write came from a MOVE.L (the reference cannot split one),
     * in which case it is taken right after the MOVE.L. */
    if (!s_irq_latency) {
        uint16_t op = (uint16_t)((g_rom[g_xpc & 0x3FFFFFu] << 8) | g_rom[(g_xpc + 1) & 0x3FFFFFu]);
        if ((op & 0xF000) != 0x2000) s_irq_latency = 1;
    }
    s_int_level = level;
    force_boundary();
}

/* Exception processing for an autovectored interrupt at the boundary before
 * `pc`, then the handler itself, on the game fiber. */
static void take_interrupt(uint32_t pc, unsigned level)
{
    uint16_t sr = g_cpu.SR;
    g_cpu.SR = (uint16_t)((sr & ~0x8000u) | 0x2000u);        /* S set, T clear */
    g_cpu.SR = (uint16_t)((g_cpu.SR & ~0x0700u) | (level << 8));
    irq_ack(level);
    uint32_t vec = m68k_read32((24u + level) * 4u) & 0xFFFFFFu;
    g_cpu.A[7] -= 4; m68k_write32(g_cpu.A[7], pc & 0xFFFFFFu);
    g_cpu.A[7] -= 2; m68k_write16(g_cpu.A[7], sr);
    g_xclk += 44u * MCLK_68K;

    int saved_rte = g_rte_pending;
    g_rte_pending = 0;
    int saved_split = g_split_sp_popped;   /* isolate the interrupted split chain */
    g_split_sp_popped = 0;
    recomp_call_addr(vec);
    g_split_sp_popped = saved_split;
    if (!g_rte_pending) {
        fprintf(stderr, "[exact] interrupt handler $%06X returned without RTE (pc $%06X)\n",
                vec, pc);
        exit(4);
    }
    if ((g_cpu.PC & 0xFFFFFFu) != (pc & 0xFFFFFFu)) {
        fprintf(stderr, "[exact] RTE to $%06X, interrupted at $%06X: unsupported\n",
                g_cpu.PC & 0xFFFFFFu, pc & 0xFFFFFFu);
        exit(4);
    }
    g_rte_pending = saved_rte;
}

/* Called by XI() when g_xclk >= g_xend: the slice is over (park the fiber
 * until the scheduler gives it more time) and/or an interrupt is due. */
void x68k_slice(uint32_t pc)
{
    for (;;) {
        if (g_xclk >= s_end) {
            g_xend = s_end;
            glue_exact_park();            /* returns at the start of a new slice */
            continue;
        }
        if (s_irq_latency == 1) {         /* run exactly one more instruction */
            s_irq_latency = 2;
            s_irq_latency_pc = pc;
            g_xend = 0;                   /* stop again at the next boundary */
            return;
        }
        s_irq_latency = 0;
        unsigned mask = (g_cpu.SR >> 8) & 7u;
        if (s_int_level > mask) {
            take_interrupt(pc, s_int_level);
            continue;                     /* the handler may have ended the slice */
        }
        break;
    }
    g_xend = s_end;
}

void x68k_sr_written(void) { force_boundary(); }

void x68k_divide_by_zero(uint32_t next_pc)
{
    fprintf(stderr, "[exact] divide by zero before $%06X: zero-divide trap not modelled\n", next_pc);
    exit(4);
}

/* Run the 68000 until master clock `target`. */
static void m68k_run(uint32_t target)
{
    if (g_xclk >= target || !glue_exact_running()) return;
    s_end = target;
    g_xend = 0;                           /* interrupt check at the slice start */
    glue_exact_resume();
}

/* ======================================================================== */
/* Z80                                                                       */

static void fm_reset_chip(uint32_t t);
static void fm_update(uint32_t t);

static void z80_run(uint32_t target)
{
    if (s_zstate == 1) machine_z80_exact_step(&s_zclk, target);
    else               s_zclk = target;
}

static void z80_busreq_w(int req, uint32_t now)
{
    if (req) {
        if (s_zstate == 1) { z80_run(now); s_zaccess = 1; }
        s_zstate |= 2;
    } else {
        if (s_zstate == 3) { s_zclk = now; s_zaccess = 0; }
        s_zstate &= 1;
    }
    g_machine.bus.z80_busreq = (uint8_t)((s_zstate & 2) ? 1 : 0);
}

static void z80_reset_w(int release, uint32_t now)
{
    if (release) {
        if (s_zstate == 0) { s_zclk = now; machine_z80_exact_reset(); fm_reset_chip(now); }
        else if (s_zstate == 2) { s_zaccess = 1; machine_z80_exact_reset(); fm_reset_chip(now); }
        s_zstate |= 1;
    } else {
        if (s_zstate == 1) z80_run(now);
        else if (s_zstate == 3) s_zaccess = 0;
        fm_reset_chip(now);
        s_zstate &= 2;
    }
    g_machine.bus.z80_reset_off = (uint8_t)(s_zstate & 1);
}

/* ======================================================================== */
/* VDP                                                                       */

static uint32_t dma_length_reg(void)
{
    uint32_t n = ((uint32_t)REG(20) << 8) | REG(19);
    return n ? n : 0x10000u;
}

/* The VDP's internal 16-bit write: one FIFO entry, then the store. */
static void vdp_bus_w(uint16_t data)
{
    GVDP *v = V;
    s_fifo[s_fifo_idx] = data;
    s_fifo_idx = (s_fifo_idx + 1) & 3;
    switch (v->code & 0x0F) {
    case 0x01: gvdp_store_vram_word(v, v->address, data); break;
    case 0x03: gvdp_store_cram(v, v->address, data);      break;
    case 0x05: gvdp_store_vsram(v, v->address, data);     break;
    default:   g_xstart += 2; g_xclk += 2; break;         /* invalid target   */
    }
    v->address = (uint16_t)(v->address + REG(15));
}

static void dma_68k(uint32_t words)
{
    uint32_t source = ((uint32_t)REG(23) << 17) | (s_dma_src << 1);
    do {
        uint16_t data;
        if ((source & 0xFFFFFFu) >= 0xA00000u && (source & 0xFFFFFFu) < 0xA20000u) {
            /* I/O area: Z80 window reads RAM unless the Z80 owns the bus. */
            data = (uint16_t)((g_ram[source & 0xFFFFu] << 8) | g_ram[(source + 1) & 0xFFFFu]);
        } else {
            data = gbus_peek16(&g_machine.bus, source & 0xFFFFFFu);
        }
        source += 2;
        source = ((uint32_t)REG(23) << 17) | (source & 0x1FFFFu);
        vdp_bus_w(data);
    } while (--words);
    s_dma_src = (source >> 1) & 0xFFFFu;
}

static void dma_copy(uint32_t bytes)
{
    GVDP *v = V;
    if (!(v->code & 0x10)) return;
    uint16_t source = (uint16_t)s_dma_src;
    do {
        v->vram[(uint16_t)(v->address ^ 1)] = v->vram[(uint16_t)(source ^ 1)];
        source++;
        v->address = (uint16_t)(v->address + REG(15));
    } while (--bytes);
    s_dma_src = source;
}

static void dma_fill(uint32_t bytes)
{
    GVDP *v = V;
    switch (v->code & 0x0F) {
    case 0x01: {
        uint8_t data = (uint8_t)(s_fifo[(s_fifo_idx + 3) & 3] >> 8);
        do {
            v->vram[(uint16_t)(v->address ^ 1)] = data;
            v->address = (uint16_t)(v->address + REG(15));
        } while (--bytes);
        break;
    }
    case 0x03: {
        uint16_t data = s_fifo[s_fifo_idx];
        do { gvdp_store_cram(v, v->address, data); v->address = (uint16_t)(v->address + REG(15)); } while (--bytes);
        break;
    }
    case 0x05: {
        uint16_t data = s_fifo[s_fifo_idx];
        do { gvdp_store_vsram(v, v->address, data); v->address = (uint16_t)(v->address + REG(15)); } while (--bytes);
        break;
    }
    default:
        do { v->address = (uint16_t)(v->address + REG(15)); } while (--bytes);
        break;
    }
}

static void vdp_ctrl_w(uint16_t data);

/* Advance the current DMA to the end of this line (or of vblank). `now` is
 * the master clock it is serviced at: during the 68000's port write (the
 * DMA's start), or at a line start (the frame loop, CPU parked). */
static void dma_update_at(uint32_t now, int from_cpu);
static void dma_update(uint32_t now) { dma_update_at(now, 1); }
static void dma_update_at(uint32_t now, int from_cpu)
{
    static const uint32_t rate_tab[2][2] = { { 16, 18 }, { 167, 205 } };
    int blank = (s_status & 8) || !(REG(1) & 0x40);
    uint32_t rate = rate_tab[blank][REG(12) & 1] >> (s_dma_type & 1);
    int32_t dma_cycles;
    if (s_status & 8) dma_cycles = (int32_t)(LINES * MPL) - (int32_t)now;
    else              dma_cycles = (int32_t)(s_mline + MPL) - (int32_t)now;
    int32_t dma_bytes = (int32_t)(((int64_t)dma_cycles * rate) / MPL);
    if ((int32_t)s_dma_length < dma_bytes) {
        dma_bytes = (int32_t)s_dma_length;
        dma_cycles = (int32_t)((dma_bytes * MPL) / rate);
    }
    if (s_dma_type < 2) {
        /* The 68000 is frozen during a 68K-bus DMA: its clock is SET to the
         * transfer's end. At the start the triggering instruction's own cost
         * still follows; a continuation at a line start replaces the clock
         * outright, as the reference does (that cost is not charged again). */
        uint32_t t = now + (uint32_t)dma_cycles;
        if (from_cpu) { g_xclk = t + (g_xclk - g_xstart); g_xstart = t; }
        else          { g_xclk = t; }
    } else {
        s_status |= 0x02;
        s_dma_end = now + (uint32_t)dma_cycles;
    }
    if (dma_bytes > 0) {
        s_dma_length -= (uint32_t)dma_bytes;
        switch (s_dma_type) {
        case 0: case 1: dma_68k((uint32_t)dma_bytes); break;
        case 2:         dma_fill((uint32_t)dma_bytes); break;
        default:        dma_copy((uint32_t)dma_bytes); break;
        }
        if (!s_dma_length) {
            uint16_t end = (uint16_t)(REG(21) + (REG(22) << 8) + REG(19) + (REG(20) << 8));
            REG(21) = (uint8_t)end;
            REG(22) = (uint8_t)(end >> 8);
            REG(19) = REG(20) = 0;
            if (s_cached_write >= 0) {
                uint16_t w = (uint16_t)s_cached_write;
                s_cached_write = -1;
                vdp_ctrl_w(w);
            }
        }
    }
}

static void fifo_update(uint32_t now)
{
    static const int h32[16 + 4] = { 230, 510, 810, 970, 1130, 1450, 1610, 1770, 2090, 2250, 2410,
        2730, 2890, 3050, 3350, 3370, MPL + 230, MPL + 510, MPL + 810, MPL + 970 };
    static const int h40[18 + 4] = { 352, 820, 948, 1076, 1332, 1460, 1588, 1844, 1972, 2100, 2356,
        2484, 2612, 2868, 2996, 3124, 3364, 3380, MPL + 352, MPL + 820, MPL + 948, MPL + 1076 };
    const int *tab;
    int slots, count = 0;
    if (REG(12) & 1) { tab = h40; slots = 18 * ((s_vcounter + 1) % LINES); }
    else             { tab = h32; slots = 16 * ((s_vcounter + 1) % LINES); }
    int cyc = (int)now - (int)s_mline;
    while (tab[count] <= cyc) count++;
    int num = (slots + count - s_fifo_slots) >> s_fifo_byte_access;
    if (num > 0) {
        s_fifo_write_cnt -= num;
        s_status &= 0xFEFF;
        if (s_fifo_write_cnt <= 0) {
            s_fifo_write_cnt = 0;
            s_status |= 0x200;
            s_fifo_slots = slots + count;
        } else {
            s_fifo_slots += num << s_fifo_byte_access;
        }
    }
    s_fifo_cycles = s_mline + (uint32_t)tab[s_fifo_slots - slots + s_fifo_byte_access];
}

static void vdp_data_w(uint16_t data)
{
    s_pending = 0;
    if (!(s_status & 8) && (REG(1) & 0x40)) {
        fifo_update(g_xstart);
        s_status &= 0xFDFF;
        if (s_fifo_write_cnt < 4) {
            s_fifo_write_cnt++;
            s_status |= (uint16_t)((s_fifo_write_cnt & 4) << 6);
        } else {
            /* FIFO full: the 68000 waits for the next slot. */
            if (s_fifo_cycles > g_xstart) { g_xclk += s_fifo_cycles - g_xstart; g_xstart = s_fifo_cycles; }
            s_fifo_slots += 1 + s_fifo_byte_access;
        }
    }
    vdp_bus_w(data);
    if (s_dmafill) {
        s_dmafill = 0;
        s_dma_length = dma_length_reg();
        dma_update(g_xstart);
    }
}

static uint16_t vdp_data_r(void)
{
    GVDP *v = V;
    uint16_t data = 0;
    s_pending = 0;
    switch (v->code & 0x1F) {
    case 0x00: {
        uint16_t a = (uint16_t)(v->address & 0xFFFE);
        data = (uint16_t)((v->vram[a] << 8) | v->vram[(uint16_t)(a + 1)]);
        break;
    }
    case 0x04: {
        unsigned idx = (v->address & 0x7E) >> 1;
        if (idx >= 40) idx = 0;
        data = (uint16_t)((v->vsram[idx] & 0x7FF) | (s_fifo[s_fifo_idx] & 0xF800));
        break;
    }
    case 0x08:
        data = (uint16_t)((v->cram[(v->address >> 1) & 0x3F] & 0x0EEE) | (s_fifo[s_fifo_idx] & ~0x0EEE));
        break;
    case 0x0C: {
        uint8_t b = v->vram[(uint16_t)(v->address ^ 1)];
        data = (uint16_t)((s_fifo[s_fifo_idx] & 0xFF00) | b);
        break;
    }
    default:
        break;
    }
    v->address = (uint16_t)(v->address + REG(15));
    return data;
}

static uint16_t hv_counter(uint32_t now);

static void vdp_reg_w(unsigned r, uint8_t d, uint32_t now)
{
    if (!(REG(1) & 4) && r > 10) return;          /* mode 4: #11-#23 locked */
    switch (r) {
    case 0: {
        uint8_t ch = (uint8_t)(d ^ REG(0));
        REG(0) = d;
        if ((ch & 0x10) && s_hint_pending) {
            if (s_vint_pending & REG(1))  set_irq(6);
            else if (d & 0x10)            set_irq_delay(4);
            else                          set_irq(0);
        }
        if ((ch & 0x02) && (REG(1) & 0x04))
            s_hvc_latch = (d & 0x02) ? (hv_counter(now) | 0x10000u) : 0;
        break;
    }
    case 1: {
        uint8_t ch = (uint8_t)(d ^ REG(1));
        REG(1) = d;
        if ((ch & 0x20) && s_vint_pending) {
            if (d & 0x20)                         set_irq_delay(6);
            else if (s_hint_pending & REG(0))     set_irq(4);
            else                                  set_irq(0);
        }
        if ((ch & 0x08) && (d & 0x04)) s_view_changed |= 2;
        if (ch & 0x04) {
            if (d & 0x04) {
                if (s_status & 8) s_view_changed |= 2;
                else              s_view_h = 224 + ((d & 8) << 1);
                s_hvc_latch = 0;
                if (REG(0) & 0x02) s_hvc_latch = hv_counter(now) | 0x10000u;
            } else {
                if (s_status & 8) s_view_changed |= 2;
                else              s_view_h = 192;
                s_hvc_latch = hv_counter(now) | 0x10000u;
            }
        }
        break;
    }
    default:
        if (r < GVDP_NUM_REGISTERS) REG(r) = d;
        break;
    }
}

static void vdp_ctrl_w(uint16_t data)
{
    GVDP *v = V;
    if (!s_pending) {
        if (s_dma_length && s_dma_type < 2) {     /* 68K DMA still running */
            s_cached_write = data;
            return;
        }
        if ((data & 0xC000) == 0x8000) vdp_reg_w((data >> 8) & 0x1F, (uint8_t)data, g_xstart);
        else                           s_pending = (uint8_t)(REG(1) & 4);
        v->address = (uint16_t)(s_addr_latch | (data & 0x3FFF));
        v->code = (uint8_t)((v->code & 0x3C) | ((data >> 14) & 0x03));
        return;
    }
    s_pending = 0;
    s_addr_latch = (uint16_t)((data & 3) << 14);
    v->address = (uint16_t)(s_addr_latch | (v->address & 0x3FFF));
    v->code = (uint8_t)((v->code & 0x03) | ((data >> 2) & 0x3C));
    if ((v->code & 0x20) && (REG(1) & 0x10)) {
        switch (REG(23) >> 6) {
        case 2:                                   /* fill: waits for a data write */
            s_dma_type = 2;
            s_dmafill = 1;
            s_status |= 0x02;
            s_dma_end = 0xFFFFFFFFu;
            break;
        case 3:                                   /* VRAM copy */
            s_dma_type = 3;
            s_dma_length = dma_length_reg();
            s_dma_src = ((uint32_t)REG(22) << 8) | REG(21);
            dma_update(g_xstart);
            break;
        default:                                  /* 68K bus -> VDP */
            s_dma_type = (v->code & 0x06) ? 0 : 1;
            s_dma_length = dma_length_reg();
            s_dma_src = ((uint32_t)REG(22) << 8) | REG(21);
            dma_update(g_xstart);
            break;
        }
    }
}

static uint16_t vdp_ctrl_r(uint32_t now)
{
    if (s_fifo_write_cnt) fifo_update(now);
    if ((s_status & 2) && !s_dma_length && now >= s_dma_end) s_status &= 0xFFFD;
    uint16_t temp = s_status;
    s_pending = 0;
    s_status &= 0xFF9F;
    if (!(REG(1) & 0x40)) temp |= 0x08;
    if ((now % MPL) < 588) temp |= 0x04;
    if (g_machine.vdp.sprite_overflow)  temp |= 0x40;
    if (g_machine.vdp.sprite_collision) temp |= 0x20;
    g_machine.vdp.sprite_overflow = g_machine.vdp.sprite_collision = 0;
    return temp;
}

uint16_t exact_peek_status(void) { return s_status; }

/* H counter: approximate until a reference trace exercises it (TODO). */
static uint16_t hv_counter(uint32_t now)
{
    uint32_t data = s_hvc_latch;
    if (data) {
        if (REG(1) & 0x04) return (uint16_t)(data & 0xFFFF);
        data &= 0xFF;
    } else {
        uint32_t c = now % MPL;
        data = (REG(12) & 1) ? (c * 210u / MPL) : (c * 171u / MPL);
    }
    int vc = (int)(now / MPL) - 1;
    int vc_max = (REG(1) & 0x04) ? ((REG(1) & 0x08) ? 0x10A : 0xEA) : 0xDA;
    if (vc > vc_max) vc -= LINES;
    return (uint16_t)(data | ((vc & 0xFF) << 8));
}

/* ======================================================================== */
/* I/O                                                                       */

static uint8_t pad_read(int port)
{
    uint8_t data = (uint8_t)((s_pad_state[port] & 0x40) | 0x3F);
    uint16_t g = g_machine.bus.pad[port];                 /* GPAD_* bits    */
    /* GPAD layout -> the reference's: X and Z trade places in bits 8-10. */
    uint32_t val = (g & 0x0FFu) | ((g & GPAD_X) ? 0x400 : 0) | ((g & GPAD_Y) ? 0x200 : 0)
                 | ((g & GPAD_Z) ? 0x100 : 0) | ((g & GPAD_MODE) ? 0x800 : 0);
    unsigned step = (unsigned)(s_pad_counter[port] | ((data >> 6) & 1));
    int six = g_machine.bus.pad_type[port] == 1;
    if (!six) step &= 1;
    switch (step) {
    case 1: case 3: case 5:
        data &= (uint8_t)~(val & 0x3F); break;
    case 0: case 2:
        data &= (uint8_t)~(val & 0x03); data &= (uint8_t)~((val >> 2) & 0x30); data &= (uint8_t)~0x0C; break;
    case 4:
        data &= (uint8_t)~((val >> 2) & 0x30); data &= (uint8_t)~0x0F; break;
    case 6:
        data &= (uint8_t)~((val >> 2) & 0x30); break;
    case 7:
        data &= (uint8_t)~(val & 0x30); data &= (uint8_t)~((val >> 8) & 0x0F); break;
    }
    return data;
}

static void pad_write(int port, uint8_t data, uint8_t mask)
{
    data = (uint8_t)((s_pad_state[port] & ~mask) | (data & mask));
    if (g_machine.bus.pad_type[port] == 1 && !(s_pad_state[port] & 0x40) && (data & 0x40)) {
        s_pad_counter[port] = (uint8_t)((s_pad_counter[port] + 2) & 6);
        s_pad_timeout[port] = 0;
    }
    s_pad_state[port] = data;
}

static void input_refresh(void)
{
    for (int p = 0; p < 2; p++)
        if (g_machine.bus.pad_type[p] == 1 && s_pad_timeout[p]++ > 25) {
            s_pad_counter[p] = 0;
            s_pad_timeout[p] = 0;
        }
}

static uint8_t io_read(unsigned off)
{
    switch (off) {
    case 0x01: case 0x02: case 0x03: {
        uint8_t mask = (uint8_t)(0x80 | s_io_reg[off + 3]);
        uint8_t data = off == 3 ? 0x7F : pad_read((int)off - 1);
        return (uint8_t)((s_io_reg[off] & mask) | (data & ~mask));
    }
    default:
        return s_io_reg[off];
    }
}

static void io_write(unsigned off, uint8_t data)
{
    switch (off) {
    case 0x01: case 0x02: case 0x03:
        s_io_reg[off] = data;
        if (off < 3) pad_write((int)off - 1, data, s_io_reg[off + 3]);
        return;
    case 0x04: case 0x05: case 0x06:
        if (data != s_io_reg[off]) {
            s_io_reg[off] = data;
            if (off < 6) pad_write((int)off - 4, s_io_reg[off - 3], data);
        }
        return;
    case 0x07: case 0x0A: case 0x0D:
        s_io_reg[off] = data; return;
    case 0x09: case 0x0C: case 0x0F:
        s_io_reg[off] = (uint8_t)(data & 0xF8); return;
    default:
        return;
    }
}

/* ======================================================================== */
/* 68000 bus                                                                 */

static int vdp_port(uint32_t a) { return a >= 0xC00000u && a < 0xC00020u; }

/* Unused data-bus bits read back as the word the 68000 prefetched: the one
 * after the current instruction (all reads of interest are its last access). */
#include "m68k_decoder.h"
static uint16_t open_bus16(void)
{
    static GenesisRom rom;
    M68KInstr in;
    uint32_t next = g_xpc + 2;
    if (!rom.rom_data) { rom.rom_data = g_rom; rom.rom_size = 0x400000u; }
    if (m68k_decode(&rom, g_xpc & 0x3FFFFFu, &in)) next = g_xpc + in.byte_length;
    next &= 0x3FFFFFu;
    return (uint16_t)((g_rom[next] << 8) | g_rom[next + 1]);
}

uint16_t xbus_read16(uint32_t a)
{
    a &= 0xFFFFFFu;
    if (a >= 0xFF0000u) return (uint16_t)((g_ram[a & 0xFFFF] << 8) | g_ram[(a + 1) & 0xFFFF]);
    if (a < 0x400000u)  return (uint16_t)((g_rom[a] << 8) | g_rom[a + 1]);
    if (vdp_port(a)) {
        switch (a & 0x1C) {
        case 0x00: return vdp_data_r();
        case 0x04: return (uint16_t)((vdp_ctrl_r(g_xstart) & 0x3FF) | (open_bus16() & 0xFC00));
        case 0x08: case 0x0C: return hv_counter(g_xstart);
        default: return 0xFFFF;
        }
    }
    if (a >= 0xA00000u && a < 0xA10000u) {
        if (!s_zaccess) return open_bus16();                 /* Z80 owns its bus */
        uint8_t b;
        if ((a & 0x6000) == 0x4000) b = exact_ym_read(g_xstart);
        else if ((a & 0x6000) == 0x6000) b = 0xFF;
        else b = g_machine.bus.z80_ram[a & 0x1FFF];
        return (uint16_t)(b << 8 | b);
    }
    if (a >= 0xA10000u && a < 0xA10020u) { uint8_t b = io_read((a >> 1) & 0x0F); return (uint16_t)(b << 8 | b); }
    if (a == 0xA11100u) return (uint16_t)(s_zstate == 3 ? (open_bus16() & 0xFEFF) : (open_bus16() | 0x0100));
    return gbus_read16(&g_machine.bus, a);
}

uint8_t xbus_read8(uint32_t a)
{
    a &= 0xFFFFFFu;
    if (a >= 0xFF0000u) return g_ram[a & 0xFFFF];
    if (a < 0x400000u)  return g_rom[a];
    if (a >= 0xA00000u && a < 0xA10000u) {
        if (!s_zaccess) { uint16_t w = open_bus16(); return (a & 1) ? (uint8_t)w : (uint8_t)(w >> 8); }
        if ((a & 0x6000) == 0x4000) return exact_ym_read(g_xstart);
        if ((a & 0x6000) == 0x6000) return 0xFF;
        return g_machine.bus.z80_ram[a & 0x1FFF];
    }
    if (a >= 0xA10000u && a < 0xA10020u) return io_read((a >> 1) & 0x0F);
    if (a == 0xA11100u) { uint8_t b = (uint8_t)(open_bus16() >> 8); return (uint8_t)(s_zstate == 3 ? (b & 0xFE) : (b | 0x01)); }
    uint16_t w = xbus_read16(a & ~1u);
    return (a & 1) ? (uint8_t)w : (uint8_t)(w >> 8);
}

void xbus_write16(uint32_t a, uint16_t v)
{
    a &= 0xFFFFFFu;
    if (a >= 0xFF0000u) { g_ram[a & 0xFFFF] = (uint8_t)(v >> 8); g_ram[(a + 1) & 0xFFFF] = (uint8_t)v; return; }
    if (vdp_port(a)) {
        switch (a & 0x1C) {
        case 0x00: vdp_data_w(v); return;
        case 0x04: vdp_ctrl_w(v); return;
        case 0x10: case 0x14: xbus_write8(a | 1, (uint8_t)v); return;   /* PSG */
        default: return;
        }
    }
    if (a >= 0xA00000u && a < 0xA10000u) { xbus_write8(a, (uint8_t)(v >> 8)); return; }
    if (a >= 0xA10000u && a < 0xA10020u) { io_write((a >> 1) & 0x0F, (uint8_t)v); return; }
    if (a == 0xA11100u) { z80_busreq_w((v >> 8) & 1, g_xstart); return; }
    if (a == 0xA11200u) { z80_reset_w((v >> 8) & 1, g_xstart); return; }
    gbus_write16(&g_machine.bus, a, v);
}

void xbus_write8(uint32_t a, uint8_t v)
{
    a &= 0xFFFFFFu;
    if (a >= 0xFF0000u) { g_ram[a & 0xFFFF] = v; return; }
    if (a >= 0xA00000u && a < 0xA10000u) {
        if (!s_zaccess) return;
        if ((a & 0x6000) == 0x0000 || (a & 0x6000) == 0x2000) {
            g_machine.bus.z80_ram[a & 0x1FFF] = v;
            g_xstart += 8; g_xclk += 8;                       /* Z80 RAM access latency */
            return;
        }
        if ((a & 0x6000) == 0x4000) exact_ym_write(g_xstart, a & 3, v);
        gbus_write8(&g_machine.bus, a, v);                     /* YM2612 audio / bank */
        return;
    }
    if (a >= 0xA10000u && a < 0xA10020u) { if (a & 1) io_write((a >> 1) & 0x0F, v); return; }
    if (a == 0xA11100u) { z80_busreq_w(v & 1, g_xstart); return; }
    if (a == 0xA11200u) { z80_reset_w(v & 1, g_xstart); return; }
    if (vdp_port(a)) {
        if ((a & 0x1C) == 0x10 || (a & 0x1C) == 0x14) { gbus_write8(&g_machine.bus, 0xC00011u, v); return; }
        xbus_write16(a & ~1u, (uint16_t)(v << 8 | v));
        return;
    }
    gbus_write8(&g_machine.bus, a, v);
}

/* ======================================================================== */
/* YM2612 timers and status                                                  */
/* The chip runs in samples of 144 68000 cycles (1008 master clocks) on a fixed
 * grid; a status read or data-port write first catches it up to the first
 * sample boundary at or after the access (whole samples). Timer A counts
 * samples, timer B is charged per catch-up in one step; status bits 0/1 are
 * the timers' overflow flags (no busy flag). Audio stays the engine's ymfm
 * path: this is only the CPU-visible timing. */
#define FM_SAMPLE 1008u
static uint32_t s_fm_count;              /* master clock of the next sample   */
static int      s_fm_ta, s_fm_tal = 1024, s_fm_tac, s_fm_tbl = 256 << 4, s_fm_tbc;
static uint8_t  s_fm_mode, s_fm_status;
static uint16_t s_fm_addr;

static void fm_update(uint32_t t)
{
    if (t <= s_fm_count) return;
    uint32_t n = (t - s_fm_count + FM_SAMPLE - 1) / FM_SAMPLE;
    for (uint32_t i = 0; i < n; i++) {
        if (s_fm_mode & 0x01) {
            if (--s_fm_tac <= 0) {
                if (s_fm_mode & 0x04) s_fm_status |= 0x01;
                s_fm_tac = s_fm_tal;
            }
        }
    }
    if (s_fm_mode & 0x02) {
        s_fm_tbc -= (int)n;
        if (s_fm_tbc <= 0) {
            if (s_fm_mode & 0x08) s_fm_status |= 0x02;
            if (s_fm_tbl) s_fm_tbc += s_fm_tbl; else s_fm_tbc = s_fm_tbl;
        }
    }
    s_fm_count += n * FM_SAMPLE;
}

static void fm_set_timers(uint8_t v)
{
    if ((v & 1) && !(s_fm_mode & 1)) s_fm_tac = s_fm_tal;
    if ((v & 2) && !(s_fm_mode & 2)) s_fm_tbc = s_fm_tbl;
    s_fm_status &= (uint8_t)(~v >> 4);
    s_fm_mode = v;
}

static void fm_reset_chip(uint32_t t)
{
    fm_update(t);
    s_fm_tac = s_fm_tbc = 0;
    fm_set_timers(0x30);
    s_fm_tbl = 256 << 4;
    s_fm_ta = 0;
    s_fm_tal = 1024;
}

uint8_t exact_ym_read(uint32_t t)
{
    fm_update(t);
    return s_fm_status;
}

void exact_ym_write(uint32_t t, unsigned port, uint8_t v)
{
    if (port & 1) fm_update(t);
    switch (port & 3) {
    case 0: s_fm_addr = v; break;
    case 2: s_fm_addr = (uint16_t)(v | 0x100); break;
    default:
        switch (s_fm_addr) {
        case 0x24: s_fm_ta = (s_fm_ta & 0x03) | (v << 2); s_fm_tal = 1024 - s_fm_ta; break;
        case 0x25: s_fm_ta = (s_fm_ta & 0x3FC) | (v & 3); s_fm_tal = 1024 - s_fm_ta; break;
        case 0x26: s_fm_tbl = (256 - v) << 4; break;
        case 0x27: fm_set_timers(v); break;
        default: break;
        }
        break;
    }
}

/* ======================================================================== */
/* Frame                                                                     */

void exact_reset(void)
{
    g_xclk = g_xstart = 40u * MCLK_68K;   /* reset exception */
    g_xend = 0;
    s_end = 0;
    s_mline = 0;
    s_int_level = 0; s_irq_latency = 0;
    s_status = 0x200;
    s_hint_pending = s_vint_pending = 0;
    s_vcounter = LINES - 1;
    s_view_h = 192; s_view_changed = 0;
    s_pending = 0; s_addr_latch = 0; s_cached_write = -1;
    s_hvc_latch = 0x10000;
    s_dma_length = 0; s_dma_type = 0; s_dma_end = 0; s_dmafill = 0; s_dma_src = 0;
    memset(s_fifo, 0, sizeof s_fifo); s_fifo_idx = 0;
    s_fifo_write_cnt = s_fifo_slots = 0; s_fifo_byte_access = 1; s_fifo_cycles = 0;
    memset(s_io_reg, 0, sizeof s_io_reg);
    s_io_reg[0x00] = 0xA0;                /* overseas NTSC, no expansion unit */
    s_io_reg[0x07] = 0xFF; s_io_reg[0x0A] = 0xFF; s_io_reg[0x0D] = 0xFB;
    s_pad_state[0] = s_pad_state[1] = 0x40;
    s_pad_counter[0] = s_pad_counter[1] = 0;
    s_pad_timeout[0] = s_pad_timeout[1] = 0;
    s_zclk = 0; s_zstate = 0; s_zirq = 0; s_zaccess = 0;
    s_fm_count = 0; s_fm_ta = 0; s_fm_tal = 1024; s_fm_tac = 0; s_fm_tbl = 256 << 4; s_fm_tbc = 0;
    s_fm_mode = 0x30; s_fm_status = 0; s_fm_addr = 0;
    { extern void machine_z80_exact_power_on(void); machine_z80_exact_power_on(); }
    g_machine.bus.z80_busreq = 0; g_machine.bus.z80_reset_off = 0;
    memset(g_machine.vdp.reg, 0, sizeof g_machine.vdp.reg);
    g_machine.vdp.code = 0; g_machine.vdp.address = 0;
}

static void hint_tick(int *h_counter)
{
    if (--*h_counter < 0) {
        *h_counter = REG(10);
        s_hint_pending = 0x10;
        if (REG(0) & 0x10) update_irq(4);
    }
}

static void run_line_end(uint32_t until)
{
    m68k_run(until);
    z80_run(until);
}

void exact_run_frame(GenesisScanlineSink sink, void *user)
{
    extern void machine_render_line(int line, GenesisScanlineSink sink, void *user);
    GVDP *v = V;
    int line = 0;
    int h_counter = REG(10);
    int zirq = 1;

    s_frame_no++;
    trace_frame_begin();
    { extern void machine_z80_trace_frame(unsigned long long); machine_z80_trace_frame(s_frame_no); }
    s_mline = 0;
    s_vcounter = LINES - 1;
    s_fifo_write_cnt = 0;
    s_fifo_slots = 0;
    input_refresh();
    if (s_view_changed & 2) {
        s_view_changed &= ~2;
        s_view_h = (REG(1) & 0x04) ? ((REG(1) & 0x08) ? 240 : 224) : 192;
    }
    s_status &= 0xFEE5;
    s_status |= 0x0200;
    if (s_dma_length) dma_update_at(0, 0);

    run_line_end(MPL);
    s_mline += MPL;

    do {                                           /* active display */
        s_vcounter = line;
        input_refresh();
        hint_tick(&h_counter);
        if (s_dma_length) dma_update_at(s_mline, 0);
        machine_render_line(line, sink, user);
        run_line_end(s_mline + MPL);
        s_mline += MPL;
    } while (++line < s_view_h);

    s_vcounter = line;
    s_status |= 0x08;                              /* vblank */
    v->in_vblank = 1;
    input_refresh();
    hint_tick(&h_counter);
    if (s_dma_length) dma_update_at(s_mline, 0);
    m68k_run(s_mline + 588);
    s_status |= 0x80;                              /* VINT flag */
    run_line_end(s_mline + 788);
    s_vint_pending = 0x20;
    if (REG(1) & 0x20) set_irq(6);
    s_zirq = 1; machine_z80_exact_irq(1);
    run_line_end(s_mline + MPL);
    s_mline += MPL;
    line++;

    do {                                           /* vertical blanking */
        s_vcounter = line;
        input_refresh();
        if (zirq) {
            run_line_end(s_mline + 788);
            s_zirq = 0; machine_z80_exact_irq(0);
            zirq = 0;
        }
        run_line_end(s_mline + MPL);
        s_mline += MPL;
    } while (++line < LINES - 1);

    v->in_vblank = 0;
    fm_update(s_mline);                            /* the frame's last FM samples */
    s_fm_count -= s_mline;
    g_xclk   -= s_mline;
    g_xstart -= s_mline;
    s_zclk   -= s_mline;
    if (s_dma_end != 0xFFFFFFFFu) s_dma_end = s_dma_end > s_mline ? s_dma_end - s_mline : 0;
    if (s_fifo_cycles > s_mline) s_fifo_cycles -= s_mline; else s_fifo_cycles = 0;
}
