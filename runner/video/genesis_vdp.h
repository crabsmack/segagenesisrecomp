/*
 * genesis_vdp.h — clean-room Sega Genesis / Mega Drive VDP (315-5313).
 *
 * AGPL-free runtime initiative, teardown item #4 (see LICENSING.md). This is an
 * ORIGINAL implementation written from documented hardware behaviour — the VDP
 * register map, the control-port address/code mechanism, DMA semantics, the
 * plane/sprite rendering rules, and H/V interrupt timing are all public
 * hardware facts (Sega's VDP documentation, the genvdp/Charles MacDonald notes,
 * and hardware test ROMs). NO code is copied or translated from any other
 * emulator: not clownmdemu (AGPL), not Genesis Plus GX (non-commercial), not
 * BlastEm/jgenesis (GPL). Permissively-licensed emulators (ares ISC, MAME
 * BSD-3) may be consulted ONLY as a last-resort reference for an obscure
 * hardware detail; their expression is not reproduced here.
 *
 * Output model matches what the runner's display path already consumes
 * (main.c colour_updated_cb / scanline_rendered_cb): per scanline we emit a row
 * of palette INDICES (0..63 + brightness), and CRAM writes raise colour-updated
 * events. The recompiled 68K and superzazu Z80 drive register writes through the
 * bus; this module owns only the VDP.
 */
#ifndef GENESIS_VDP_H
#define GENESIS_VDP_H

#include <stdint.h>
#include <stddef.h>

/* ---- Hardware constants --------------------------------------------------- */
#define GVDP_VRAM_SIZE      0x10000            /* 64 KB byte-addressed         */
#define GVDP_CRAM_ENTRIES   64                 /* 4 palette lines * 16 colours */
#define GVDP_VSRAM_ENTRIES  40                 /* 40 scroll words (16-bit MD)  */
#define GVDP_NUM_REGISTERS  24                 /* $00..$17                     */
#define GVDP_MAX_WIDTH      512                /* H40 320 + up to 96px widescreen margin/side */
#define GVDP_MAX_HEIGHT     240                /* V30 active lines (PAL)       */
#define GVDP_SPRITES_MAX    80                 /* H40 sprite table entries     */

/* CRAM index space the scanline callback emits: 64 colours * 3 brightnesses
 * (normal / shadow / highlight). The display path maps these to ARGB. */
#define GVDP_PALETTE_NORMAL     0
#define GVDP_PALETTE_SHADOW     64
#define GVDP_PALETTE_HIGHLIGHT  128
#define GVDP_TOTAL_PALETTE      192

/* Widescreen pillarbox bar sentinel: a synthetic index one past the real
 * palette that the display path maps to a fixed bar colour (opaque black).
 * Emitted by gvdp_render_scanline for pillarbox columns ONLY when black-bar
 * mode is selected; backdrop-bar mode emits the real backdrop index instead,
 * so this stays branch-free in the index->ARGB conversion. The host sizes its
 * ARGB cache to GVDP_OUTPUT_PALETTE_SIZE and seeds this slot with black. */
#define GVDP_WS_BAR_INDEX       GVDP_TOTAL_PALETTE

/* Optional presentation colors, separate from CRAM and the bar sentinel. */
#define GVDP_HOST_PALETTE_BASE  (GVDP_WS_BAR_INDEX + 1)
#define GVDP_HOST_PALETTE_SIZE  (256 - GVDP_HOST_PALETTE_BASE)
#define GVDP_OUTPUT_PALETTE_SIZE 256

/* ---- Callbacks the VDP needs from its host ------------------------------- */
/* DMA 68k->VRAM/CRAM/VSRAM must read source words from the main bus. The host
 * supplies a reader over the 68K address space (word reads). */
typedef uint16_t (*GVDP_BusReadWord)(void *user, uint32_t address_68k);

/* Raised when a CRAM entry changes, so the host can recompute its ARGB cache
 * (mirrors clownmdemu's colour_updated callback). value = 9-bit BGR (0xBGR). */
typedef void (*GVDP_ColourUpdated)(void *user, unsigned index, uint16_t bgr9);

/* ---- VDP state ------------------------------------------------------------ */
typedef struct GVDP {
    /* Memories */
    uint8_t  vram[GVDP_VRAM_SIZE];
    uint16_t cram[GVDP_CRAM_ENTRIES];          /* stored as 9-bit BGR          */
    /* The VDP's internal sprite attribute cache (68000 byte order, like vram):
     * each sprite's Y, size and link are read from here, name/attributes and X
     * from VRAM. Written by every VRAM write that lands in the current table
     * (gvdp_sat_byte); not refreshed when register 5 moves the table. */
    uint8_t  sat[0x400];
    /* Sprite-pixel markers in the 32 columns either side of the screen
     * ([0,32): x = -32..-1; [32,64): x = width..width+31). They persist from
     * line to line where the background pass doesn't rewrite them, as in the
     * reference's bordered sprite line buffer, and sprites partly on screen
     * collide with them (sprite_render_line). */
    uint8_t  spr_edge[64];
    /* Sprite masking carried into the next line: the line hit the sprite pixel
     * limit, so an x==0 sprite masks from the start of the next line. */
    uint8_t  spr_mask_carry;
    uint8_t  spr_reserved[7];
    uint16_t vsram[GVDP_VSRAM_ENTRIES];

    /* Register file ($00..$17). */
    uint8_t  reg[GVDP_NUM_REGISTERS];

    /* Control-port address/code FSM. The control port takes two 16-bit writes;
     * together they form a 5-bit code (read/write target + DMA) and a 16-bit
     * (plus 2 high) address that auto-increments by reg[15] after each access. */
    uint8_t  control_pending;                  /* second-write latch           */
    uint8_t  code;                             /* 6-bit CD0..CD5               */
    uint16_t address;                          /* current VRAM/CRAM/VSRAM addr */
    uint8_t  address_hi;                        /* CD bits / 128KB addr extend  */

    /* Status register bits (assembled on read). */
    uint8_t  in_vblank;
    uint8_t  in_hblank;
    uint8_t  dma_active;
    uint8_t  sprite_overflow;
    uint8_t  sprite_collision;
    uint8_t  vint_pending;                     /* status bit 7 (F flag)        */

    /* Interrupt timing. */
    uint16_t hint_counter;                     /* counts down reg[10] per line */
    uint16_t scanline;                         /* current raster line          */

    /* Pending DMA (fill/copy) state, serviced incrementally. */
    uint8_t  dma_fill_pending;

    /* Host hooks. */
    GVDP_BusReadWord   bus_read;
    void              *bus_read_user;
    GVDP_ColourUpdated colour_updated;
    void              *colour_updated_user;
} GVDP;

/* ---- Lifecycle ------------------------------------------------------------ */
void gvdp_init(GVDP *v);
void gvdp_reset(GVDP *v);
/* Draw the complete SAT without per-scanline sprite/pixel dropout. Sprite
 * ordering, priority and deliberate X=0 masking remain intact. Default off. */
void gvdp_set_unlimited_sprites(int enabled);
/* Simulation, not presentation: it decides when sprite evaluation raises the
 * status register's overflow flag. Part of the rollback machine section. */
int  gvdp_unlimited_sprites(void);

/* Host sprites: additive actors a game draws from its own pattern data into
 * the sprite layer after every SAT sprite, so native sprites keep precedence
 * and plane priority, CRAM (including mid-frame palette swaps) and
 * shadow/highlight apply exactly as for hardware sprites. `line`, `total` and
 * `offset` match the current output row (offset = output column of screen
 * column 0). Write a pixel only where opaque[x] is 0. Presentation only: the
 * callback must not mutate the guest, and never raises overflow/collision.
 * For independent colors, opt in with gvdp_set_host_palette below. */
typedef struct GVDPSpriteLayer {
    int line, total, offset;
    uint8_t *index;   /* CRAM 0..63, or GVDP_HOST_PALETTE_BASE + host slot */
    uint8_t *opaque;
    uint8_t *high;    /* sprite priority bit */
} GVDPSpriteLayer;
typedef void (*GVDPHostSprites)(void *user, const struct GVDP *v,
                                const GVDPSpriteLayer *layer);
void gvdp_set_host_sprites(GVDPHostSprites draw, void *user);
/* Copy up to GVDP_HOST_PALETTE_SIZE final ARGB colors. NULL clears the
 * palette; unused slots are opaque black. Host colors obey sprite/plane
 * priority but bypass hardware shadow/highlight (the caller owns shading).
 * No guest CRAM, machine state or color callbacks are changed. Unregistering
 * the host sprite callback clears these colors too. Returns 0 for oversize. */
int gvdp_set_host_palette(const uint32_t *argb, unsigned count);
/* Read-only color table for index-to-ARGB sinks; NULL when disabled. */
const uint32_t *gvdp_host_palette(void);

/* ---- 68K port interface ($C00000 data, $C00004 control) ------------------ */
void     gvdp_write_data   (GVDP *v, uint16_t value);
void     gvdp_write_control(GVDP *v, uint16_t value);
uint16_t gvdp_read_data    (GVDP *v);
uint16_t gvdp_read_control (GVDP *v);          /* status register             */

/* H/V counter read ($C00008). */
uint16_t gvdp_read_hv_counter(const GVDP *v);

/* Side-effect-free views of the two stateful ports, for HOST inspection
 * (debuggers, scripts, rollback digests) only — never for emulated 68K
 * accesses. peek_data returns the word the next data-port read would return
 * without advancing the address or resetting the control FSM; peek_status
 * returns the status word without clearing the V-int flag or advancing the
 * phantom H-blank toggle that gvdp_read_control performs. */
uint16_t gvdp_peek_data  (const GVDP *v);
uint16_t gvdp_peek_status(const GVDP *v);

/* Fetch-and-clear the 68K freeze cycles owed by the last 68K->VDP DMA
 * (hardware freezes the 68K for the whole transfer; fill/copy run in
 * background and don't stall). Kept OUTSIDE the GVDP struct: it is transient
 * (consumed right after the triggering port write) and the save states
 * snapshot GVDP as a raw struct image — a new field would break existing
 * save files. */
uint32_t gvdp_rb_pending_stall(void);
void     gvdp_rb_set_pending_stall(uint32_t v);
uint32_t gvdp_consume_68k_stall(GVDP *v);
/* Raw VRAM (odd address = byte-swapped word) / CRAM / VSRAM stores, no
   auto-increment: the exact scheduler's port layer (runner/exact_sched.c). */
void gvdp_store_vram_word(GVDP *v, uint16_t addr, uint16_t value);
/* A byte just written to VRAM at `addr` (68000 byte order): mirror it into the
 * sprite attribute cache when it lands in the current table. Every VRAM write
 * path calls it (word stores do it themselves). */
void gvdp_sat_byte(GVDP *v, uint16_t addr, uint8_t b);
/* Parse the sprites of output row `line` one line ahead (sets the overflow
 * status bit); the exact scheduler parses row 0 at the frame's last line. */
void gvdp_sprite_parse(GVDP *v, int line);
void gvdp_store_cram(GVDP *v, uint16_t addr, uint16_t value);
void gvdp_store_vsram(GVDP *v, uint16_t addr, uint16_t value);

/* ---- Always-on VDP event ring ---------------------------------------------
 * Port-level flight recorder: every register write, completed control
 * command, DMA execution/decline, and direct data-port write. Always
 * compiled in (Release too); probes query backward via the debug server
 * (cmd `vdp_events`) — never arm-then-record. Ring storage is file-static
 * in genesis_vdp.c (exposed via these globals) so the GVDP struct and the
 * raw save-state image keep their layout. */
enum {
    GVDP_EVT_REG          = 1,   /* code=reg index, value=reg value           */
    GVDP_EVT_CMD          = 2,   /* completed control pair: code/addr/inc     */
    GVDP_EVT_DATA         = 3,   /* data-port write: code/addr/value/inc      */
    GVDP_EVT_DMA_68K      = 4,   /* executed 68k->VDP DMA: addr/inc/len/src   */
    GVDP_EVT_DMA_FILL_ARM = 5,   /* VRAM fill armed                           */
    GVDP_EVT_DMA_COPY     = 6,   /* executed VRAM copy                        */
    GVDP_EVT_DMA_SKIP     = 7    /* DMA-bit command declined (see reason)     */
};
enum {
    GVDP_SKIP_DMA_OFF      = 1,  /* reg1 M1 (DMA enable) clear                */
    GVDP_SKIP_NO_BUSREAD   = 2,  /* no bus_read hook wired                    */
    GVDP_SKIP_FILL_TRIGGER = 3   /* DATA event doubled as a fill trigger      */
};
typedef struct {
    uint32_t seq;
    uint8_t  kind;               /* GVDP_EVT_*                                */
    uint8_t  code;               /* VDP code bits (or reg index for REG)      */
    uint8_t  reason;             /* GVDP_SKIP_* where applicable              */
    uint8_t  in_vblank;
    uint16_t scanline;
    uint16_t addr;               /* VDP address at the event                  */
    uint16_t value;              /* data word / reg value / cmd second half   */
    uint16_t inc;                /* auto-increment (reg 15) at the event      */
    uint16_t len;                /* DMA length in words                       */
    uint32_t src;                /* DMA source (68K byte address)             */
} GVdpEvent;
#define GVDP_EVENT_CAP 16384
extern GVdpEvent g_gvdp_events[GVDP_EVENT_CAP];
extern uint32_t  g_gvdp_event_seq;

/* ---- Geometry / mode queries (from registers) ----------------------------- */
int gvdp_screen_width (const GVDP *v);          /* 256 (H32) or 320 (H40)      */
int gvdp_screen_height(const GVDP *v);          /* 224 (V28) or 240 (V30) — RASTER lines */

/* ---- Widescreen (16:9) opt-in --------------------------------------------- */
/* Set the per-side extra pixels rendered around the authentic active window.
 * 0 (default) = authentic 4:3; the value is clamped to the output buffer.
 * NOT part of VDP state (excluded from raw-struct save images). */
void gvdp_set_ws_extra(int extra_px);
/* Arm a FIXED 16:9 output canvas (in pixels) used for EVERY frame while
 * widescreen is on, decoupled from the per-frame content margin set by
 * gvdp_set_ws_extra(). Gameplay frames fill the canvas with widened content;
 * non-gameplay frames (menus/title/intro) center the authentic 320/256 view
 * and pillarbox the sides, so the output width — and thus the window — never
 * changes. 0 (default) disarms: output reverts to screen_width + 2*extra.
 * NOT part of VDP state (excluded from raw-struct save images). */
void gvdp_set_ws_canvas(int canvas_w);
/* Pillarbox bar colour: 0 = per-scanline backdrop colour (seamless, looks like
 * one wider background); 1 = opaque black (classic letterbox). The host picks
 * the user-facing default (see GENESIS_WS_BARS in main.c). */
void gvdp_set_ws_bar_black(int black);
/* Active output width including widescreen margins (the armed canvas when set,
 * else screen_width + 2*extra). */
int  gvdp_active_width(const GVDP *v);
/* In widescreen, the per-line sprite pixel budget: 1 = the authentic screen
 * width (320 in H40, the hardware's limit, so a widened view drops exactly the
 * sprites the 4:3 view drops for the same table), 0 (default) = the output
 * width (the margins and pillarbox widen the budget). No effect without a
 * margin or canvas. Presentation parameter, outside the VDP struct. */
void gvdp_set_ws_authentic_sprite_budget(int on);

/* Widescreen margin columns for 2-cell (per-16-px column) vertical scroll.
 * VSRAM holds the columns of the authentic window only (20 in H40, 16 in H32);
 * a margin column is drawn with the nearest edge column's value (the clamp)
 * unless the host fills it here. Columns are numbered in screen space,
 * col = floor(x / 16): -1 is x -16..-1, 20 is x 320..335 in H40 (16 in H32),
 * so a 40-px margin draws columns -3..-1 and 20..22. Values are VSRAM words
 * (plane A, plane B; 11 bits kept). A filled column keeps its value until set
 * again or cleared, like VSRAM: a host that widens an H-int raster sets it
 * between lines where the raster rewrites VSRAM (the scanline renders after
 * the line's interrupt work). Presentation only, outside the VDP struct.
 * Returns 0 (nothing set) for col outside GVDP_WS_VCOL_FIRST..LAST. */
#define GVDP_WS_VCOL_SIDE   8          /* per side: the widest margin (128 px, H32) / 16 */
#define GVDP_WS_VCOL_FIRST  (-GVDP_WS_VCOL_SIDE)                 /* -8 */
#define GVDP_WS_VCOL_LAST   (320 / 16 - 1 + GVDP_WS_VCOL_SIDE)   /* 27 */
int  gvdp_set_ws_vscroll_column(int col, uint16_t vs_a, uint16_t vs_b);
void gvdp_clear_ws_vscroll_column(int col);
void gvdp_clear_ws_vscroll_columns(void);

/* Host sprite table: entries a host draws past the hardware's 80 (64 in H32),
 * as if linked after the SAT chain's last entry, in array order (the link
 * byte is ignored). Same rules as SAT sprites: Y/size/name/attributes/X in the
 * SAT's own format (y and x include the +128 origin; 10-bit x as above),
 * priority, shadow/highlight operators, the x=0 masking state carried from
 * the SAT sprites of the same line, and the per-line count and pixel limits
 * unless gvdp_set_unlimited_sprites is on (a line the SAT already filled
 * draws none). Drawn into the hardware sprite layer before the host-sprite
 * callback. Never raises the overflow or collision status bits (presentation
 * only). The list is copied; it stays until set again (count 0 clears).
 * Returns 0 (list unchanged) for count > GVDP_HOST_SAT_MAX. */
#define GVDP_HOST_SAT_MAX 256
typedef struct GVDPHostSat {
    uint16_t y;        /* SAT word 0: y + 128 (+256 in interlace mode 2)     */
    uint8_t  size;     /* SAT byte 2: (width-1) << 2 | (height-1), in cells   */
    uint8_t  link;     /* SAT byte 3: ignored (array order is link order)    */
    uint16_t attr;     /* SAT word 2: priority, palette, flips, pattern      */
    uint16_t x;        /* SAT word 3: x + 128                                 */
} GVDPHostSat;
int      gvdp_set_host_sat(const GVDPHostSat *list, unsigned count);
unsigned gvdp_host_sat_count(void);

/* Render counters (presentation, cumulative since the last reset; outside
 * the VDP struct). A sprite "on a line" is one whose rows cover it, counted
 * in link order over the SAT chain then the host table, on screen or not (off
 * screen sprites take part in the hardware limits too). */
typedef struct GVDPStats {
    uint64_t sprite_lines;            /* sprite lines evaluated                     */
    uint64_t sprite_drop_lines;       /* lines on which a limit dropped a sprite    */
    uint64_t sprite_drops_count;      /* sprites on a line past the 20th (16th H32) */
    uint64_t sprite_drops_pixels;     /* sprites within the count cut short or not
                                         drawn because the line's pixel budget ran out */
    uint64_t host_sat_sprites;        /* host-table sprites drawn on a line         */
    uint64_t ws_margin_columns;       /* 2-cell V-scroll margin columns drawn, per line */
    uint64_t ws_edge_clamped_columns; /* of those, drawn with the edge clamp (not filled) */
} GVDPStats;
void gvdp_get_stats(GVDPStats *out);
void gvdp_reset_stats(void);

/* Diagnostic: force Plane B hscroll constant (line 0) + mark its wrap column. */
void gvdp_set_bgdiag(int on);
int gvdp_display_enabled(const GVDP *v);
int gvdp_interlace_double(const GVDP *v);       /* 1 = interlace mode 2        */
int gvdp_output_height(const GVDP *v);          /* raster height x2 in IM2     */

/* ---- Rendering ------------------------------------------------------------ */
/* Render one active scanline into `out` as palette indices (see GVDP_PALETTE_*
 * for shadow/highlight offsets). `out` must hold at least gvdp_screen_width()
 * entries. Returns the number of pixels written. */
int gvdp_render_scanline(GVDP *v, int line, uint8_t *out);
const char *gvdp_plane_implementation(void);

/* ---- Per-frame raster timing ---------------------------------------------- */
/* Called by the scheduler at the start of each scanline; advances the H-int
 * counter and sets/clears V-int pending at the vblank boundary. Returns a mask
 * of interrupts that should fire this line. */
enum { GVDP_IRQ_HBLANK = 1, GVDP_IRQ_VBLANK = 2 };
unsigned gvdp_begin_scanline(GVDP *v, int line);

#endif /* GENESIS_VDP_H */
