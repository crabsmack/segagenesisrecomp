/* The phase-6 presentation extensions of genesis_vdp.c, each with its control:
 * on, a planted scene renders differently; off (or cleared), byte-identical.
 *   - widescreen 2-cell V-scroll margin columns (gvdp_set_ws_vscroll_column)
 *   - the host sprite table past the SAT (gvdp_set_host_sat)
 *   - the per-line drop counters (gvdp_get_stats)
 *   - the authentic sprite pixel budget in widescreen
 * Build with GENESIS_BATCHED_PLANES=0 and =1 (both plane paths). */
#include "genesis_vdp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

static GVDP v;
static uint8_t row[GVDP_MAX_WIDTH], ref[GVDP_MAX_WIDTH];

static void wr(unsigned a, unsigned n) { gvdp_store_vram_word(&v, (uint16_t)a, (uint16_t)n); }

/* H40, display on, plane A $C000, B $E000, SAT $F800, H-scroll $FC00, 64x32 planes.
 * Tile t (1..15) is solid colour t. */
static void base(void)
{
    gvdp_init(&v);
    v.reg[1] = 0x40; v.reg[2] = 0x30; v.reg[4] = 0x07; v.reg[5] = 0x7C;
    v.reg[12] = 0x01; v.reg[13] = 0x3F; v.reg[16] = 0x01;
    for (int t = 1; t < 16; t++)
        memset(v.vram + t * 32, (t << 4) | t, 32);
    gvdp_set_ws_extra(0); gvdp_set_ws_canvas(0); gvdp_set_unlimited_sprites(0);
    gvdp_set_ws_authentic_sprite_budget(0); gvdp_clear_ws_vscroll_columns();
    gvdp_set_host_sat(NULL, 0); gvdp_reset_stats();
}

static void sprite(int i, int sx, int sy, int size, int next, unsigned attr)
{
    unsigned a = 0xF800 + i * 8;
    wr(a, (unsigned)(sy + 128)); wr(a + 2, (unsigned)((size << 8) | next));
    wr(a + 4, attr); wr(a + 6, (unsigned)(sx + 128));
}

static int render(int line) { memset(row, 0xEE, sizeof row); return gvdp_render_scanline(&v, line, row); }

static void test_margin_columns(void)
{
    base();
    v.reg[11] = 0x04;                                   /* 2-cell V-scroll, full H-scroll */
    for (int r = 0; r < 32; r++)                        /* plane A cell row r = tile 2 + (r & 7) */
        for (int c = 0; c < 64; c++) wr(0xC000 + (r * 64 + c) * 2, 2 + (r & 7));
    v.vsram[0] = 0;                                     /* column 0: row 0 -> colour 2 */
    v.vsram[38] = 16;                                   /* column 19: row 2 -> colour 4 */
    for (int c = 1; c < 19; c++) v.vsram[c * 2] = 8;

    /* 4:3: margin columns are never drawn, filled or not */
    CHECK(render(0) == 320); memcpy(ref, row, 320);
    gvdp_set_ws_vscroll_column(-1, 24, 0);
    gvdp_set_ws_vscroll_column(20, 24, 0);
    CHECK(render(0) == 320 && !memcmp(ref, row, 320));
    GVDPStats st; gvdp_get_stats(&st);
    CHECK(st.ws_margin_columns == 0 && st.ws_edge_clamped_columns == 0);
    gvdp_clear_ws_vscroll_columns();

    /* 40-px margins, nothing filled: the clamp (the left edge column's 2, the right's 4) */
    gvdp_set_ws_extra(40);
    CHECK(render(0) == 400);
    for (int x = 0; x < 40; x++) CHECK(row[x] == 2);
    for (int x = 360; x < 400; x++) CHECK(row[x] == 4);
    CHECK(row[40] == 2 && row[359] == 4);
    memcpy(ref, row, 400);
    gvdp_get_stats(&st);
    CHECK(st.ws_margin_columns == 6 && st.ws_edge_clamped_columns == 6);  /* -3..-1, 20..22 */

    /* filled: column -1 (x -16..-1 = output 24..39) and 20..22 (output 360..399) */
    CHECK(gvdp_set_ws_vscroll_column(-1, 24, 0));        /* row 3 -> colour 5 */
    for (int c = 20; c <= 22; c++) CHECK(gvdp_set_ws_vscroll_column(c, 8, 0));
    CHECK(!gvdp_set_ws_vscroll_column(GVDP_WS_VCOL_LAST + 1, 8, 0));
    CHECK(!gvdp_set_ws_vscroll_column(GVDP_WS_VCOL_FIRST - 1, 8, 0));
    gvdp_reset_stats();
    render(0);
    for (int x = 0; x < 24; x++) CHECK(row[x] == 2);
    for (int x = 24; x < 40; x++) CHECK(row[x] == 5);
    for (int x = 40; x < 360; x++) CHECK(row[x] == ref[x]);   /* the 4:3 view untouched */
    for (int x = 360; x < 400; x++) CHECK(row[x] == 3);
    gvdp_get_stats(&st);
    CHECK(st.ws_margin_columns == 6 && st.ws_edge_clamped_columns == 2); /* -3, -2 */

    /* per line: the host rewrites a column between lines (an H-int raster) */
    gvdp_set_ws_vscroll_column(-3, 32, 0);              /* row 4 -> colour 6 at x -40..-33 */
    render(1);
    for (int x = 0; x < 8; x++) CHECK(row[x] == 6);
    CHECK(row[8] == 2);

    /* cleared: byte-identical to the clamp again (the control) */
    gvdp_clear_ws_vscroll_columns();
    render(0);
    CHECK(!memcmp(ref, row, 400));
    gvdp_set_ws_vscroll_column(-1, 24, 0);
    gvdp_clear_ws_vscroll_column(-1);
    render(0);
    CHECK(!memcmp(ref, row, 400));

    /* full-screen V-scroll mode ignores the columns */
    v.reg[11] = 0x00;
    render(0); memcpy(ref, row, 400);
    gvdp_set_ws_vscroll_column(-1, 24, 0);
    gvdp_reset_stats();
    render(0);
    CHECK(!memcmp(ref, row, 400));
    gvdp_get_stats(&st);
    CHECK(st.ws_margin_columns == 0);
    puts("margin columns: clamp by default, filled columns drawn per line, cleared = clamp");
}

static void test_host_sat(void)
{
    GVDPStats st;
    GVDPHostSat hs[3];
    base();
    gvdp_set_unlimited_sprites(1);
    /* a full SAT: 80 8x8 sprites on line 0, x 0..158, colour 1 */
    for (int i = 0; i < 80; i++) sprite(i, i * 2, 0, 0, i + 1 < 80 ? i + 1 : 0, 0x0001);
    render(0); memcpy(ref, row, 320);
    CHECK(row[0] == 1 && row[165] == 1 && row[200] == 0);

    /* one host sprite past the 80th: colour 2 (palette 0, tile 2) at x 200 */
    hs[0] = (GVDPHostSat){ 128, 0, 0, 0x0002, 128 + 200 };
    CHECK(gvdp_set_host_sat(hs, 1) && gvdp_host_sat_count() == 1);
    gvdp_reset_stats();
    render(0);
    for (int x = 200; x < 208; x++) CHECK(row[x] == 2);
    for (int x = 0; x < 200; x++) CHECK(row[x] == ref[x]);
    gvdp_get_stats(&st);
    CHECK(st.host_sat_sprites == 1);
    render(9);                                          /* not on its rows */
    CHECK(row[200] == 0);

    /* behind every SAT sprite (link order): a host sprite at x 10 stays colour 1 */
    hs[0].x = 128 + 10;
    gvdp_set_host_sat(hs, 1);
    render(0);
    CHECK(row[10] == 1 && row[17] == 1);
    /* only SAT sprite 0 (x 0..7) on the line, the host sprite over it at x 4 */
    for (int i = 1; i < 80; i++) sprite(i, 200 + i * 8, 100, 0, i + 1 < 80 ? i + 1 : 0, 0x0001);
    hs[0].x = 128 + 4;
    gvdp_set_host_sat(hs, 1);
    v.sprite_collision = 0;
    render(0);
    CHECK(row[7] == 1 && row[8] == 2 && row[11] == 2 && row[12] == 0);
    CHECK(!v.sprite_collision);                         /* the host's overlap raises nothing */
    sprite(1, 4, 0, 0, 2, 0x0001);
    render(0);
    CHECK(v.sprite_collision);                          /* control: SAT over SAT */

    /* masking: an x=0 host sprite after an x!=0 sprite masks the rest of the line */
    for (int i = 0; i < 80; i++) sprite(i, i * 2, 0, 0, i + 1 < 80 ? i + 1 : 0, 0x0001);
    hs[0] = (GVDPHostSat){ 128, 0, 0, 0x0002, 128 + 200 };
    hs[1] = (GVDPHostSat){ 128, 0, 0, 0x0003, 0 };      /* raw x 0 */
    hs[2] = (GVDPHostSat){ 128, 0, 0, 0x0003, 128 + 240 };
    gvdp_set_host_sat(hs, 3);
    render(0);
    CHECK(row[200] == 2 && row[240] == 0);
    hs[1].x = 128 + 300;                                /* not a mask: drawn */
    gvdp_set_host_sat(hs, 3);
    render(0);
    CHECK(row[240] == 3 && row[300] == 3);

    /* limits on: a line the SAT filled draws no host sprite; one short of it draws one */
    gvdp_set_unlimited_sprites(0);
    for (int i = 0; i < 20; i++) sprite(i, i * 8, 0, 0, i + 1 < 20 ? i + 1 : 0, 0x0001);
    hs[0] = (GVDPHostSat){ 128, 0, 0, 0x0002, 128 + 200 };
    gvdp_set_host_sat(hs, 1);
    gvdp_reset_stats();
    render(0);
    CHECK(row[200] == 0);
    gvdp_get_stats(&st);
    CHECK(st.sprite_drops_count == 1 && st.host_sat_sprites == 0);
    sprite(19, 152, 0, 0, 0, 0x0001);
    sprite(18, 144, 0, 0, 0, 0x0001);                   /* 19 SAT sprites on the line */
    render(0);
    CHECK(row[200] == 2);
    CHECK(!v.sprite_overflow);

    /* cleared: byte-identical to no host table (the control) */
    gvdp_set_host_sat(NULL, 0);
    render(0); memcpy(ref, row, 320);
    CHECK(row[200] == 0);
    CHECK(!gvdp_set_host_sat(hs, GVDP_HOST_SAT_MAX + 1) && gvdp_host_sat_count() == 0);
    puts("host table: drawn past the SAT, behind it, masked and limited as SAT sprites, no status bits");
}

static void test_drop_counters(void)
{
    GVDPStats st;
    base();
    /* 21 8-px sprites on line 0: one dropped by the count */
    for (int i = 0; i < 21; i++) sprite(i, i * 8, 0, 0, i + 1 < 21 ? i + 1 : 0, 0x0001);
    render(0);
    CHECK(row[160] == 0 && row[159] == 1);
    gvdp_get_stats(&st);
    CHECK(st.sprite_lines == 1 && st.sprite_drop_lines == 1 && st.sprite_drops_count == 1 &&
          st.sprite_drops_pixels == 0);
    /* 25: five dropped */
    for (int i = 20; i < 25; i++) sprite(i, i * 8, 0, 0, i + 1 < 25 ? i + 1 : 0, 0x0001);
    gvdp_reset_stats(); render(0); gvdp_get_stats(&st);
    CHECK(st.sprite_drops_count == 5 && st.sprite_drop_lines == 1);
    render(50); gvdp_get_stats(&st);                    /* an empty line counts no drop */
    CHECK(st.sprite_lines == 2 && st.sprite_drop_lines == 1);
    gvdp_set_unlimited_sprites(1);
    gvdp_reset_stats(); render(0); gvdp_get_stats(&st);
    CHECK(row[160] == 1 && st.sprite_drops_count == 0 && st.sprite_drop_lines == 0);
    gvdp_set_unlimited_sprites(0);

    /* 11 32-px sprites: 320 px after ten, the eleventh dropped by the pixel budget */
    base();
    for (int i = 0; i < 11; i++) sprite(i, i * 28, 0, 0x0C, i + 1 < 11 ? i + 1 : 0, 0x0001 + (i == 10));
    render(0); gvdp_get_stats(&st);
    CHECK(row[256] == 1 && row[283] == 4 && row[290] == 0); /* sprite 9 (x 252..283, tiles 1-4) drawn, 10 not */
    CHECK(st.sprite_drops_pixels == 1 && st.sprite_drops_count == 0);
    /* the first 24 px wide: the eleventh is cut to one cell (still a drop) */
    sprite(0, 0, 0, 0x08, 1, 0x0001);
    gvdp_reset_stats(); render(0); gvdp_get_stats(&st);
    CHECK(row[283] == 4 && row[285] == 2 && row[288] == 0);  /* x 280..287, behind sprite 9 */
    CHECK(st.sprite_drops_pixels == 1);
    puts("drop counters: count and pixel drops counted, none with the limits lifted");
}

static void test_authentic_budget(void)
{
    GVDPStats st;
    base();
    /* 11 32-px sprites, the eleventh (colour 2) at x 330 */
    for (int i = 0; i < 10; i++) sprite(i, i * 32, 0, 0x0C, i + 1, 0x0001);
    sprite(10, 330, 0, 0x0C, 0, 0x0002);
    render(0); memcpy(ref, row, 320);
    gvdp_set_ws_authentic_sprite_budget(1);             /* no margin: no effect */
    render(0);
    CHECK(!memcmp(ref, row, 320));
    gvdp_set_ws_authentic_sprite_budget(0);

    gvdp_set_ws_extra(40);                              /* the output-width budget: 400 px */
    gvdp_reset_stats(); render(0); gvdp_get_stats(&st);
    CHECK(row[40 + 330] == 2 && st.sprite_drops_pixels == 0);
    gvdp_set_ws_authentic_sprite_budget(1);             /* 320 px, as the 4:3 view */
    gvdp_reset_stats(); render(0); gvdp_get_stats(&st);
    CHECK(row[40 + 330] == 0 && st.sprite_drops_pixels == 1);
    for (int x = 0; x < 320; x++) CHECK(row[40 + x] == ref[x]);   /* the centre = the 4:3 line */
    gvdp_set_ws_authentic_sprite_budget(0);
    gvdp_set_ws_extra(0);
    puts("authentic budget: a widened line drops what the 4:3 line drops");
}

int main(void)
{
    printf("planes: %s\n", gvdp_plane_implementation());
    test_margin_columns();
    test_host_sat();
    test_drop_counters();
    test_authentic_budget();
    puts("vdp_p6_test passed");
    return 0;
}
