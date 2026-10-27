/*
 * input_script.c — see input_script.h for the format spec.
 *
 * Implementation is a step-machine over a pre-parsed instruction
 * array. Each tick processes instructions until one blocks (WAIT,
 * WAIT_RAM*) or the array runs out. ASSERTs that fail call the
 * crash-report dump and exit. Held-button mask survives tick
 * boundaries until explicitly RELEASEd.
 */

#include "input_script.h"
#include "crash_report.h"
#include "genesis_runtime.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/* Case-insensitive compare: _stricmp is the MSVC CRT spelling; on POSIX
 * the equivalent is strcasecmp from <strings.h>. */
#ifndef _WIN32
#include <strings.h>
#define _stricmp strcasecmp
#endif

/* ---- Op codes ---- */

typedef enum {
    OP_WAIT,
    OP_HOLD,
    OP_RELEASE,        /* mask 0xFFFF means RELEASE_ALL */
    OP_PADS,           /* set every listed player's whole pad mask */
    OP_PRESS,          /* arg = button, count = frames to hold */
    OP_ASSERT_RAM8,
    OP_WAIT_RAM8,
    OP_ASSERT_RAM16,
    OP_WAIT_RAM16,
    OP_WRITE_RAM8,
    OP_WRITE_RAM16,
    OP_WRITE_RAM32,
    OP_SCREENSHOT,
    OP_SAVE_STATE,
    OP_LOAD_STATE,
    OP_DUMP_RAM,
    OP_DUMP_VRAM,
    OP_EXIT,
    OP_VPAD,           /* one more frame's pads for this frame's queue (frame-synchronous runners) */
} OpCode;

typedef struct {
    OpCode    op;
    uint32_t  arg32;       /* WAIT count, RAM addr, EXIT code */
    uint32_t  arg32b;      /* RAM expected value, PRESS frames */
    uint16_t  button_mask; /* HOLD/RELEASE/PRESS button bit */
    uint16_t  pads[4];     /* PADS: masks for players 1..arg32 */
    int       line_no;     /* source line for error messages */
    char      str[260];    /* path/string argument */
} ScriptOp;

/* ---- State ---- */

#define SCRIPT_PLAYERS 4
#define SCRIPT_BUTTONS 12
/* Grows on demand: a whole-game input stream (PADS/WAIT per input change) runs to
 * tens of thousands of directives. */
static ScriptOp *s_ops;
static int      s_op_count = 0;
static int      s_op_cap   = 0;
static int      s_pc       = 0;     /* next op to execute */
static int      s_active   = 0;
static uint64_t s_wait_until_frame = 0;
static uint16_t s_held_mask[SCRIPT_PLAYERS];
static uint8_t  s_players_used[SCRIPT_PLAYERS];
static unsigned s_parse_player;
static int      s_exit_pending     = 0;
static int      s_exit_code        = 0;
static char     s_pending_screenshot[260];
static char     s_pending_save_state[260];
static char     s_pending_load_state[260];
static char     s_pending_ram_dump[260];
static char     s_pending_vram_dump[260];

/* One deadline per button/player: presses cannot exhaust a small timer pool. */
static uint64_t s_press_until[SCRIPT_PLAYERS][SCRIPT_BUTTONS];

/* VPAD p1 p2 directives of the current frame, in order: a frame-synchronous runner plays one
 * runner frame per logical frame of the game, and a frame that spans several reference frames
 * (a pause loop polling the pads the V-int refreshes) gets their pads one per V-int it waits for
 * (the game's spec takes them: input_script_vpad). Emptied at every tick. */
static uint16_t *s_vq;
static uint32_t  s_vq_n, s_vq_cap;
static uint64_t  s_tick_frame;   /* the frame of the last tick (input_script_peek) */

/* ---- Button name parsing ---- */

static int parse_button(const char *s, uint16_t *out) {
    if      (!_stricmp(s, "UP"))    *out = 1u << 0;
    else if (!_stricmp(s, "DOWN"))  *out = 1u << 1;
    else if (!_stricmp(s, "LEFT"))  *out = 1u << 2;
    else if (!_stricmp(s, "RIGHT")) *out = 1u << 3;
    else if (!_stricmp(s, "B"))     *out = 1u << 4;
    else if (!_stricmp(s, "C"))     *out = 1u << 5;
    else if (!_stricmp(s, "A"))     *out = 1u << 6;
    else if (!_stricmp(s, "START")) *out = 1u << 7;
    else if (!_stricmp(s, "X"))     *out = 1u << 8;   /* 6-button extras (GPAD_*) */
    else if (!_stricmp(s, "Y"))     *out = 1u << 9;
    else if (!_stricmp(s, "Z"))     *out = 1u << 10;
    else if (!_stricmp(s, "MODE"))  *out = 1u << 11;
    else return 0;
    return 1;
}

/* ---- Parser ---- */

static int parse_line(char *line, int line_no) {
    /* Strip comment + trailing whitespace */
    char *hash = strchr(line, '#');
    if (hash) *hash = '\0';
    char *end = line + strlen(line);
    while (end > line && isspace((unsigned char)end[-1])) *--end = '\0';
    char *p = line;
    while (*p && isspace((unsigned char)*p)) p++;
    if (*p == '\0') return 1;     /* blank line OK */

    if (s_op_count >= s_op_cap) {
        int cap = s_op_cap ? s_op_cap * 2 : 1024;
        ScriptOp *grown = (ScriptOp *)realloc(s_ops, (size_t)cap * sizeof *s_ops);
        if (!grown) {
            fprintf(stderr, "input_script: out of memory at %d directives\n", s_op_count);
            return 0;
        }
        s_ops = grown;
        s_op_cap = cap;
    }
    ScriptOp *o = &s_ops[s_op_count];
    o->line_no = line_no;
    o->str[0]  = '\0';

    char tok[64], a1[260], a2[260];
    int n = sscanf(p, "%63s %259s %259s", tok, a1, a2);
    if (n < 1) return 1;

    if (!_stricmp(tok, "PLAYER")) {
        if (n != 2 || strlen(a1) != 1 || a1[0] < '1' || a1[0] > '4') goto bad;
        s_parse_player = (unsigned)(a1[0] - '1');
        s_players_used[s_parse_player] = 1;
        return 1;
    }

    if (!_stricmp(tok, "PADS")) {
        /* PADS <p1> [<p2> [<p3> [<p4>]]]: whole held masks, hex, GPAD_* bit layout
         * (a recorded input stream sets every pad each time anything changes). */
        char pv[4][32];
        int np = sscanf(p, "%63s %31s %31s %31s %31s", tok, pv[0], pv[1], pv[2], pv[3]) - 1;
        if (np < 1) goto bad;
        o->op = OP_PADS;
        o->arg32 = (uint32_t)np;
        for (int i = 0; i < np; i++) {
            char *e;
            unsigned long v = strtoul(pv[i], &e, 16);
            if (*e || v > 0xFFFu) goto bad;
            o->pads[i] = (uint16_t)v;
            s_players_used[i] = 1;
        }
    } else if (!_stricmp(tok, "VPAD")) {
        /* VPAD <p1> [<p2>]: hex masks, appended to the frame's queue */
        char pv[2][32];
        int np = sscanf(p, "%63s %31s %31s", tok, pv[0], pv[1]) - 1;
        if (np < 1) goto bad;
        o->op = OP_VPAD;
        for (int i = 0; i < 2; i++) {
            char *e;
            unsigned long v = i < np ? strtoul(pv[i], &e, 16) : 0;
            if ((i < np && *e) || v > 0xFFFu) goto bad;
            o->pads[i] = (uint16_t)v;
        }
    } else if (!_stricmp(tok, "WAIT") && n >= 2) {
        o->op    = OP_WAIT;
        o->arg32 = (uint32_t)strtoul(a1, NULL, 0);
    } else if (!_stricmp(tok, "HOLD") && n >= 2) {
        if (!parse_button(a1, &o->button_mask)) goto bad;
        o->op = OP_HOLD;
        o->arg32 = s_parse_player;
        s_players_used[s_parse_player] = 1;
    } else if (!_stricmp(tok, "RELEASE")) {
        o->op = OP_RELEASE;
        o->arg32 = s_parse_player;
        if (n >= 2) {
            if (!parse_button(a1, &o->button_mask)) goto bad;
        } else {
            o->button_mask = 0xFFFF;  /* sentinel: release all */
        }
    } else if (!_stricmp(tok, "PRESS") && n >= 3) {
        if (!parse_button(a1, &o->button_mask)) goto bad;
        o->op     = OP_PRESS;
        o->arg32 = s_parse_player;
        s_players_used[s_parse_player] = 1;
        o->arg32b = (uint32_t)strtoul(a2, NULL, 0);
    } else if ((!_stricmp(tok, "ASSERT_RAM8") || !_stricmp(tok, "WAIT_RAM8")) && n >= 3) {
        o->op     = !_stricmp(tok, "ASSERT_RAM8") ? OP_ASSERT_RAM8 : OP_WAIT_RAM8;
        o->arg32  = (uint32_t)strtoul(a1, NULL, 16);
        o->arg32b = (uint32_t)strtoul(a2, NULL, 16);
    } else if ((!_stricmp(tok, "ASSERT_RAM16") || !_stricmp(tok, "WAIT_RAM16")) && n >= 3) {
        o->op     = !_stricmp(tok, "ASSERT_RAM16") ? OP_ASSERT_RAM16 : OP_WAIT_RAM16;
        o->arg32  = (uint32_t)strtoul(a1, NULL, 16);
        o->arg32b = (uint32_t)strtoul(a2, NULL, 16);
    } else if (!_stricmp(tok, "WRITE_RAM8") && n >= 3) {
        o->op     = OP_WRITE_RAM8;
        o->arg32  = (uint32_t)strtoul(a1, NULL, 16);
        o->arg32b = (uint32_t)strtoul(a2, NULL, 16);
    } else if (!_stricmp(tok, "WRITE_RAM16") && n >= 3) {
        o->op     = OP_WRITE_RAM16;
        o->arg32  = (uint32_t)strtoul(a1, NULL, 16);
        o->arg32b = (uint32_t)strtoul(a2, NULL, 16);
    } else if (!_stricmp(tok, "WRITE_RAM32") && n >= 3) {
        o->op     = OP_WRITE_RAM32;
        o->arg32  = (uint32_t)strtoul(a1, NULL, 16);
        o->arg32b = (uint32_t)strtoul(a2, NULL, 16);
    } else if (!_stricmp(tok, "SCREENSHOT")) {
        o->op = OP_SCREENSHOT;
        if (n >= 2) snprintf(o->str, sizeof(o->str), "%s", a1);
        else        snprintf(o->str, sizeof(o->str), "screenshot.png");
    } else if (!_stricmp(tok, "SAVE_STATE") && n >= 2) {
        o->op = OP_SAVE_STATE;
        snprintf(o->str, sizeof(o->str), "%s", a1);
    } else if (!_stricmp(tok, "LOAD_STATE") && n >= 2) {
        o->op = OP_LOAD_STATE;
        snprintf(o->str, sizeof(o->str), "%s", a1);
    } else if (!_stricmp(tok, "DUMP_RAM") && n >= 2) {
        o->op = OP_DUMP_RAM;
        snprintf(o->str, sizeof(o->str), "%s", a1);
    } else if (!_stricmp(tok, "DUMP_VRAM") && n >= 2) {
        o->op = OP_DUMP_VRAM;
        snprintf(o->str, sizeof(o->str), "%s", a1);
    } else if (!_stricmp(tok, "EXIT")) {
        o->op    = OP_EXIT;
        o->arg32 = (n >= 2) ? (uint32_t)strtoul(a1, NULL, 0) : 0u;
    } else {
        goto bad;
    }
    s_op_count++;
    return 1;

bad:
    fprintf(stderr, "input_script: parse error at line %d: %s\n", line_no, p);
    return 0;
}

int input_script_load(const char *path) {
    s_op_count = 0;
    s_pc = 0;
    s_active = 0;
    s_wait_until_frame = 0;
    memset(s_held_mask, 0, sizeof s_held_mask);
    memset(s_players_used, 0, sizeof s_players_used);
    s_parse_player = 0;
    s_exit_pending = 0;
    s_pending_screenshot[0] = '\0';
    s_pending_save_state[0] = '\0';
    s_pending_load_state[0] = '\0';
    s_pending_ram_dump[0] = '\0';
    s_pending_vram_dump[0] = '\0';
    memset(s_press_until, 0, sizeof s_press_until);
    if (!path) return 0;

    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "input_script: cannot open '%s'\n", path);
        return 0;
    }
    char line[512];
    int line_no = 0;
    while (fgets(line, sizeof(line), f)) {
        line_no++;
        if (!parse_line(line, line_no)) { fclose(f); return 0; }
    }
    fclose(f);
    s_active = 1;
    fprintf(stderr, "input_script: loaded %d directives from %s\n", s_op_count, path);
    return 1;
}

void input_script_tick(uint64_t frame,
                       input_script_read8_fn  r8,
                       input_script_read16_fn r16,
                       input_script_write8_fn  w8,
                       input_script_write16_fn w16,
                       input_script_write32_fn w32)
{
    if (!s_active) return;

    s_vq_n = 0;   /* a frame's VPAD queue is its own */
    s_tick_frame = frame;

    /* Auto-release any PRESS that has timed out. */
    for (unsigned p = 0; p < SCRIPT_PLAYERS; ++p) {
        for (unsigned b = 0; b < SCRIPT_BUTTONS; ++b) {
            if (s_press_until[p][b] && frame >= s_press_until[p][b]) {
                s_held_mask[p] &= (uint16_t)~(1u << b);
                s_press_until[p][b] = 0;
            }
        }
    }

    /* Honor outstanding WAIT. */
    if (frame < s_wait_until_frame) return;

    /* Drain instructions until we block. */
    while (s_pc < s_op_count) {
        ScriptOp *o = &s_ops[s_pc];
        switch (o->op) {
            case OP_WAIT:
                s_wait_until_frame = frame + o->arg32;
                s_pc++;
                if (frame < s_wait_until_frame) return;
                break;

            case OP_VPAD:
                if (s_vq_n == s_vq_cap) {
                    uint32_t cap = s_vq_cap ? s_vq_cap * 2 : 64;
                    uint16_t *g = (uint16_t *)realloc(s_vq, (size_t)cap * 2 * sizeof *g);
                    if (!g) { s_pc++; break; }
                    s_vq = g;
                    s_vq_cap = cap;
                }
                s_vq[2 * s_vq_n] = o->pads[0];
                s_vq[2 * s_vq_n + 1] = o->pads[1];
                s_vq_n++;
                s_pc++;
                break;

            case OP_PADS:
                for (uint32_t i = 0; i < o->arg32; ++i) {
                    s_held_mask[i] = o->pads[i];
                    for (unsigned b = 0; b < SCRIPT_BUTTONS; ++b) s_press_until[i][b] = 0;
                }
                s_pc++;
                break;

            case OP_HOLD:
                s_held_mask[o->arg32] |= o->button_mask;
                for (unsigned b = 0; b < SCRIPT_BUTTONS; ++b)
                    if (o->button_mask & (1u << b)) s_press_until[o->arg32][b] = 0;
                s_pc++;
                break;

            case OP_RELEASE:
                s_held_mask[o->arg32] &= (uint16_t)~o->button_mask;
                for (unsigned b = 0; b < SCRIPT_BUTTONS; ++b)
                    if (o->button_mask & (1u << b)) s_press_until[o->arg32][b] = 0;
                s_pc++;
                break;

            case OP_PRESS: {
                for (unsigned b = 0; b < SCRIPT_BUTTONS; ++b) {
                    if (!(o->button_mask & (1u << b))) continue;
                    if (o->arg32b) s_held_mask[o->arg32] |= o->button_mask;
                    else s_held_mask[o->arg32] &= (uint16_t)~o->button_mask;
                    s_press_until[o->arg32][b] = o->arg32b ? frame + o->arg32b : 0;
                }
                s_pc++;
                break;
            }

            case OP_ASSERT_RAM8: {
                uint8_t v = r8 ? r8(o->arg32) : 0;
                if (v != (uint8_t)o->arg32b) {
                    char reason[160];
                    snprintf(reason, sizeof(reason),
                             "input_script line %d: ASSERT_RAM8 $%06X expected $%02X got $%02X",
                             o->line_no, o->arg32, (uint8_t)o->arg32b, v);
                    crash_report_dump_persistent(reason, &g_cpu, o->arg32, 0, frame);
                    exit(3);
                }
                s_pc++;
                break;
            }

            case OP_WAIT_RAM8: {
                uint8_t v = r8 ? r8(o->arg32) : 0;
                if (v != (uint8_t)o->arg32b) return;   /* keep blocking */
                s_pc++;
                break;
            }

            case OP_ASSERT_RAM16: {
                uint16_t v = r16 ? r16(o->arg32) : 0;
                if (v != (uint16_t)o->arg32b) {
                    char reason[160];
                    snprintf(reason, sizeof(reason),
                             "input_script line %d: ASSERT_RAM16 $%06X expected $%04X got $%04X",
                             o->line_no, o->arg32, (uint16_t)o->arg32b, v);
                    crash_report_dump_persistent(reason, &g_cpu, o->arg32, 0, frame);
                    exit(3);
                }
                s_pc++;
                break;
            }

            case OP_WAIT_RAM16: {
                uint16_t v = r16 ? r16(o->arg32) : 0;
                if (v != (uint16_t)o->arg32b) return;
                s_pc++;
                break;
            }

            case OP_WRITE_RAM8:
                if (w8) w8(o->arg32, (uint8_t)o->arg32b);
                fprintf(stderr, "[input_script] WRITE_RAM8 at frame %llu ($%06X=$%02X)\n",
                        (unsigned long long)frame, o->arg32, (uint8_t)o->arg32b);
                s_pc++;
                break;

            case OP_WRITE_RAM16:
                if (w16) w16(o->arg32, (uint16_t)o->arg32b);
                fprintf(stderr, "[input_script] WRITE_RAM16 at frame %llu ($%06X=$%04X)\n",
                        (unsigned long long)frame, o->arg32, (uint16_t)o->arg32b);
                s_pc++;
                break;

            case OP_WRITE_RAM32:
                if (w32) w32(o->arg32, (uint32_t)o->arg32b);
                fprintf(stderr, "[input_script] WRITE_RAM32 at frame %llu ($%06X=$%08X)\n",
                        (unsigned long long)frame, o->arg32, (uint32_t)o->arg32b);
                s_pc++;
                break;

            case OP_SCREENSHOT:
                snprintf(s_pending_screenshot, sizeof(s_pending_screenshot), "%s", o->str);
                /* Wired by the main loop next frame — see input_script_pending_screenshot. */
                fprintf(stderr, "[input_script] SCREENSHOT requested at frame %llu (path=%s)\n",
                        (unsigned long long)frame, o->str);
                s_pc++;
                break;

            case OP_SAVE_STATE:
                snprintf(s_pending_save_state, sizeof(s_pending_save_state), "%s", o->str);
                fprintf(stderr, "[input_script] SAVE_STATE requested at frame %llu (path=%s)\n",
                        (unsigned long long)frame, o->str);
                s_pc++;
                break;

            case OP_LOAD_STATE:
                snprintf(s_pending_load_state, sizeof(s_pending_load_state), "%s", o->str);
                fprintf(stderr, "[input_script] LOAD_STATE requested at frame %llu (path=%s)\n",
                        (unsigned long long)frame, o->str);
                s_pc++;
                break;

            case OP_DUMP_RAM:
                snprintf(s_pending_ram_dump, sizeof(s_pending_ram_dump), "%s", o->str);
                fprintf(stderr, "[input_script] DUMP_RAM requested at frame %llu (path=%s)\n",
                        (unsigned long long)frame, o->str);
                s_pc++;
                break;

            case OP_DUMP_VRAM:
                snprintf(s_pending_vram_dump, sizeof(s_pending_vram_dump), "%s", o->str);
                fprintf(stderr, "[input_script] DUMP_VRAM requested at frame %llu (path=%s)\n",
                        (unsigned long long)frame, o->str);
                s_pc++;
                break;

            case OP_EXIT:
                s_exit_pending = 1;
                s_exit_code    = (int)o->arg32;
                fprintf(stderr, "[input_script] EXIT %d at frame %llu\n",
                        s_exit_code, (unsigned long long)frame);
                s_pc++;
                return;
        }
    }
}

uint32_t input_script_vpad_count(void) { return s_vq_n; }
uint16_t input_script_vpad(uint32_t i, int player)
{
    return i < s_vq_n && player >= 0 && player < 2 ? s_vq[2 * i + (uint32_t)player] : 0;
}

/* The pads of the n-th reference frame after the current frame's own (its first and its VPAD
 * queue), reading the script ahead without running it: the rest of the current WAIT, then each
 * later group `PADS x [VPAD a ...] WAIT w` as the frames x, a, ... (with VPADs) or x w times.
 * For a frame-synchronous run whose logical frame outlasts the reference's (a pause the
 * reference never made, after the two diverged): its waits go on taking input instead of
 * spinning on the last pads for ever. 0 past the script's end. */
uint16_t input_script_peek(uint32_t n, int player)
{
    if (player < 0 || player >= 2) return 0;
    uint16_t held = s_held_mask[player];
    uint64_t rest = s_wait_until_frame > s_tick_frame + 1 ? s_wait_until_frame - s_tick_frame - 1 : 0;
    if (n < rest) return held;
    n -= (uint32_t)rest;
    int first_pending = 0;
    for (int pc = s_pc; pc < s_op_count; pc++) {
        const ScriptOp *o = &s_ops[pc];
        switch (o->op) {
        case OP_PADS:
            if ((uint32_t)player < o->arg32) held = o->pads[player];
            first_pending = 1;
            break;
        case OP_VPAD:
            if (first_pending) {
                if (n == 0) return held;
                n--;
                first_pending = 0;
            }
            if (n == 0) return o->pads[player];
            n--;
            break;
        case OP_WAIT: {
            uint32_t w = o->arg32 ? o->arg32 : 1;
            uint32_t k = first_pending ? w : w - 1;
            if (n < k) return held;
            n -= k;
            first_pending = 1;
            break;
        }
        default:
            break;
        }
    }
    return 0;
}

uint16_t input_script_player_mask(int player) {
    return s_active && player >= 0 && player < SCRIPT_PLAYERS ? s_held_mask[player] : 0;
}
bool input_script_player_used(int player) {
    return s_active && player >= 0 && player < SCRIPT_PLAYERS && s_players_used[player];
}
uint16_t input_script_held_mask(void) { return input_script_player_mask(0); }
bool    input_script_should_exit(void){ return s_exit_pending != 0; }
int     input_script_exit_code  (void){ return s_exit_code;    }
bool    input_script_active     (void){ return s_active != 0;  }

static bool take_pending_path(char *pending, char *out, size_t out_cap)
{
    if (!pending[0])
        return false;
    if (out && out_cap)
        snprintf(out, out_cap, "%s", pending);
    pending[0] = '\0';
    return true;
}

bool input_script_take_save_state(char *out, size_t out_cap)
{
    return take_pending_path(s_pending_save_state, out, out_cap);
}

bool input_script_take_screenshot(char *out, size_t out_cap)
{
    return take_pending_path(s_pending_screenshot, out, out_cap);
}

bool input_script_take_load_state(char *out, size_t out_cap)
{
    return take_pending_path(s_pending_load_state, out, out_cap);
}

bool input_script_take_ram_dump(char *out, size_t out_cap)
{
    return take_pending_path(s_pending_ram_dump, out, out_cap);
}

bool input_script_take_vram_dump(char *out, size_t out_cap)
{
    return take_pending_path(s_pending_vram_dump, out, out_cap);
}

int input_script_mutates_state(void)
{
    for (int i = 0; i < s_op_count; i++)
        switch (s_ops[i].op) {
        case OP_WRITE_RAM8: case OP_WRITE_RAM16: case OP_WRITE_RAM32:
        case OP_SAVE_STATE: case OP_LOAD_STATE:
            return 1;
        default: break;
        }
    return 0;
}
