/*
 * ram_views.c - 68000 work RAM, mapped twice.
 *
 * The 68000 decodes 24 address bits, so a program can hold the same work-RAM byte as
 * $00FFxxxx or as $FFFFxxxx (A5-relative address arithmetic gives the latter), and it may
 * store either 32-bit value in memory. Hand-written C that runs alongside generated code
 * uses plain host pointers into RAM; to turn such a pointer back into the exact 32-bit
 * value the original had, the RAM is mapped at two host addresses backed by the same
 * pages: g_ram stands for $00FFxxxx, g_ram_ffff for $FFFFxxxx. Everything else in the
 * runner uses g_ram only.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#define _GNU_SOURCE
#include <sys/mman.h>
#include <unistd.h>
#endif

uint8_t *g_ram;
uint8_t *g_ram_ffff;

void genesis_ram_init(void)
{
    if (g_ram) return;
#ifdef _WIN32
    HANDLE h = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, 0x10000, NULL);
    if (h) {
        g_ram      = (uint8_t *)MapViewOfFile(h, FILE_MAP_ALL_ACCESS, 0, 0, 0x10000);
        g_ram_ffff = (uint8_t *)MapViewOfFile(h, FILE_MAP_ALL_ACCESS, 0, 0, 0x10000);
    }
#else
    int fd = memfd_create("genesis-ram", 0);
    if (fd >= 0 && ftruncate(fd, 0x10000) == 0) {
        void *a = mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        void *b = mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (a != MAP_FAILED && b != MAP_FAILED) { g_ram = (uint8_t *)a; g_ram_ffff = (uint8_t *)b; }
    }
#endif
    if (!g_ram || !g_ram_ffff) {
        fprintf(stderr, "genesis_ram_init: cannot map work RAM twice\n");
        exit(2);
    }
}
