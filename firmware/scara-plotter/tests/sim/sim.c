/*
 * Banco di prova senza hardware: esegue il firmware compilato (.elf) su un
 * Mega2560 simulato (simavr), con RAMPS, due finecorsa per giunto e il
 * braccio SCARA fisico. Invia un file G-code riga per riga come farebbe un
 * host (aspetta "ok") e confronta la posizione reale della penna con quella
 * comandata.
 *
 *   sim fw.elf prova.gcode  tower_x tower_y L1 L2  rel(0|1)  steps_deg_A steps_deg_B \
 *       startA startB  endstopA_lo endstopA_hi endstopB_lo endstopB_hi  nc(0|1) invA(0|1) invB(0|1)
 *
 * Pin RAMPS 1.4 sul Mega: X step PF0 dir PF1 | Y step PF6 dir PF7
 *                         X min PE5 max PE4  | Y min PJ1 max PJ0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <simavr/sim_avr.h>
#include <simavr/sim_elf.h>
#include <simavr/avr_ioport.h>
#include <simavr/avr_uart.h>

#define D2R (M_PI / 180.0)
static avr_t *avr;
static double tx, ty, L1, L2, spd[2], start[2], elo[2], ehi[2], steps[2];
static int rel, nc, inv[2];
static int dirpin[2], lastStep[2], levels[4] = {-1, -1, -1, -1};
static double HARD = 12.0;          /* gradi oltre il finecorsa prima di rompere il meccanismo (env SIM_HARD) */
static int fatal = 0;
static avr_irq_t *pin_irq[4];             /* Xmin Xmax Ymin Ymax */

static double angle(int i) { return start[i] + steps[i] / spd[i]; }

static void tip(double *x, double *y) {
    double a = angle(0) * D2R, b = angle(1) * D2R;
    if (rel) b += a;
    *x = tx + L1 * cos(a) + L2 * cos(b);
    *y = ty + L1 * sin(a) + L2 * sin(b);
}

/* I finecorsa sono interruttori esterni: scrivo direttamente il valore nei registri PINE/PINJ
   (indirizzi dati 0x2C e 0x103 sul Mega2560), così vince sul pull-up simulato da simavr. */
static const struct { char port; int bit; int reg; } ESP[4] = { {'E', 5, 0x2C}, {'E', 4, 0x2C}, {'J', 1, 0x103}, {'J', 0, 0x103} };
static void reassert_pins(void) {
    for (int k = 0; k < 4; k++) {
        if (levels[k] < 0) continue;
        uint8_t m = (uint8_t)(1 << ESP[k].bit);
        if (levels[k]) avr->data[ESP[k].reg] |= m; else avr->data[ESP[k].reg] &= (uint8_t)~m;
    }
}
static void drive_pin(int k, int lvl) { levels[k] = lvl; reassert_pins(); }

static void update_endstops(void) {
    double a = angle(0), b = angle(1);
    int trig[4] = { a <= elo[0], a >= ehi[0], b <= elo[1], b >= ehi[1] };
    for (int k = 0; k < 4; k++) {
        int lvl = nc ? trig[k] : !trig[k];
        if (lvl != levels[k]) { levels[k] = lvl; drive_pin(k, lvl); if (getenv("SIM_TRACE")) printf("PIN %d -> %d  (A=%.2f B=%.2f)\n", k, lvl, a, b); }
    }
    if (a < elo[0] - HARD || a > ehi[0] + HARD || b < elo[1] - HARD || b > ehi[1] + HARD) {
        if (!fatal) printf("FATAL: giunto oltre il fine corsa meccanico (A=%.2f B=%.2f)\n", a, b);
        fatal = 1;
    }
}

static void dir_cb(avr_irq_t *irq, uint32_t v, void *p) { dirpin[(intptr_t)p] = v; }
static void step_cb(avr_irq_t *irq, uint32_t v, void *p) {
    int ax = (intptr_t)p;
    if (v && !lastStep[ax]) { steps[ax] += (dirpin[ax] != inv[ax]) ? 1 : -1; update_endstops(); }
    lastStep[ax] = v;
}

/* ---- seriale ---- */
static char line[512]; static int ll;
static int got_ok, got_err; static double fw_x, fw_y; static int got_pos;
static void uart_cb(avr_irq_t *irq, uint32_t v, void *p) {
    char c = (char)v;
    if (c == '\r') return;
    if (c != '\n') { if (ll < (int)sizeof(line) - 1) line[ll++] = c; return; }
    line[ll] = 0; ll = 0;
    if (!line[0]) return;
    printf("<  %s\n", line);
    if (!strncmp(line, "ok", 2)) got_ok = 1;
    if (!strncmp(line, "Error:", 6)) got_err = 1;
    if (!strncmp(line, "X:", 2)) {
        double x, y;
        if (sscanf(line, "X:%lf Y:%lf", &x, &y) == 2) { fw_x = x; fw_y = y; got_pos = 1; }
    }
}

int main(int argc, char **argv) {
    if (argc < 16) { fprintf(stderr, "uso: vedi commento in testa\n"); return 2; }
    const char *elf = argv[1], *gc = argv[2];
    tx = atof(argv[3]); ty = atof(argv[4]); L1 = atof(argv[5]); L2 = atof(argv[6]);
    rel = atoi(argv[7]); spd[0] = atof(argv[8]); spd[1] = atof(argv[9]);
    start[0] = atof(argv[10]); start[1] = atof(argv[11]);
    elo[0] = atof(argv[12]); ehi[0] = atof(argv[13]); elo[1] = atof(argv[14]); ehi[1] = atof(argv[15]);
    nc = argc > 16 ? atoi(argv[16]) : 1;
    inv[0] = argc > 17 ? atoi(argv[17]) : 0;
    inv[1] = argc > 18 ? atoi(argv[18]) : 0;
    if (getenv("SIM_HARD")) HARD = atof(getenv("SIM_HARD"));

    FILE *f = fopen(gc, "r");
    if (!f) { perror(gc); return 2; }
    static char lines[4000][160]; int nl = 0;
    while (nl < 4000 && fgets(lines[nl], sizeof lines[0], f)) {
        char *s = lines[nl]; s[strcspn(s, "\r\n")] = 0;
        if (!s[0] || s[0] == ';') continue;
        nl++;
    }
    fclose(f);

    elf_firmware_t fw; memset(&fw, 0, sizeof fw);
    if (elf_read_firmware(elf, &fw)) { fprintf(stderr, "ELF non leggibile\n"); return 2; }
    avr = avr_make_mcu_by_name("atmega2560");
    if (!avr) { fprintf(stderr, "mcu non trovato\n"); return 2; }
    avr_init(avr); avr_load_firmware(avr, &fw); avr->frequency = 16000000; avr->log = LOG_ERROR;

    uint32_t flags = 0;
    avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &flags);
    flags &= ~(AVR_UART_FLAG_STDIO | AVR_UART_FLAG_POLL_SLEEP);
    avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &flags);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), uart_cb, NULL);
    avr_irq_t *rx = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT);

    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('F'), 0), step_cb, (void *)0);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('F'), 1), dir_cb, (void *)0);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('F'), 6), step_cb, (void *)1);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('F'), 7), dir_cb, (void *)1);
    pin_irq[0] = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('E'), 5);
    pin_irq[1] = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('E'), 4);
    pin_irq[2] = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('J'), 1);
    pin_irq[3] = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('J'), 0);
    update_endstops();

    const uint64_t BYTE = 1400, BOOT = 24000000ULL;
    const uint64_t TIMEOUT = 16000000ULL * (getenv("SIM_TIMEOUT_S") ? atoi(getenv("SIM_TIMEOUT_S")) : 90);
    uint64_t udre_low_since = 0;
    const int trace = getenv("SIM_TRACE") != NULL; uint64_t next_trace = 0;
    int idx = 0, pos = 0, sending = 0, failures = 0, checks = 0;
    uint64_t next_byte = BOOT, sent_at = 0;
    double cmd_x = 0, cmd_y = 0; int have_cmd = 0, want_check = 0, want_pos = 0;
    char cur[160] = "";

    while (!fatal) {
        int st = avr_run(avr);
        reassert_pins();
        /* simavr conserva il valore scritto in UCSR0A: Marlin lo azzera in begin() e il flag UDRE
           (bit 5) resta spento finché non parte una trasmissione. Su un AVR vero è sola lettura. */
        if (avr->data[0xC0] & 0x20) udre_low_since = 0;
        else if (!udre_low_since) udre_low_since = avr->cycle;
        else if (avr->cycle - udre_low_since > 20000) { avr->data[0xC0] |= 0x20; udre_low_since = 0; }
        if (trace && avr->cycle >= next_trace) { printf("TRACE cyc=%llu pc=0x%x I=%d UCSR0A=%02x B=%02x C=%02x UBRR=%02x%02x\n", (unsigned long long)avr->cycle, avr->pc, (avr->sreg[S_I] != 0), avr->data[0xC0], avr->data[0xC1], avr->data[0xC2], avr->data[0xC5], avr->data[0xC4]); next_trace = avr->cycle + 8000000ULL; }
        if (st == cpu_Done || st == cpu_Crashed) { printf("FATAL: cpu fermata (%d)\n", st); fatal = 1; break; }
        if (!sending) {
            if (avr->cycle < next_byte) continue;
            if (idx >= nl) break;
            strcpy(cur, lines[idx++]); pos = 0; sending = 1; got_ok = got_err = 0; sent_at = avr->cycle;
            printf(">  %s\n", cur);
            { char *p = cur; double x, y; int hx = 0, hy = 0;
              if (!strncmp(cur, "G0 ", 3) || !strncmp(cur, "G1 ", 3)) {
                  for (p = cur; *p; p++) {
                      if ((*p == 'X' || *p == 'x') && sscanf(p + 1, "%lf", &x) == 1) { cmd_x = x; hx = 1; }
                      if ((*p == 'Y' || *p == 'y') && sscanf(p + 1, "%lf", &y) == 1) { cmd_y = y; hy = 1; }
                  }
                  if (hx || hy) have_cmd = 1;
              }
              want_check = !strncmp(cur, "M400", 4);
              want_pos = !strncmp(cur, "M114", 4); got_pos = 0; }
        }
        if (sending && pos <= (int)strlen(cur) && avr->cycle >= next_byte) {
            avr_raise_irq(rx, pos < (int)strlen(cur) ? (uint8_t)cur[pos] : '\n');
            pos++; next_byte = avr->cycle + BYTE;
        }
        if (sending && got_err) { printf("FALLITO: errore dal firmware\n"); failures++; sending = 0; next_byte = avr->cycle + BYTE; }
        else if (sending && got_ok && pos > (int)strlen(cur)) {
            sending = 0; next_byte = avr->cycle + BYTE * 4;
            double px, py; tip(&px, &py);
            if (getenv("SIM_PEEK")) { unsigned addr = strtoul(getenv("SIM_PEEK"), NULL, 16) & 0xFFFF; printf("PEEK  [0x%x] = 0x%02x%02x\n", addr, avr->data[addr + 1], avr->data[addr]); }
            if (want_check && have_cmd) {
                double e = hypot(px - cmd_x, py - cmd_y); checks++;
                printf("CHECK  penna X=%.3f Y=%.3f  comandato X=%.3f Y=%.3f  errore %.3f mm %s\n", px, py, cmd_x, cmd_y, e, e > 0.3 ? "<-- FUORI" : "");
                if (e > 0.3) failures++;
            }
            if (want_pos && got_pos) {
                double e = hypot(px - fw_x, py - fw_y); checks++;
                printf("FWPOS  firmware X=%.3f Y=%.3f  penna reale X=%.3f Y=%.3f  differenza %.3f mm %s\n", fw_x, fw_y, px, py, e, e > 0.3 ? "<-- FUORI" : "");
                if (e > 0.3) failures++;
            }
        }
        else if (sending && avr->cycle - sent_at > TIMEOUT) { printf("FALLITO: timeout su '%s'\n", cur); failures++; fatal = 1; }
    }
    printf("RISULTATO %s  verifiche=%d fallimenti=%d  A=%.2f B=%.2f passi=%.0f/%.0f\n", (fatal || failures) ? "FAIL" : "OK", checks, failures, angle(0), angle(1), steps[0], steps[1]);
    return (fatal || failures) ? 1 : 0;
}
