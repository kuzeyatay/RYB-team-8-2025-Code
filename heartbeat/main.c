// heartbeat.c  — Address 1 (HEARTBEAT)
#include <libpynq.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>  // for debugging if you want
#include <stdlib.h> // for exit()
#include <unistd.h>

#define UART_CH UART0

#define MSTR 0
#define HRTBT 1 // this module
#define CRY 2
#define MTR 3

#define TIMEOUT 20
#define MAX_PAY 5 // need this so the variable is global

// GPIO pin where the photodiode+op-amp output is connected.
// (We mainly use ADC0 for analog reading now, this pin init is harmless.)
#define HB_PIN IO_AR2

// --- Global display so Ctrl+C handler can access it ---
static display_t disp;

// --- Global for raw ADC (optional debug) ---
float lvl = 0;

// --- Monotonic time in milliseconds ---
// now_msec
// Return current time in milliseconds using CLOCK_MONOTONIC.
// Used for measuring inter-beat intervals.
static double now_msec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1.0e6;
}

// --------------------- small helpers ---------------------

static void itoa_u(unsigned v, char *out)
{
    /* converts v into decimal ASCII, stores in out */
    char tmp[16];
    int n = 0;

    if (!v)
    {
        out[0] = '0';
        out[1] = 0;
        return;
    }

    while (v && n < 16)
    {
        tmp[n++] = (char)('0' + (v % 10));
        v /= 10;
    }

    for (int i = 0; i < n; i++)
    {
        out[i] = tmp[n - 1 - i];
    }
    out[n] = 0;
}

static inline int clampi(int v, int lo, int hi)
{
    /* limits v between lo and hi */
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static void clear_line(display_t *d, int y, int h, uint16_t bg)
{
    /* clears a text line region around baseline y, height h, with color bg */
    int x1 = 0, y1 = y - h + 2, x2 = DISPLAY_WIDTH - 1, y2 = y + 2;

    // clamp to screen
    x1 = clampi(x1, 0, DISPLAY_WIDTH - 1);
    x2 = clampi(x2, 0, DISPLAY_WIDTH - 1);
    y1 = clampi(y1, 0, DISPLAY_HEIGHT - 1);
    y2 = clampi(y2, 0, DISPLAY_HEIGHT - 1);

    if (x2 < x1 || y2 < y1)
        return;

    displayDrawFillRect(d, x1, y1, x2, y2, bg);
}

static void draw_line(display_t *d, FontxFile *fx, int x, int y, const char *s, uint16_t col)
{
    /* draws string s at (x,y) in color col */
    displayDrawString(d, fx, x, y, (uint8_t *)s, col);
}

// --------------------- UART helpers ---------------------

static int timeouted_byte(int ms)
{
    /* waits up to ms for one UART byte, returns byte or -1 on timeout */
    int waited = 0;
    while (waited < ms)
    {
        if (uart_has_data(UART_CH))
        {
            return (int)uart_recv(UART_CH);
        }
        sleep_msec(1);
        waited += 1;
    }
    return -1;
}

// convenience wrapper: TIMEOUT ms per byte
static int receive_byte(void)
{
    return timeouted_byte(TIMEOUT);
}

// [DST][SRC][LEN][PAYLOAD]
void send_message(uint8_t dst, uint8_t src, const uint8_t payload[], uint8_t len)
{
    /* sends one ring message over UART */
    uart_send(UART_CH, dst);
    uart_send(UART_CH, src);
    uart_send(UART_CH, len);
    for (int i = 0; i < len; i++)
    {
        uart_send(UART_CH, payload[i]);
    }
}

/* helper macro: C has no overloading */
#define send_message(dst, src, payload) \
    send_message(dst, src, payload, (uint8_t)sizeof(payload))

static uint8_t g_src = 0;
static uint8_t g_len = 0;
static uint8_t g_payload[MAX_PAY];

// receive_message
// UPDATED: Now non-blocking. Returns -1 immediately if no data.
static int receive_message(void)
{
    // 1. Check if ANY data is available. If not, return immediately.
    //    This prevents the 20ms blocking delay every loop.
    if (!uart_has_data(UART_CH))
    {
        return -1;
    }

    // 2. Data is confirmed, so we can now safely use the timeout function
    //    to read the full frame without stalling the main loop unnecessarily.
    int b = receive_byte(); // Read DST
    if (b < 0)
        return -1;
    uint8_t dst = (uint8_t)b;

    b = receive_byte(); // Read SRC
    if (b < 0)
        return -1;
    uint8_t src = (uint8_t)b;

    b = receive_byte(); // Read LEN
    if (b < 0)
        return -1;
    uint8_t len = (uint8_t)b;

    // --- Forwarding Logic ---
    if (dst != HRTBT)
    {
        uart_send(UART_CH, dst);
        uart_send(UART_CH, src);
        uart_send(UART_CH, len);
        for (int i = 0; i < len; i++)
        {
            int pb = receive_byte();
            if (pb < 0)
                return -2;
            uart_send(UART_CH, (uint8_t)pb);
        }
        return 0;
    }

    // --- Receive Logic (For Me) ---
    if (len > MAX_PAY)
        len = MAX_PAY; // Safety clamp

    for (int i = 0; i < len; i++)
    {
        b = receive_byte();
        if (b < 0)
            return -3;
        g_payload[i] = (uint8_t)b;
    }

    g_src = src;
    g_len = len;
    return g_len;
}

// ------------------ Photodiode-based heartbeat measurement ------------------

// global “real sensor” BPM estimate (0 means “no reliable value yet”)
static int g_bpm_est = 0;

/*
 *  - We read the analog signal from ADC0 (photodiode circuit).
 *  - We track peak (high) and trough (low) values in the waveform.
 *  - We maintain a threshold between them to detect beats.
 *  - On each detected beat, we measure IBI (inter-beat interval).
 *  - BPM is computed from the average of the last 10 IBI values.
 *
 * All timing is based on now_msec() (CLOCK_MONOTONIC).
 */
static void heartbeat_update(double t_ms)
{
    // --- Static state (kept between calls) ---
    static int BPM = 0;        // last computed BPM
    static int IBI = 600;      // inter-beat interval (ms), initial guess
    static int rate[10] = {0}; // rolling history of last 10 IBI values
    static int rate_count = 0; // how many entries are valid (<=10)

    static int Peak = 512;      // running peak of the waveform
    static int Trough = 512;    // running trough of the waveform
    static int Threshold = 550; // detection threshold between Peak & Trough
    static int Amp = 100;       // amplitude estimate = Peak - Trough

    static bool Pulse = false;      // true while we are "in" a beat
    static bool firstBeat = true;   // special handling for very first beat
    static bool secondBeat = false; // special case for second beat

    static double lastBeatTime_ms = 0.0; // time (ms) of last detected beat

    // --- Read analog from photodiode on ADC0 ---
    float v = adc_read_channel(ADC0); // 0.0 .. ~3.3 V (depending on board)
    lvl = v;                          // store globally if you want to log it

    // Scale to something like 0..1023 for threshold math
    // (3.3 * 310 ≈ 1023)
    int Signal = (int)(v * 310.0f);

    // Time since last beat in ms
    double sampleCounter = t_ms;
    int N = (int)(sampleCounter - lastBeatTime_ms);

    // ---------------- Track trough (minimum) and peak (maximum) ----------------
    // We only look for a trough after some part of the IBI has passed to avoid noise.
    if (Signal < Threshold && N > (IBI / 5) * 3)
    {
        if (Signal < Trough)
        {
            Trough = Signal;
        }
    }

    // Track peak when signal is above threshold
    if (Signal > Threshold && Signal > Peak)
    {
        Peak = Signal;
    }

    // ---------------- Look for a beat (rising over threshold) ----------------
    // Basic conditions:
    //   - we were not inside a Pulse before
    //   - Signal crosses above Threshold
    //   - enough time passed since the last beat (refractory period, ~250 ms)
    if (!Pulse && Signal > Threshold && N > 250)
    {
        // We detected a potential beat.
        Pulse = true;
        IBI = (int)(sampleCounter - lastBeatTime_ms);
        lastBeatTime_ms = sampleCounter;

        // Ignore the first beat, we do not have a stable history yet.
        if (firstBeat)
        {
            firstBeat = false;
            secondBeat = true;
            return;
        }

        // Second beat: initialize rate[] with this IBI
        if (secondBeat)
        {
            secondBeat = false;
            for (int i = 0; i < 10; i++)
            {
                rate[i] = IBI;
            }
            rate_count = 10;
        }
        else
        {
            // Shift rate[] left, append new IBI at the end
            for (int i = 0; i < 9; i++)
            {
                rate[i] = rate[i + 1];
            }
            rate[9] = IBI;
            if (rate_count < 10)
            {
                rate_count++;
            }
        }

        // Compute average IBI from history
        long total = 0;
        for (int i = 0; i < rate_count; i++)
        {
            total += rate[i];
        }
        int avgIBI = (rate_count > 0) ? (int)(total / rate_count) : IBI;

        // Convert IBI (ms) to BPM
        if (avgIBI > 0)
        {
            BPM = (int)(60000 / avgIBI);
        }
        else
        {
            BPM = 0;
        }

        g_bpm_est = BPM; // publish BPM
    }

    // ---------------- End of beat: going back below threshold ----------------
    if (Signal < Threshold && Pulse)
    {
        Pulse = false;
        Amp = Peak - Trough;
        if (Amp < 20)
        {
            // very small amplitude, force a minimum to keep Threshold sane
            Amp = 20;
        }
        // Set new Threshold halfway between peak and trough
        Threshold = Trough + Amp / 2;
        Peak = Threshold;
        Trough = Threshold;
    }

    // ---------------- If no beat for a long time, reset detector -------------
    // After ~2.5 seconds without a beat, assume signal lost or sensor off.
    if (N > 2500)
    {
        Threshold = 550;
        Peak = 512;
        Trough = 512;
        lastBeatTime_ms = sampleCounter;
        firstBeat = true;
        secondBeat = false;
        Pulse = false;
        BPM = 0;
        g_bpm_est = 0;
        rate_count = 0;
    }
}

// -------- safe exit on Ctrl+C ----------
static void handle_sigint(int sig __attribute__((unused)))
{
  displayFillScreen(&disp, RGB_BLACK);
  printf("\nExited\n");
  display_destroy(&disp);
  pynq_destroy();
  exit(0);
}

static void restart_program(void)
{
    // Prevent Ctrl+C during restart teardown/exec
    signal(SIGINT, SIG_IGN);

    // Optional: clear screen as a UX cue (safe to skip if you suspect display code)
    displayFillScreen(&disp, RGB_BLACK);

    // Execute self (argv[0] must be non-NULL)
    execl("/proc/self/exe", "/proc/self/exe", (char*)NULL);

    // If execl returns, it failed
    perror("execl failed");
    _exit(127);
}


int main(void)
{
    // Install Ctrl+C handler
    signal(SIGINT, handle_sigint);

    // HW init
    pynq_init();
    uart_init(UART_CH);
    uart_reset_fifos(UART_CH);

    // UART pins
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);

    // GPIO for heartbeat sensor (not strictly needed if you use only ADC0)
    gpio_init();
    gpio_set_direction(HB_PIN, GPIO_DIR_INPUT);
    switchbox_set_pin(HB_PIN, SWB_GPIO);

    // ADC for photodiode
    adc_init();

    // Buttons (for fake BPM override)
    buttons_init();
    static int restart_hold_ms = 0;

    // Display init  (use global disp here)
    display_init(&disp);
    display_set_flip(&disp, true, true);
    displayFillScreen(&disp, RGB_BLACK);
    FontxFile fx[2];
    uint8_t glyph[FontxGlyphBufSize], fw, fh;
    InitFontx(fx, "/boot/ILGH16XB.FNT", "");
    GetFontx(fx, 0, glyph, &fw, &fh);
    displaySetFontDirection(&disp, TEXT_DIRECTION0);

    int x = 6, y = fh * 1;
    draw_line(&disp, fx, x, y, "HEARTBEAT MODULE", RGB_GREEN);
    y += fh;
    draw_line(&disp, fx, x, y, "Waiting for 'H'/'A'/'R'...", RGB_WHITE);
    y += fh;
    int y_val = y; // line where BPM / RND text is drawn
    y += fh;

    // ---- state ----
    uint8_t bpm_button = 0; // BPM chosen with buttons (fake)
    uint32_t rand_tick = 0; // for 'R' command

    // edge-trigger memory for buttons
    int prev_b0 = 0, prev_b1 = 0;

    while (1)
    {
        // current time in ms from monotonic clock
        double t_ms = now_msec();

        // --- button-based fake BPM (edge detected) ---
        int b0 = get_button_state(0);
        int b1 = get_button_state(1);
        int b3 = get_button_state(3);
        if (b0 && !prev_b0)
        {
            bpm_button = 80; // button 0 -> 80 BPM
        }
        if (b1 && !prev_b1)
        {
            bpm_button = 200; // button 1 -> 200 BPM
        }
        prev_b0 = b0;
        prev_b1 = b1;

        // Button 3 = RESTART (long press ~1s)
        // Note: this shares button 3 with FREQ+. Short press increments freq, long press restarts.
        if (b3)
        {
            restart_hold_ms += 20;
            if (restart_hold_ms >= 1000)
            {
                restart_program();
            }
        }
        else
        {
            restart_hold_ms = 0;
        }

        // --- update real heartbeat from photodiode (PulseSensor-style logic) ---
        heartbeat_update(t_ms);

        // choose which BPM to use:
        // if sensor BPM is in reasonable range, prefer it; else use button BPM
        int bpm_effective;
        if (g_bpm_est >= 40 && g_bpm_est <= 240)
        {
            bpm_effective = g_bpm_est;
        }
        else
        {
            bpm_effective = bpm_button;
        }

        // clamp for display, but allow 0 to mean "no BPM yet"
        int bpm_display = clampi(bpm_effective, 0, 250);

        // --- Display BPM on PYNQ ---
        clear_line(&disp, y_val, fh, RGB_BLACK);
        char buf[32], num[16];
        strcpy(buf, "BPM=");

        if (bpm_display <= 0)
        {
            strcat(buf, "---");
        }
        else
        {
            itoa_u((unsigned)bpm_display, num);
            strcat(buf, num);
        }

        draw_line(&disp, fx, x, y_val, buf, RGB_WHITE);

        // --- UART receive & handle ---
        int r = receive_message();
        if (r > 0)
        {
            if (g_len >= 1)
            {
                uint8_t cmd = g_payload[0];

                if (cmd == 'A')
                {
                    // Echo 'A' for boot ping
                    uint8_t rsp[] = {'A'};
                    send_message(MSTR, HRTBT, rsp);
                }
                else if (cmd == 'R')
                {
                    // pseudo-random byte for demo
                    uint8_t v = (uint8_t)((rand_tick * 73u + 41u) & 0xFFu);
                    rand_tick++;
                    uint8_t rsp[] = {'R', v};
                    send_message(MSTR, HRTBT, rsp);

                    // show RND on screen (temporary, until next BPM update overwrites it)
                    clear_line(&disp, y_val, fh, RGB_BLACK);
                    char b2[32], n2[16];
                    strcpy(b2, "RND=");
                    itoa_u(v, n2);
                    strcat(b2, n2);
                    draw_line(&disp, fx, x, y_val, b2, RGB_YELLOW);
                }
                else if (cmd == 'H')
                {
                    // reply with current BPM (sensor if valid, else button)
                    uint8_t rsp[] = {'H', (uint8_t)clampi(bpm_effective, 0, 255)};
                    send_message(MSTR, HRTBT, rsp);
                }
                // else: ignore unknown
            }
        }

        // loop rate ~50 Hz
        sleep_msec(20);
    }

    // not reached, but kept for completeness
    display_destroy(&disp);
    gpio_destroy();
    pynq_destroy();
    adc_destroy();
    uart_reset_fifos(UART_CH);
    return 0;
}
