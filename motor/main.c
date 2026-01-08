// motor.c  — Address 3 (MOTOR)
// Ring frame: [DST][SRC][LEN][PAYLOAD...]
// Motor only receives values meant for it and never replies.
// It forwards frames that are NOT for it.

#include <libpynq.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <buttons.h> // <-- adjust include if needed

#define UART_CH UART0
#define MSTR 0
#define HRTBT 1
#define CRY 2
#define MTR 3 // this module
#define TIMEOUT 20
#define MAX_PAY 8

// Logical channels for safety checks (no HW meaning here)
#define AMP_CH 0
#define FREQ_CH 1

// *** SET THESE TO THE REAL PINS DRIVING THE CRADLE HARDWARE ***
#define AMP_PWM_PIN IO_AR2  // amplitude PWM output pin
#define FREQ_PWM_PIN IO_AR3 // frequency PWM output pin

// PWM block identifiers
#define AMP_PWM PWM0  // amplitude PWM block
#define FREQ_PWM PWM1 // frequency PWM block

// Switchbox configuration values (this is what switchbox_set_pin expects)
#define AMP_PWM_CFG SWB_PWM0
#define FREQ_PWM_CFG SWB_PWM1

// --- globals for font size (used to safely clip text on screen) ---
static uint8_t g_fw = 8;  // font width (set in main after GetFontx)
static uint8_t g_fh = 16; // font height (set in main after GetFontx)

// --- Global display so Ctrl+C handler can access it ---
static display_t disp;

// --- tiny itoa (no sprintf) ---
static void itoa_u(unsigned v, char *out)
{
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
    out[i] = tmp[n - 1 - i];
  out[n] = 0;
}

// --- display helpers ---
static inline int clampi(int v, int lo, int hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void clear_line(display_t *d, int y, int h, uint16_t bg)
{
  int x1 = 0, y1 = y - h + 2, x2 = DISPLAY_WIDTH - 1, y2 = y + 2;
  x1 = clampi(x1, 0, DISPLAY_WIDTH - 1);
  x2 = clampi(x2, 0, DISPLAY_WIDTH - 1);
  y1 = clampi(y1, 0, DISPLAY_HEIGHT - 1);
  y2 = clampi(y2, 0, DISPLAY_HEIGHT - 1);
  if (x2 < x1 || y2 < y1) return;
  displayDrawFillRect(d, x1, y1, x2, y2, bg);
}

// Safe draw_line: truncates string so it never goes off-screen.
static void draw_line(display_t *d, FontxFile *fx, int x, int y, const char *s, uint16_t col)
{
  char buf[64];
  int fw = (g_fw == 0) ? 8 : g_fw;
  int max_chars = (DISPLAY_WIDTH - x) / fw;
  if (max_chars <= 0) return;
  if (max_chars > (int)(sizeof(buf) - 1)) max_chars = sizeof(buf) - 1;

  int n = 0;
  while (n < max_chars && s[n] != '\0')
  {
    buf[n] = s[n];
    n++;
  }
  buf[n] = '\0';
  displayDrawString(d, fx, x, y, (uint8_t *)buf, col);
}

// --- UART helpers ---
static int timeouted_byte(int ms)
{
  int waited = 0;
  while (waited < ms)
  {
    if (uart_has_data(UART_CH))
      return (int)uart_recv(UART_CH);
    sleep_msec(1);
    waited += 1;
  }
  return -1;
}

static int receive_byte(void) { return timeouted_byte(TIMEOUT); }

// --- parsed frame globals (filled by receive_message) ---
static uint8_t g_src = 0;
static uint8_t g_len = 0;
static uint8_t g_payload[MAX_PAY];

// Receive one frame; forward if not for MTR.
// Return payload length (>0) if for MTR, 0 if forwarded, <0 on timeout/error.
// UPDATED: Now non-blocking. Returns -1 immediately if no data.
static int receive_message(void)
{
  if (!uart_has_data(UART_CH))
    return -1;

  int b = receive_byte(); // DST
  if (b < 0) return -1;
  uint8_t dst = (uint8_t)b;

  b = receive_byte(); // SRC
  if (b < 0) return -1;
  uint8_t src = (uint8_t)b;

  b = receive_byte(); // LEN
  if (b < 0) return -1;
  uint8_t len = (uint8_t)b;

  // Forward if not for me
  if (dst != MTR)
  {
    uart_send(UART_CH, dst);
    uart_send(UART_CH, src);
    uart_send(UART_CH, len);
    for (int i = 0; i < len; i++)
    {
      int pb = receive_byte();
      if (pb < 0) return -2;
      uart_send(UART_CH, (uint8_t)pb);
    }
    return 0;
  }

  // Receive payload (for me)
  if (len > MAX_PAY) len = MAX_PAY;
  for (int i = 0; i < len; i++)
  {
    b = receive_byte();
    if (b < 0) return -3;
    g_payload[i] = (uint8_t)b;
  }

  g_src = src;
  g_len = len;
  return g_len;
}

// [DST][SRC][LEN][PAYLOAD]
static void send_message_frame(uint8_t dst, uint8_t src, const uint8_t payload[], uint8_t len)
{
  uart_send(UART_CH, dst);
  uart_send(UART_CH, src);
  uart_send(UART_CH, len);
  for (int i = 0; i < len; i++)
    uart_send(UART_CH, payload[i]);
}

// convenience macro for fixed-size payload arrays
#define SEND_MESSAGE(dst, src, payload_arr) \
  send_message_frame((dst), (src), (payload_arr), (uint8_t)sizeof(payload_arr))

// ---------------- TABLE-CORRECT MAPPINGS ----------------
//
// Region 1..5:
// Duty Cycle Input Range: 0–10, 10–30, 30–50, 50–70, 70–90  (range label only)
// Frequency (Hz):         0.20, 0.35, 0.50, 0.65, 0.70
// Amplitude (%):          20,   40,   60,   80,   100
//
static int amp_region_to_percent(int region_1to5)
{
  switch (region_1to5)
  {
    case 1: return 20;
    case 2: return 40;
    case 3: return 60;
    case 4: return 80;
    case 5: return 100;
    default: return 20;
  }
}

static int freq_region_to_hz_x100(int region_1to5)
{
  // Hz * 100 to avoid floats: 0.20->20, 0.35->35, ...
  switch (region_1to5)
  {
    case 1: return 20;
    case 2: return 35;
    case 3: return 50;
    case 4: return 65;
    case 5: return 70;
    default: return 20;
  }
}

// Safety check: percent in [0..100]
static void set_pwm_percent(int channel, int percent)
{
  (void)channel;
  if (percent < 0 || percent > 100)
  {
    printf("[EMERGENCY] percent=%d out of [0..100]\n", percent);
    exit(1);
  }
}

// Build: "A_IDX=5 (100%)" where idx shown is region (1..5)
static void make_amp_line(const char *label, uint8_t region_1to5, char *out, size_t out_sz)
{
  char n1[16], n2[16];

  if (region_1to5 < 1) region_1to5 = 1;
  if (region_1to5 > 5) region_1to5 = 5;

  out[0] = '\0';
  strncat(out, label, out_sz - 1);

  itoa_u(region_1to5, n1);
  strncat(out, n1, out_sz - strlen(out) - 1);

  strncat(out, " (", out_sz - strlen(out) - 1);

  itoa_u((unsigned)amp_region_to_percent(region_1to5), n2);
  strncat(out, n2, out_sz - strlen(out) - 1);

  strncat(out, "%)", out_sz - strlen(out) - 1);
}

// Build: "F_IDX=5 (0.70Hz)" where idx shown is region (1..5)
static void make_freq_line(const char *label, uint8_t region_1to5, char *out, size_t out_sz)
{
  char n1[16], nHz[16];

  if (region_1to5 < 1) region_1to5 = 1;
  if (region_1to5 > 5) region_1to5 = 5;

  int hz_x100 = freq_region_to_hz_x100(region_1to5);

  out[0] = '\0';
  strncat(out, label, out_sz - 1);

  itoa_u(region_1to5, n1);
  strncat(out, n1, out_sz - strlen(out) - 1);

  strncat(out, " (0.", out_sz - strlen(out) - 1);

  // Always 2 digits after decimal for your values (20,35,50,65,70)
  itoa_u((unsigned)hz_x100, nHz);
  strncat(out, nHz, out_sz - strlen(out) - 1);

  strncat(out, "Hz)", out_sz - strlen(out) - 1);
}

// Apply frequency output (square wave) using PWM period.
// Assumes PWM timebase is 10 ns per tick (consistent with your 1kHz example).
static void set_freq_pwm_from_hz_x100(int hz_x100)
{
  if (hz_x100 < 1) hz_x100 = 1; // avoid divide-by-zero

  // f = hz_x100 / 100.0
  // period_seconds = 1/f = 100 / hz_x100
  // ticks = period_seconds / 10ns = (100 / hz_x100) / 1e-8 = 1e10 / hz_x100
  // 1e10 fits in uint64_t; result fits uint32_t for hz_x100 >= 1.
  uint64_t ticks64 = (uint64_t)10000000000ULL / (uint64_t)hz_x100;
  if (ticks64 < 2) ticks64 = 2; // minimum sane
  if (ticks64 > 0xFFFFFFFFULL) ticks64 = 0xFFFFFFFFULL;

  uint32_t period_ticks = (uint32_t)ticks64;

  // Re-init PWM1 with the new period (drop-in approach)
  pwm_init(FREQ_PWM, period_ticks);

  // 50% duty square wave at desired frequency
  pwm_set_duty_cycle(FREQ_PWM, period_ticks / 2);
}

// amp_index, freq_index are 0..4 (matrix indices for A/F).
// We map index -> region (1..5) -> amplitude% and frequency Hz per TABLE.
static void command_motor(int amp_index, int freq_index)
{
  if (amp_index < 0 || amp_index > 4 || freq_index < 0 || freq_index > 4)
  {
    printf("[SYSTEM][ERROR] command_motor out-of-bounds A%d F%d\n",
           amp_index + 1, freq_index + 1);
    return;
  }

  int a_region = amp_index + 1;  // 1..5
  int f_region = freq_index + 1; // 1..5

  int amp_pct = amp_region_to_percent(a_region);   // 20..100
  int hz_x100 = freq_region_to_hz_x100(f_region);  // 20..70  (0.20..0.70 Hz)

  // Safety checks (now aligned to table’s 100% amplitude)
  set_pwm_percent(AMP_CH, amp_pct);
  // Frequency channel not a "%", so no percent safety check here.

  // --- Amplitude PWM (keep your existing 1 kHz carrier) ---
  uint32_t amp_period = 100000; // 1 ms period at 10 ns ticks => 1 kHz
  pwm_init(AMP_PWM, amp_period);
  pwm_set_duty_cycle(AMP_PWM, (uint32_t)(amp_period * (uint32_t)amp_pct / 100U));

  // --- Frequency output PWM (square wave at 0.20..0.70 Hz) ---
  set_freq_pwm_from_hz_x100(hz_x100);
}

static void handle_sigint(int sig __attribute__((unused)))
{
  displayFillScreen(&disp, RGB_BLACK);
  printf("\n Exited\n");
  display_destroy(&disp);
  pynq_destroy();
  exit(0);
}

int main(void)
{
  signal(SIGINT, handle_sigint);

  // IO + UART init
  pynq_init();
  uart_init(UART_CH);
  uart_reset_fifos(UART_CH);

  // UART pins (do NOT reuse these for PWM)
  switchbox_set_pin(IO_AR0, SWB_UART0_RX);
  switchbox_set_pin(IO_AR1, SWB_UART0_TX);

  // PWM outputs – map to cradle driver pins
  switchbox_set_pin(AMP_PWM_PIN, AMP_PWM_CFG);
  switchbox_set_pin(FREQ_PWM_PIN, FREQ_PWM_CFG);

  // Initialize PWMs to something valid
  pwm_init(AMP_PWM, 100000);    // 1 kHz carrier (amplitude)
  pwm_init(FREQ_PWM, 100000);   // placeholder until first command

  // Buttons init
  buttons_init();

  // display init
  display_init(&disp);
  display_set_flip(&disp, true, true);
  displayFillScreen(&disp, RGB_BLACK);
  FontxFile fx[2];
  uint8_t glyph[FontxGlyphBufSize], fw, fh;
  InitFontx(fx, "/boot/ILGH16XB.FNT", "");
  GetFontx(fx, 0, glyph, &fw, &fh);
  g_fw = fw;
  g_fh = fh;
  displaySetFontDirection(&disp, TEXT_DIRECTION0);

  int x = 6, y = fh * 1;
  draw_line(&disp, fx, x, y, "MOTOR MODULE", RGB_GREEN);
  y += fh;

  // --- ASCII table ---
  const char *tbl[] = {
      "+-------------------+",
      "   DC%     F(Hz)  A%",
      "1  0-10    0.2    20",
      "2  10-30   0.35   40",
      "3  30-50   0.5    60",
      "4  50-70   0.65   80",
      "5  70-90   0.7   100",
      "+-------------------+",
  };

  for (unsigned i = 0; i < sizeof(tbl) / sizeof(tbl[0]); i++)
  {
    draw_line(&disp, fx, x, y, tbl[i], RGB_WHITE);
    y += fh;
  }

  draw_line(&disp, fx, x, y, "Waiting for 'M' (A,F)...", RGB_WHITE);
  y += fh;

  int y_amp = y;  y += fh;
  int y_freq = y; y += fh;

  uint8_t amp_idx = 4;   // 0..4
  uint8_t freq_idx = 4;  // 0..4

  // Apply initial motor command to match initial indices
  command_motor((int)amp_idx, (int)freq_idx);

  // Initial display (show TABLE-correct values)
  {
    char buf[64];

    make_amp_line("A_IDX=", (uint8_t)(amp_idx + 1), buf, sizeof(buf));
    draw_line(&disp, fx, x, y_amp, buf, RGB_YELLOW);

    make_freq_line("F_IDX=", (uint8_t)(freq_idx + 1), buf, sizeof(buf));
    draw_line(&disp, fx, x, y_freq, buf, RGB_YELLOW);
  }

  // button previous states for edge detection (0..3)
  int prev_b0 = 0, prev_b1 = 0, prev_b2 = 0, prev_b3 = 0;

  while (1)
  {
    // 1) Handle UART messages from master
    int r = receive_message();
    if (r > 0 && g_len >= 1)
    {
      uint8_t cmd = g_payload[0];
      if (cmd == 'A')
      {
        // boot ping reply (if you truly want "never replies", delete this block)
        uint8_t rsp[] = {'A'};
        SEND_MESSAGE(MSTR, MTR, rsp);
      }
      else if (cmd == 'M' && g_len >= 3)
      {
        amp_idx = g_payload[1];
        freq_idx = g_payload[2];

        if (amp_idx > 4) amp_idx = 4;
        if (freq_idx > 4) freq_idx = 4;

        command_motor((int)amp_idx, (int)freq_idx);

        // refresh display (TABLE-correct)
        clear_line(&disp, y_amp, fh, RGB_BLACK);
        clear_line(&disp, y_freq, fh, RGB_BLACK);

        char buf[64];

        make_amp_line("A_IDX=", (uint8_t)(amp_idx + 1), buf, sizeof(buf));
        draw_line(&disp, fx, x, y_amp, buf, RGB_WHITE);

        make_freq_line("F_IDX=", (uint8_t)(freq_idx + 1), buf, sizeof(buf));
        draw_line(&disp, fx, x, y_freq, buf, RGB_WHITE);
      }
    }

    // 2) Button controls (4 buttons): A-/A+/F-/F+
    int b0 = get_button_state(0); // AMP -1
    int b1 = get_button_state(1); // AMP +1
    int b2 = get_button_state(2); // FREQ -1
    int b3 = get_button_state(3); // FREQ +1

    bool changed = false;

    if (b0 && !prev_b0) { if (amp_idx > 0) amp_idx--; changed = true; }
    if (b1 && !prev_b1) { if (amp_idx < 4) amp_idx++; changed = true; }
    if (b2 && !prev_b2) { if (freq_idx > 0) freq_idx--; changed = true; }
    if (b3 && !prev_b3) { if (freq_idx < 4) freq_idx++; changed = true; }

    if (changed)
    {
      command_motor((int)amp_idx, (int)freq_idx);

      clear_line(&disp, y_amp, fh, RGB_BLACK);
      clear_line(&disp, y_freq, fh, RGB_BLACK);

      char buf[64];

      make_amp_line("A_IDX=", (uint8_t)(amp_idx + 1), buf, sizeof(buf));
      draw_line(&disp, fx, x, y_amp, buf, RGB_WHITE);

      make_freq_line("F_IDX=", (uint8_t)(freq_idx + 1), buf, sizeof(buf));
      draw_line(&disp, fx, x, y_freq, buf, RGB_WHITE);
    }

    prev_b0 = b0; prev_b1 = b1; prev_b2 = b2; prev_b3 = b3;

    sleep_msec(20);
  }

  // Unreachable normally
  pwm_destroy(AMP_PWM);
  pwm_destroy(FREQ_PWM);
  display_destroy(&disp);
  pynq_destroy();
  return 0;
}
