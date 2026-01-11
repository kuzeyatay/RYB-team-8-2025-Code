// motor.c  — Address 3 (MOTOR)
// Ring frame: [DST][SRC][LEN][PAYLOAD...]
// Motor receives values meant for it.
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
#include <unistd.h>

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
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static void clear_line(display_t *d, int y, int h, uint16_t bg)
{
  int x1 = 0, y1 = y - h + 2, x2 = DISPLAY_WIDTH - 1, y2 = y + 2;
  x1 = clampi(x1, 0, DISPLAY_WIDTH - 1);
  x2 = clampi(x2, 0, DISPLAY_WIDTH - 1);
  y1 = clampi(y1, 0, DISPLAY_HEIGHT - 1);
  y2 = clampi(y2, 0, DISPLAY_HEIGHT - 1);
  if (x2 < x1 || y2 < y1)
    return;
  displayDrawFillRect(d, x1, y1, x2, y2, bg);
}

// Safe draw_line: truncates string so it never goes off-screen.
static void draw_line(display_t *d, FontxFile *fx, int x, int y, const char *s, uint16_t col)
{
  char buf[64];

  int fw = (g_fw == 0) ? 8 : g_fw;
  int max_chars = (DISPLAY_WIDTH - x) / fw;
  if (max_chars <= 0)
    return;

  if (max_chars > (int)(sizeof(buf) - 1))
    max_chars = sizeof(buf) - 1;

  int n = 0;
  while (n < max_chars && s[n] != '\0')
  {
    buf[n] = s[n];
    n++;
  }
  buf[n] = '\0';

  displayDrawString(d, fx, x, y, (uint8_t *)buf, col);
}

// NEW: draw a rectangular outline around one text-line band
static void draw_frame_for_line(display_t *d, int x, int y, int fh, uint16_t col)
{
  // Match clear_line geometry so frame hugs the same band
  int x1 = x - 2;
  int y1 = y - fh + 2;
  int x2 = DISPLAY_WIDTH - 8; // right margin so it doesn't touch the edge
  int y2 = y + 2;

  x1 = clampi(x1, 0, DISPLAY_WIDTH - 1);
  x2 = clampi(x2, 0, DISPLAY_WIDTH - 1);
  y1 = clampi(y1, 0, DISPLAY_HEIGHT - 1);
  y2 = clampi(y2, 0, DISPLAY_HEIGHT - 1);
  if (x2 <= x1 || y2 <= y1)
    return;

  // If your libpynq lacks displayDrawRect, replace this with 4 lines (see below)
  displayDrawRect(d, x1, y1, x2, y2, col);

  // Fallback (uncomment if displayDrawRect is missing):
  // displayDrawLine(d, x1, y1, x2, y1, col);
  // displayDrawLine(d, x2, y1, x2, y2, col);
  // displayDrawLine(d, x2, y2, x1, y2, col);
  // displayDrawLine(d, x1, y2, x1, y1, col);
}

// --- UART helpers ---
static int timeouted_byte(int ms)
{
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

static int receive_byte(void) { return timeouted_byte(TIMEOUT); }

// --- parsed frame globals (filled by receive_message) ---
static uint8_t g_src = 0;
static uint8_t g_len = 0;
static uint8_t g_payload[MAX_PAY];

// UPDATED: Now non-blocking. Returns -1 immediately if no data.
static int receive_message(void)
{
  if (!uart_has_data(UART_CH))
  {
    return -1;
  }

  int b = receive_byte(); // DST
  if (b < 0)
    return -1;
  uint8_t dst = (uint8_t)b;

  b = receive_byte(); // SRC
  if (b < 0)
    return -1;
  uint8_t src = (uint8_t)b;

  b = receive_byte(); // LEN
  if (b < 0)
    return -1;
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
      if (pb < 0)
        return -2;
      uart_send(UART_CH, (uint8_t)pb);
    }
    return 0;
  }

  // Receive for me
  if (len > MAX_PAY)
    len = MAX_PAY;

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

// [DST][SRC][LEN][PAYLOAD]
static void send_message_impl(uint8_t dst, uint8_t src, const uint8_t payload[], uint8_t len)
{
  uart_send(UART_CH, dst);
  uart_send(UART_CH, src);
  uart_send(UART_CH, len);
  for (int i = 0; i < len; i++)
  {
    uart_send(UART_CH, payload[i]);
  }
}

#define send_message(dst, src, payload) \
  send_message_impl(dst, src, payload, (uint8_t)sizeof(payload))

// region 1..5 -> midpoint %
static int region_mid_duty(int r)
{
  switch (r)
  {
  case 1:
    return 5;
  case 2:
    return 20;
  case 3:
    return 40;
  case 4:
    return 60;
  case 5:
    return 80;
  default:
    return 5;
  }
}

// idx 0..4 -> midpoint %
static int idx_to_percent(uint8_t idx)
{
  if (idx > 4)
    idx = 4;
  return region_mid_duty((int)idx + 1);
}

// Safety check for duty cycle in percent
static void set_pwm_percent(int channel, int percent)
{
  (void)channel;
  if (percent > 90)
  {
    exit(1);
  }
}

// Set PWM from indices 0..4
static void command_motor(int amp_index, int freq_index)
{
  if (amp_index < 0 || amp_index > 4 || freq_index < 0 || freq_index > 4)
  {
    return;
  }

  int dutyA = region_mid_duty(amp_index + 1);
  int dutyF = region_mid_duty(freq_index + 1);

  set_pwm_percent(AMP_CH, dutyA);
  set_pwm_percent(FREQ_CH, dutyF);

  uint32_t period = 100000; // 1 kHz
  pwm_set_duty_cycle(AMP_PWM, (uint32_t)(period * dutyA / 100));
  pwm_set_duty_cycle(FREQ_PWM, (uint32_t)(period * dutyF / 100));
}



// Draw A and F lines with percentages + framed outlines
static void draw_af_lines(display_t *d, FontxFile *fx, int x, int y_amp, int y_freq,
                          uint8_t amp_idx, uint8_t freq_idx, uint16_t color, uint16_t bg, int fh)
{
  clear_line(d, y_amp, fh, bg);
  clear_line(d, y_freq, fh, bg);

  int a_pct = idx_to_percent(amp_idx);
  int f_pct = idx_to_percent(freq_idx);

  char buf[64], num[16];

  strcpy(buf, "A_IDX=");
  itoa_u(amp_idx, num);
  strcat(buf, num);
  strcat(buf, " (");
  itoa_u((unsigned)a_pct, num);
  strcat(buf, num);
  strcat(buf, "%)");
  draw_line(d, fx, x, y_amp, buf, color);
  draw_frame_for_line(d, x, y_amp, fh, color);

  strcpy(buf, "F_IDX=");
  itoa_u(freq_idx, num);
  strcat(buf, num);
  strcat(buf, " (");
  itoa_u((unsigned)f_pct, num);
  strcat(buf, num);
  strcat(buf, "%)");
  draw_line(d, fx, x, y_freq, buf, color);
  draw_frame_for_line(d, x, y_freq, fh, color);
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

  // 1 kHz PWM
  pwm_init(AMP_PWM, 100000);
  pwm_init(FREQ_PWM, 100000);

  // Buttons init
  buttons_init();
  static int restart_hold_ms = 0;

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
  draw_line(&disp, fx, x, y, "Waiting for 'M' (A,F)...", RGB_WHITE);
  y += fh;

  sleep_msec(100);

  // --- INITIAL: set motors to 80% / 80% ---
  uint8_t amp_idx = 4;
  uint8_t freq_idx = 4;
  command_motor((int)amp_idx, (int)freq_idx);

  draw_line(&disp, fx, x, y, "[ALERT] INIT SENT", RGB_YELLOW);
  y += fh;

  y += fh;
  int y_amp = y;
  y += fh;
  int y_freq = y;
  y += fh;

  // initial display (with frames)
  draw_af_lines(&disp, fx, x, y_amp, y_freq, amp_idx, freq_idx, RGB_WHITE, RGB_BLACK, fh);

  // --- button previous states for edge detection (0..3) ---
  int prev_b0 = 0, prev_b1 = 0, prev_b2 = 0, prev_b3 = 0;

  while (1)
  {
    // --- 1) Handle UART messages from master ---
    int r = receive_message();
    if (r > 0 && g_len >= 1)
    {
      uint8_t cmd = g_payload[0];
      if (cmd == 'A')
      {
        uint8_t rsp[] = {'A'};
        send_message(MSTR, MTR, rsp);
      }
      else if (cmd == 'M' && g_len >= 3)
      {
        amp_idx = g_payload[1];
        freq_idx = g_payload[2];

        if (amp_idx > 4)
          amp_idx = 4;
        if (freq_idx > 4)
          freq_idx = 4;

        command_motor((int)amp_idx, (int)freq_idx);

        draw_af_lines(&disp, fx, x, y_amp, y_freq, amp_idx, freq_idx, RGB_WHITE, RGB_BLACK, fh);
      }
    }

    // --- 2) Button controls (4 buttons): A-/A+/F-/F+ ---
    int b0 = get_button_state(0); // AMP -1
    int b1 = get_button_state(1); // AMP +1
    int b2 = get_button_state(2); // FREQ -1
    int b3 = get_button_state(3); // FREQ +1

    bool changed = false;

    if (b0 && !prev_b0)
    {
      if (amp_idx > 0)
        amp_idx--;
      changed = true;
    }
    if (b1 && !prev_b1)
    {
      if (amp_idx < 4)
        amp_idx++;
      changed = true;
    }
    if (b2 && !prev_b2)
    {
      if (freq_idx > 0)
        freq_idx--;
      changed = true;
    }
    if (b3 && !prev_b3)
    {
      if (freq_idx < 4)
        freq_idx++;
      changed = true;
    }

    if (changed)
    {
      command_motor((int)amp_idx, (int)freq_idx);
      draw_af_lines(&disp, fx, x, y_amp, y_freq, amp_idx, freq_idx, RGB_WHITE, RGB_BLACK, fh);
    }

    prev_b0 = b0;
    prev_b1 = b1;
    prev_b2 = b2;
    prev_b3 = b3;

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

    sleep_msec(20);
  }

  // Unreachable
  pwm_destroy(AMP_PWM);
  pwm_destroy(FREQ_PWM);
  display_destroy(&disp);
  pynq_destroy();
  return 0;
}
