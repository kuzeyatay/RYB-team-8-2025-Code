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

  // Use global font width; fall back to 8 if not set yet.
  int fw = (g_fw == 0) ? 8 : g_fw;

  // How many characters fit from x to the right edge
  int max_chars = (DISPLAY_WIDTH - x) / fw;
  if (max_chars <= 0)
    return; // nothing fits on this line

  if (max_chars > (int)(sizeof(buf) - 1))
  {
    max_chars = sizeof(buf) - 1;
  }

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

// Receive one frame; forward if not for MTR.
// Return payload length (>0) if for MTR, 0 if forwarded, <0 on timeout/error.
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


// Return the safe midpoint duty (%) for region 1..5
// Regions are:
//   1 -> ~ 0–10%  (mid 5)
//   2 -> ~10–30%  (mid 20)
//   3 -> ~30–50%  (mid 40)
//   4 -> ~50–70%  (mid 60)
//   5 -> ~70–90%  (mid 80)
static int region_mid_duty(int r)
{
  switch (r)
  {
  case 1:
    return 5; // 0–10%  -> mid 5%
  case 2:
    return 20; // 10–30% -> mid 20%
  case 3:
    return 40; // 30–50% -> mid 40%
  case 4:
    return 60; // 50–70% -> mid 60%
  case 5:
    return 80; // 70–90% -> mid 80%
  default:
    return 5;
  }
}

// Safety check for duty cycle in percent
static void set_pwm_percent(int channel, int percent)
{
  (void)channel; // channel kept for future extension

  if (percent > 90)
  {
    printf("[EMERGENCY] duty=%d%% > 90%%\n", percent);
    exit(1); // never allow >90%
  }
}

// amp_index, freq_index are 0..4 (matrix indices for A/F)
// We map index 0..4 -> region 1..5 -> duty midpoints.
static void command_motor(int amp_index, int freq_index)
{
  // safety: indices must be 0..4
  if (amp_index < 0 || amp_index > 4 || freq_index < 0 || freq_index > 4)
  {
    printf("[SYSTEM][ERROR] command_motor out-of-bounds A%d F%d\n",
           amp_index + 1, freq_index + 1);
    return;
  }

  int a_region = amp_index + 1;  // 1..5
  int f_region = freq_index + 1; // 1..5

  int dutyA = region_mid_duty(a_region); // percentage
  int dutyF = region_mid_duty(f_region); // percentage

  // global duty safety
  set_pwm_percent(AMP_CH, dutyA);
  set_pwm_percent(FREQ_CH, dutyF);

  // 1 kHz PWM: period = 100000 steps @ 10 ns base
  uint32_t period = 100000;

  pwm_set_duty_cycle(AMP_PWM, (uint32_t)(period * dutyA / 100));
  pwm_set_duty_cycle(FREQ_PWM, (uint32_t)(period * dutyF / 100));
}

static void handle_sigint(int sig __attribute__((unused)))
{
    displayFillScreen(&disp, RGB_BLACK); // Clear display to black
    printf("\n Exited\n");
    display_destroy(&disp); // De-initialize display
    pynq_destroy();         // De-initialize PYNQ hardware
    exit(0);                // Exit program
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

  // 1 kHz PWM: period = 100000 * 10 ns = 1 ms
  pwm_init(AMP_PWM, 100000);
  pwm_init(FREQ_PWM, 100000);

  // Buttons init
  buttons_init(); // from buttons library

  // display init
  display_init(&disp);
  display_set_flip(&disp, true, true);
  displayFillScreen(&disp, RGB_BLACK);
  FontxFile fx[2];
  uint8_t glyph[FontxGlyphBufSize], fw, fh;
  InitFontx(fx, "/boot/ILGH16XB.FNT", "");
  GetFontx(fx, 0, glyph, &fw, &fh);
  g_fw = fw; // store globally so draw_line can clip safely
  g_fh = fh;
  displaySetFontDirection(&disp, TEXT_DIRECTION0);

  int x = 6, y = fh * 1;
  draw_line(&disp, fx, x, y, "MOTOR MODULE", RGB_GREEN);
  y += fh;
  draw_line(&disp, fx, x, y, "Waiting for 'M' (A,F)...", RGB_WHITE);
  y += fh;
  int y_amp = y;
  y += fh;
  int y_freq = y;
  y += fh;

  uint8_t amp_idx = 4;  // 0..4
  uint8_t freq_idx = 4; // 0..4

  // initial display
  {
    char buf[32], num[16];
    strcpy(buf, "A_IDX=");
    itoa_u(amp_idx, num);
    strcat(buf, num);
    draw_line(&disp, fx, x, y_amp, buf, RGB_YELLOW);

    strcpy(buf, "F_IDX=");
    itoa_u(freq_idx, num);
    strcat(buf, num);
    draw_line(&disp, fx, x, y_freq, buf, RGB_YELLOW);
  }

  // --- button previous states for edge detection (0..3) ---
  int prev_b0 = 0;
  int prev_b1 = 0;
  int prev_b2 = 0;
  int prev_b3 = 0;

  while (1)
  {
    // --- 1) Handle UART messages from master ---
    int r = receive_message();
    if (r > 0 && g_len >= 1)
    {
      uint8_t cmd = g_payload[0];
      if (cmd == 'A')
      {
        // Echo 'A' for boot ping
        uint8_t rsp[] = {'A'};
        send_message(MSTR, MTR, rsp);
      }
      else if (cmd == 'M' && g_len >= 3)
      {
        // payload[1], payload[2] are indices 0..4
        amp_idx = g_payload[1];
        freq_idx = g_payload[2];

        // clamp just in case master misbehaves
        if (amp_idx > 4)
          amp_idx = 4;
        if (freq_idx > 4)
          freq_idx = 4;

        command_motor((int)amp_idx, (int)freq_idx);

        // refresh display
        clear_line(&disp, y_amp, fh, RGB_BLACK);
        clear_line(&disp, y_freq, fh, RGB_BLACK);

        char buf[32], num[16];
        strcpy(buf, "A_IDX=");
        itoa_u(amp_idx, num);
        strcat(buf, num);
        draw_line(&disp, fx, x, y_amp, buf, RGB_WHITE);

        strcpy(buf, "F_IDX=");
        itoa_u(freq_idx, num);
        strcat(buf, num);
        draw_line(&disp, fx, x, y_freq, buf, RGB_WHITE);
      }
    }

    // --- 2) Button controls (4 buttons): A-/A+/F-/F+ ---
    int b0 = get_button_state(0); // AMP -1
    int b1 = get_button_state(1); // AMP +1
    int b2 = get_button_state(2); // FREQ -1
    int b3 = get_button_state(3); // FREQ +1

    bool changed = false;

    // Rising edge: Button0 -> amplitude -1
    if (b0 && !prev_b0)
    {
      if (amp_idx > 0) amp_idx--;
      changed = true;
    }

    // Rising edge: Button1 -> amplitude +1
    if (b1 && !prev_b1)
    {
      if (amp_idx < 4) amp_idx++;
      changed = true;
    }

    // Rising edge: Button2 -> frequency -1
    if (b2 && !prev_b2)
    {
      if (freq_idx > 0) freq_idx--;
      changed = true;
    }

    // Rising edge: Button3 -> frequency +1
    if (b3 && !prev_b3)
    {
      if (freq_idx < 4) freq_idx++;
      changed = true;
    }

    // If anything changed, apply + update display
    if (changed)
    {
      command_motor((int)amp_idx, (int)freq_idx);

      clear_line(&disp, y_amp, fh, RGB_BLACK);
      clear_line(&disp, y_freq, fh, RGB_BLACK);

      char buf[32], num[16];
      strcpy(buf, "A_IDX=");
      itoa_u(amp_idx, num);
      strcat(buf, num);
      draw_line(&disp, fx, x, y_amp, buf, RGB_WHITE);

      strcpy(buf, "F_IDX=");
      itoa_u(freq_idx, num);
      strcat(buf, num);
      draw_line(&disp, fx, x, y_freq, buf, RGB_WHITE);
    }

    prev_b0 = b0;
    prev_b1 = b1;
    prev_b2 = b2;
    prev_b3 = b3;

    sleep_msec(20);
  }

  // (Normally unreachable, but for completeness)
  pwm_destroy(AMP_PWM);
  pwm_destroy(FREQ_PWM);
  display_destroy(&disp);
  pynq_destroy();
  return 0;
}
