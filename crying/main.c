//IMPORTANT
#define CRYING_PIN ADC4
// SWITCH ONE AWAY THE SCREEN FOR DEMO
// TOWARD THE SCREEN FOR TEST
// hold BUTTON 0 until MAXVOLTAGE goes away to reset maxvoltage
//end important

#include <libpynq.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#define UART_CH UART0
#define MSTR 0
#define HRTBT 1
#define CRY 2
#define MTR 3

#define TIMEOUT 20
#define MAX_PAY 5

// Old code used: SAMPLES=3000 with 1ms sleep => ~3000ms window.
// This one preserves the same "effective window duration" but do it without sleeping.
// The reason for that is if we sleep for 3s we are responding to a sender that already discarded our message after 20ms has passed
// This is a big issue because responsive submodules must respond immediatly or the response will get lost and the master module will catch it 
// on the next loop, delayin the entire system
#define SAMPLES 3000
#define TIME_BETWEEN_SAMPLES_MS 1
#define START_SAMPLES 5000

#define WINDOW_MS   (SAMPLES * TIME_BETWEEN_SAMPLES_MS)         // ~3000 ms
#define CALIB_MS    (START_SAMPLES * TIME_BETWEEN_SAMPLES_MS)   // ~5000 ms

static display_t g_disp;

// time helper (monotonic ms) 
static uint64_t now_msec(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

//Display
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

// display helpers
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

static void draw_line(display_t *d, FontxFile *fx, int x, int y, const char *s, uint16_t col)
{
  displayDrawString(d, fx, x, y, (uint8_t *)s, col);
}

// -------- safe exit on Ctrl+C ----------
static void handle_sigint(int sig __attribute__((unused)))
{
  displayFillScreen(&g_disp, RGB_BLACK);
  printf("\n Exited\n");
  display_destroy(&g_disp);
  pynq_destroy();
  exit(0);
}

// -------- uart I/O ----------
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

// [DST][SRC][LEN][PAYLOAD...]
void send_message(uint8_t dst, uint8_t src, const uint8_t payload[], uint8_t len)
{
  uart_send(UART_CH, dst);
  uart_send(UART_CH, src);
  uart_send(UART_CH, len);
  for (int i = 0; i < len; i++)
    uart_send(UART_CH, payload[i]);
}
#define send_message(dst, src, payload) \
  send_message(dst, src, payload, (uint8_t)sizeof(payload))

static uint8_t g_src = 0;
static uint8_t g_len = 0;
static uint8_t g_payload[MAX_PAY];

// NON-BLOCKING: if no UART data waiting, return 0 immediately (don’t stall sampling).
static int receive_message(void)
{
  if (!uart_has_data(UART_CH))
    return 0;

  int b;

  b = receive_byte();
  if (b < 0) return -1;
  uint8_t dst = (uint8_t)b;

  b = receive_byte();
  if (b < 0) return -1;
  uint8_t src = (uint8_t)b;

  b = receive_byte();
  if (b < 0) return -1;
  uint8_t len = (uint8_t)b;

  if (dst != CRY)
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

// -------- continuous sampler state ----------
typedef struct
{
  uint64_t win_start_ms;
  float win_max;
} maxwin_t;

static void maxwin_reset(maxwin_t *w, uint64_t now_ms)
{
  w->win_start_ms = now_ms;
  w->win_max = 0.0f;
}

// returns true when a window completes; outputs window max via *out_max
static bool maxwin_update(maxwin_t *w, uint64_t now_ms, float sample, uint32_t window_ms, float *out_max)
{
  if (sample > w->win_max)
    w->win_max = sample;

  if ((now_ms - w->win_start_ms) >= window_ms)
  {
    *out_max = w->win_max;
    // start next window immediately, no sleeping
    w->win_start_ms = now_ms;
    w->win_max = 0.0f;
    return true;
  }
  return false;
}

int main(void)
{
  signal(SIGINT, handle_sigint);

  pynq_init();
  uart_init(UART_CH);
  uart_reset_fifos(UART_CH);
  switchbox_set_pin(IO_AR0, SWB_UART0_RX);
  switchbox_set_pin(IO_AR1, SWB_UART0_TX);
  buttons_init();
  switches_init();

  display_init(&g_disp);
  display_set_flip(&g_disp, true, true);
  displayFillScreen(&g_disp, RGB_BLACK);

  FontxFile fx[2];
  uint8_t glyph[FontxGlyphBufSize], fw, fh;
  InitFontx(fx, "/boot/ILGH16XB.FNT", "");
  GetFontx(fx, 0, glyph, &fw, &fh);
  displaySetFontDirection(&g_disp, TEXT_DIRECTION0);

  int x = 6, y = fh * 1;
  draw_line(&g_disp, fx, x, y, "CRYING MODULE", RGB_GREEN);
  y += fh;
  draw_line(&g_disp, fx, x, y, "Waiting for 'C'/'A'/'R'...", RGB_WHITE);
  y += fh;
  int y_val = y + fh;
  y += fh;

  adc_init();

  // -------- calibration (non-blocking) ----------
  float maxVolume = 0.001f; // avoid divide-by-zero
  bool calibrating = true;
  uint64_t cal_start = now_msec();
  float cal_max = 0.0f;

  // display calibration status
  clear_line(&g_disp, y_val - fh, fh, RGB_BLACK);
  draw_line(&g_disp, fx, x, y_val - fh, "CALIBRATING...", RGB_YELLOW);

  // window for crying measurement
  maxwin_t cry_win;
  maxwin_reset(&cry_win, now_msec());

  uint8_t cry = 0, prev_cry = 0;
  uint32_t tick = 0;

  while (1)
  {
    uint64_t tnow = now_msec();

    // one continuous ADC sample per loop ---
    float vin = adc_read_channel(CRYING_PIN);

    // --- handle (re-)calibration in the background ---
    if (calibrating)
    {
      if (vin > cal_max) cal_max = vin;

      if ((tnow - cal_start) >= CALIB_MS)
      {
        calibrating = false;
        maxVolume = (cal_max > 0.001f) ? cal_max : 0.001f;

        printf("Max Volume = %.3f\n", maxVolume);

        // update display
        clear_line(&g_disp, y_val - fh, fh, RGB_BLACK);
        char buf2[40], num2[16];
        strcpy(buf2, "MAX INPUT VOLTAGE=");
        itoa_u((unsigned)(maxVolume * 1000.0f), num2);
        strcat(buf2, num2);
        strcat(buf2, " mV");
        draw_line(&g_disp, fx, x, y_val - fh, buf2, RGB_WHITE);

        // reset crying window after calibration
        maxwin_reset(&cry_win, tnow);
      }
    }

    // --- Button 0: restart calibration (non-blocking again) ---
    if (get_button_state(0))
    {
      calibrating = true;
      cal_start = tnow;
      cal_max = 0.0f;

      clear_line(&g_disp, y_val - fh, fh, RGB_BLACK);
      draw_line(&g_disp, fx, x, y_val - fh, "CALIBRATING...", RGB_YELLOW);

      wait_until_button_released(0);
    }

    // --- update crying percent every WINDOW_MS using max-over-window ---
    float win_max = 0.0f;
    if (!calibrating)
    {
      if (maxwin_update(&cry_win, tnow, vin, WINDOW_MS, &win_max))
      {
        int pct = clampi((int)(100.0f * (win_max / maxVolume)), 0, 100);
        cry = (uint8_t)pct;
      }
    }

    
    
    
      int r = receive_message(); // now non-blocking when idle
      if (r > 0 && g_len >= 1)
      {
        uint8_t cmd = g_payload[0];

        if (cmd == 'A')
        {
          uint8_t rsp[] = {'A'};
          send_message(MSTR, CRY, rsp);
        }
        else if (cmd == 'R')
        {
          uint8_t v = (uint8_t)((tick * 97u + 13u) & 0xFFu);
          tick++;
          uint8_t rsp[] = {'R', v};
          send_message(MSTR, CRY, rsp);

          clear_line(&g_disp, y_val, fh, RGB_BLACK);
          char buf[32], num[16];
          strcpy(buf, "RND=");
          itoa_u(v, num);
          strcat(buf, num);
          draw_line(&g_disp, fx, x, y_val, buf, RGB_YELLOW);
        }
        else if (cmd == 'C')
        {
          uint8_t rsp[] = {'C', cry};
          send_message(MSTR, CRY, rsp);
        }
      }
    
    //test here (makes more sense to me)
    if (get_switch_state(1) == 1)
    {
      // test setup: just print latest published cry value (updated every WINDOW_MS)
      // (no sleeping)
      if (prev_cry != cry)
        printf("%u\n", (unsigned)cry);
    }

    // display update only when cry changes (This also kinda is useless if we are not in any test mode because it will continously change)
    if (prev_cry != cry)
    {
      prev_cry = cry;
      clear_line(&g_disp, y_val, fh, RGB_BLACK);
      char buf[32], num[16];
      strcpy(buf, "PCT=");
      itoa_u(cry, num);
      strcat(buf, num);
      strcat(buf, "%");
      draw_line(&g_disp, fx, x, y_val, buf, RGB_WHITE);
    }

    
    if (!uart_has_data(UART_CH)) sleep_msec(1); //This slows the sampling down, could remove it. idk
    
  }

  display_destroy(&g_disp);
  pynq_destroy();
  return 0;
}
