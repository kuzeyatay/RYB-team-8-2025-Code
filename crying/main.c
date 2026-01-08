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

// ADC sampling / UI
#define TIME_BETWEEN_SAMPLES_MS 5     // 200 Hz sampling
#define UI_REFRESH_MS 100             // 10 Hz UI refresh

// Peak-to-peak windowing (choose 100–300 ms; 200 ms is a good start)
#define P2P_WINDOW_MS 200
#define P2P_SAMPLES (P2P_WINDOW_MS / TIME_BETWEEN_SAMPLES_MS)

// Calibration duration (ms)
#define CAL_BASELINE_MS 3000          // 3 s quiet
#define CAL_MAX_MS 5000               // 5 s loud playback
#define CAL_BASELINE_SAMPLES (CAL_BASELINE_MS / TIME_BETWEEN_SAMPLES_MS)
#define CAL_MAX_SAMPLES (CAL_MAX_MS / TIME_BETWEEN_SAMPLES_MS)

// global display so handler can access it
static display_t g_disp;

// -------- itoa_u ----------
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

// -------- display helpers ----------
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
  printf("\nExited\n");
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

static int receive_byte(void)
{
  return timeouted_byte(TIMEOUT);
}

// [DST][SRC][LEN][PAYLOAD...]
static void send_message(uint8_t dst, uint8_t src, const uint8_t payload[], uint8_t len)
{
  uart_send(UART_CH, dst);
  uart_send(UART_CH, src);
  uart_send(UART_CH, len);
  for (int i = 0; i < len; i++)
    uart_send(UART_CH, payload[i]);
}

#define SEND_MESSAGE(dst, src, payload) \
  send_message((dst), (src), (payload), (uint8_t)sizeof(payload))

// -------- globals for parsed message ----------
static uint8_t g_src = 0;
static uint8_t g_len = 0;
static uint8_t g_payload[MAX_PAY];

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

  // Forward frames not addressed to me
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
  return (int)g_len;
}

// --- non-blocking time (ms) ---
static uint32_t now_msec_u32(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  uint64_t ms = (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
  return (uint32_t)ms;
}

// -------------------- P2P loudness tracking --------------------
static uint32_t g_last_sample_ms = 0;

static float g_adc_latest = 0.0f;

static float g_win_min = 10.0f;
static float g_win_max = 0.0f;
static int   g_win_count = 0;

static float g_latest_p2p = 0.0f;   // volts
static float g_p2p_quiet  = 0.0f;   // volts (noise-floor p2p)
static float g_p2p_max    = 0.2f;   // volts (reference max p2p)

static float   g_latest_pct = 0.0f;
static uint8_t g_latest_cry = 0;

// Update windowed peak-to-peak (max-min) and map to %
static void cry_sampler_update(void)
{
  uint32_t t = now_msec_u32();
  if ((uint32_t)(t - g_last_sample_ms) < (uint32_t)TIME_BETWEEN_SAMPLES_MS)
    return;
  g_last_sample_ms = t;

  float v = adc_read_channel(ADC0);
  g_adc_latest = v;

  if (v < g_win_min) g_win_min = v;
  if (v > g_win_max) g_win_max = v;
  g_win_count++;

  if (g_win_count >= (int)P2P_SAMPLES)
  {
    float p2p = g_win_max - g_win_min;
    if (p2p < 0.0f) p2p = 0.0f;
    g_latest_p2p = p2p;

    // Map p2p -> percent using quiet/max references
    float denom = (g_p2p_max - g_p2p_quiet);
    if (denom < 0.001f) denom = 0.001f;

    float x = (p2p - g_p2p_quiet) / denom;
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;

    float pct = 100.0f * x;
    g_latest_pct = pct;
    g_latest_cry = (uint8_t)(pct + 0.5f);

    // reset window
    g_win_min = 10.0f;
    g_win_max = 0.0f;
    g_win_count = 0;
  }
}

// --- calibration helpers for p2p ---
// Measure average p2p while quiet (noise floor)
static float measureQuietP2P(int total_samples)
{
  float sum = 0.0f;
  int windows = 0;

  float wmin = 10.0f, wmax = 0.0f;
  int wcount = 0;

  for (int i = 0; i < total_samples; i++)
  {
    float v = adc_read_channel(ADC0);
    if (v < wmin) wmin = v;
    if (v > wmax) wmax = v;
    wcount++;

    if (wcount >= (int)P2P_SAMPLES)
    {
      float p2p = wmax - wmin;
      sum += p2p;
      windows++;

      wmin = 10.0f; wmax = 0.0f; wcount = 0;
    }
    sleep_msec(TIME_BETWEEN_SAMPLES_MS);
  }

  if (windows <= 0) return 0.0f;
  return sum / (float)windows;
}

// Measure robust max p2p during loud playback (average of top 5 window p2p)
static float measureMaxP2P(int total_samples)
{
  float top1=0, top2=0, top3=0, top4=0, top5=0;

  float wmin = 10.0f, wmax = 0.0f;
  int wcount = 0;

  for (int i = 0; i < total_samples; i++)
  {
    float v = adc_read_channel(ADC0);
    if (v < wmin) wmin = v;
    if (v > wmax) wmax = v;
    wcount++;

    if (wcount >= (int)P2P_SAMPLES)
    {
      float p2p = wmax - wmin;

      // keep top 5 window p2p values
      if (p2p > top1) { top5=top4; top4=top3; top3=top2; top2=top1; top1=p2p; }
      else if (p2p > top2) { top5=top4; top4=top3; top3=top2; top2=p2p; }
      else if (p2p > top3) { top5=top4; top4=top3; top3=p2p; }
      else if (p2p > top4) { top5=top4; top4=p2p; }
      else if (p2p > top5) { top5=p2p; }

      wmin = 10.0f; wmax = 0.0f; wcount = 0;
    }

    sleep_msec(TIME_BETWEEN_SAMPLES_MS);
  }

  float avg_top5 = (top1 + top2 + top3 + top4 + top5) / 5.0f;
  return avg_top5;
}

int main(void)
{
  signal(SIGINT, handle_sigint);

  // ---- HW init ----
  pynq_init();
  uart_init(UART_CH);
  uart_reset_fifos(UART_CH);
  switchbox_set_pin(IO_AR0, SWB_UART0_RX);
  switchbox_set_pin(IO_AR1, SWB_UART0_TX);
  buttons_init();
  switches_init();

  // ---- Display init ----
  display_init(&g_disp);
  display_set_flip(&g_disp, true, true);
  displayFillScreen(&g_disp, RGB_BLACK);

  FontxFile fx[2];
  uint8_t glyph[FontxGlyphBufSize], fw, fh;
  InitFontx(fx, "/boot/ILGH16XB.FNT", "");
  GetFontx(fx, 0, glyph, &fw, &fh);
  displaySetFontDirection(&g_disp, TEXT_DIRECTION0);

  int x = 6;
  int y = fh * 1;

  draw_line(&g_disp, fx, x, y, "CRYING MODULE", RGB_GREEN);
  y += fh;
  

  int y_adc = y; y += fh;
  int y_p2p = y; y += fh;
  int y_pct = y; y += fh;

  adc_init();

  // ---- boot calibration (P2P based) ----
  clear_line(&g_disp, y_adc, fh, RGB_BLACK);
  draw_line(&g_disp, fx, x, y_adc, "Calib: QUIET...", RGB_YELLOW);
  g_p2p_quiet = measureQuietP2P(CAL_BASELINE_SAMPLES);

  clear_line(&g_disp, y_adc, fh, RGB_BLACK);
  draw_line(&g_disp, fx, x, y_adc, "Calib: LOUD...", RGB_YELLOW);
  g_p2p_max = measureMaxP2P(CAL_MAX_SAMPLES);

  // safety: ensure separation
  if (g_p2p_max < g_p2p_quiet + 0.02f)
    g_p2p_max = g_p2p_quiet + 0.02f;

  printf("P2P quiet=%f V, P2P max=%f V\n", g_p2p_quiet, g_p2p_max);

  // init runtime sampler
  g_last_sample_ms = now_msec_u32();
  g_win_min = 10.0f; g_win_max = 0.0f; g_win_count = 0;
  g_latest_p2p = 0.0f;
  g_latest_pct = 0.0f;
  g_latest_cry = 0;

  uint32_t last_ui_ms = 0;
  uint32_t tick = 0;

  while (1)
  {
    cry_sampler_update();

    // UI refresh
    uint32_t now = now_msec_u32();
    if ((uint32_t)(now - last_ui_ms) >= (uint32_t)UI_REFRESH_MS)
    {
      last_ui_ms = now;

      // ADC (mV)
      clear_line(&g_disp, y_adc, fh, RGB_BLACK);
      char bufA[32], numA[16];
      unsigned mv = (unsigned)(g_adc_latest * 1000.0f + 0.5f);
      strcpy(bufA, "ADC=");
      itoa_u(mv, numA);
      strcat(bufA, numA);
      strcat(bufA, "mV");
      draw_line(&g_disp, fx, x, y_adc, bufA, RGB_CYAN);

      // P2P (mV)
      clear_line(&g_disp, y_p2p, fh, RGB_BLACK);
      char bufB[32], numB[16];
      unsigned p2pmv = (unsigned)(g_latest_p2p * 1000.0f + 0.5f);
      strcpy(bufB, "P2P=");
      itoa_u(p2pmv, numB);
      strcat(bufB, numB);
      strcat(bufB, "mV");
      draw_line(&g_disp, fx, x, y_p2p, bufB, RGB_CYAN);

      // PCT
      clear_line(&g_disp, y_pct, fh, RGB_BLACK);
      char bufP[32], numP[16];
      strcpy(bufP, "PCT=");
      itoa_u((unsigned)g_latest_cry, numP);
      strcat(bufP, numP);
      strcat(bufP, "%");
      draw_line(&g_disp, fx, x, y_pct, bufP, RGB_WHITE);
    }

   
      // UART mode
      int r = receive_message();
      if (r > 0 && g_len >= 1)
      {
        uint8_t cmd = g_payload[0];

        if (cmd == 'A')
        {
          uint8_t rsp[] = {'A'};
          SEND_MESSAGE(MSTR, CRY, rsp);
        }
        else if (cmd == 'R')
        {
          uint8_t v = (uint8_t)((tick * 97u + 13u) & 0xFFu);
          tick++;
          uint8_t rsp[] = {'R', v};
          SEND_MESSAGE(MSTR, CRY, rsp);
        }
        else if (cmd == 'C')
        {
          uint8_t rsp[] = {'C', g_latest_cry};
          SEND_MESSAGE(MSTR, CRY, rsp);
        }
      }
      sleep_msec(2);
    
    
  }

  display_destroy(&g_disp);
  pynq_destroy();
  return 0;
}
