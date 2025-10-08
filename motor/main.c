// motor.c  — Address 3 (MOTOR)
// Ring frame: [DST][SRC][LEN][PAYLOAD...]
// Motor only receives values meant for it and never replies.
// It forwards frames that are NOT for it.

#include <libpynq.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>

#define UART_CH UART0
#define MSTR   0
#define HRTBT  1
#define CRY    2
#define MTR    3       // this module
#define TIMEOUT 20
#define MAX_PAY 8

// --- tiny itoa (no sprintf) ---
static void itoa_u(unsigned v, char *out){
  char tmp[16]; int n=0;
  if(!v){ out[0]='0'; out[1]=0; return; }
  while(v && n<16){ tmp[n++]=(char)('0'+(v%10)); v/=10; }
  for(int i=0;i<n;i++) out[i]=tmp[n-1-i];
  out[n]=0;
}

// --- display helpers ---
static inline int clampi(int v,int lo,int hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }
static void clear_line(display_t *d, int y, int h, uint16_t bg){
  int x1=0, y1=y-h+2, x2=DISPLAY_WIDTH-1, y2=y+2;
  x1=clampi(x1,0,DISPLAY_WIDTH-1); x2=clampi(x2,0,DISPLAY_WIDTH-1);
  y1=clampi(y1,0,DISPLAY_HEIGHT-1); y2=clampi(y2,0,DISPLAY_HEIGHT-1);
  if(x2<x1 || y2<y1) return;
  displayDrawFillRect(d, x1,y1,x2,y2,bg);
}
static void draw_line(display_t *d, FontxFile *fx, int x, int y, const char *s, uint16_t col){
  displayDrawString(d, fx, x, y, (uint8_t*)s, col);
}

// --- UART helpers ---
static int timeouted_byte(int ms) {
  int waited = 0;
  while (waited < ms) {
    if (uart_has_data(UART_CH)) {
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
static int receive_message(void){
  int b;

  b = receive_byte(); if (b < 0) return -1;
  uint8_t dst = (uint8_t)b;

  b = receive_byte(); if (b < 0) return -1;
  uint8_t src = (uint8_t)b;

  b = receive_byte(); if (b < 0) return -1;
  uint8_t len = (uint8_t)b;

  if (dst != MTR) {
    // forward as-is
    uart_send(UART_CH, dst);
    uart_send(UART_CH, src);
    uart_send(UART_CH, len);
    for (int i = 0; i < len; i++) {
      int pb = receive_byte();
      if (pb < 0) return -2;
      uart_send(UART_CH, (uint8_t)pb);
    }
    return 0;
  }

  if (len > MAX_PAY) len = MAX_PAY;
  for (int i = 0; i < len; i++) {
    b = receive_byte();
    if (b < 0) return -3;
    g_payload[i] = (uint8_t)b;
  }
  g_src = src;
  g_len = len;
  return g_len;
}

int main(void){
  // IO init
  pynq_init();
  uart_init(UART_CH);
  uart_reset_fifos(UART_CH);
  switchbox_set_pin(IO_AR0, SWB_UART0_RX);
  switchbox_set_pin(IO_AR1, SWB_UART0_TX);

  // display init
  display_t disp; display_init(&disp);
  display_set_flip(&disp, true, true);
  displayFillScreen(&disp, RGB_BLACK);
  FontxFile fx[2];
  uint8_t glyph[FontxGlyphBufSize], fw, fh;
  InitFontx(fx, "/boot/ILGH16XB.FNT", "");
  GetFontx(fx, 0, glyph, &fw, &fh);
  displaySetFontDirection(&disp, TEXT_DIRECTION0);

  int x = 6, y = fh*1;
  draw_line(&disp, fx, x, y, "MOTOR MODULE", RGB_GREEN); y += fh;
  draw_line(&disp, fx, x, y, "Waiting for 'M' (amp,freq)...", RGB_WHITE); y += fh;
  int y_amp = y; y += fh;
  int y_freq = y; y += fh;

  uint8_t amp = 0;
  uint8_t freq = 0;

  // initial display
  {
    char buf[32], num[16];
    strcpy(buf, "AMP=");
    itoa_u(amp, num); strcat(buf, num);
    draw_line(&disp, fx, x, y_amp, buf, RGB_YELLOW);

    strcpy(buf, "FREQ=");
    itoa_u(freq, num); strcat(buf, num);
    draw_line(&disp, fx, x, y_freq, buf, RGB_YELLOW);
  }

  while (1) {
    int r = receive_message();
    if (r <= 0) { sleep_msec(2); continue; }

    if (g_len >= 1) {
      uint8_t cmd = g_payload[0];

      if (cmd == 'M' && g_len >= 3) {
        // update outputs from payload
        amp  = g_payload[1];          
        freq = g_payload[2];           

        // here you'd drive the actual motor using amp/freq

        // refresh display
        clear_line(&disp, y_amp,  fh, RGB_BLACK);
        clear_line(&disp, y_freq, fh, RGB_BLACK);

        char buf[32], num[16];
        strcpy(buf, "AMP=");
        itoa_u(amp, num); strcat(buf, num);
        draw_line(&disp, fx, x, y_amp, buf, RGB_WHITE);

        strcpy(buf, "FREQ=");
        itoa_u(freq, num); strcat(buf, num);
        draw_line(&disp, fx, x, y_freq, buf, RGB_WHITE);
      }

      // ignore 'A', 'R', or anything else; motor never replies
    }

    sleep_msec(20);
  }

  display_destroy(&disp);
  pynq_destroy();
  return 0;
}

