// crying.c  — Address 2 (CRYING)
#include <libpynq.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#define UART_CH UART0
#define MSTR   0
#define HRTBT  1
#define CRY    2  // this module
#define MTR    3
#define TIMEOUT 20
#define MAX_PAY 5

// global display so handler can access it
static display_t g_disp;

// -------- itoa_u (no sprintf) ----------
static void itoa_u(unsigned v, char *out){
  char tmp[16]; int n=0;
  if(!v){ out[0]='0'; out[1]=0; return; }
  while(v && n<16){ tmp[n++]=(char)('0'+(v%10)); v/=10; }
  for(int i=0;i<n;i++) out[i]=tmp[n-1-i];
  out[n]=0;
}

// -------- display helpers ----------
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

// -------- safe exit on Ctrl+C ----------
static void handle_sigint(int sig __attribute__((unused))){
  displayFillScreen(&g_disp, RGB_BLACK);
  printf("\n Exited\n");
  display_destroy(&g_disp);
  pynq_destroy();
  exit(0);
}

// -------- uart I/O ----------
static int timeouted_byte(int ms) {
  int waited = 0;
  while (waited < ms) {
    if (uart_has_data(UART_CH)) return (int)uart_recv(UART_CH);
    sleep_msec(1);
    waited += 1;
  }
  return -1;
}
static int receive_byte(void) { return timeouted_byte(TIMEOUT); }

// [DST][SRC][LEN][PAYLOAD...]
static void send_message(uint8_t dst, uint8_t src, const uint8_t payload[], uint8_t len){
  uart_send(UART_CH, dst);
  uart_send(UART_CH, src);
  uart_send(UART_CH, len);
  for (int i = 0; i < len; i++) uart_send(UART_CH, payload[i]);
}
#define send_message(dst, src, payload) \
  send_message(dst, src, payload, (uint8_t)sizeof(payload))

// -------- globals for parsed message ----------
static uint8_t g_src = 0;
static uint8_t g_len = 0;
static uint8_t g_payload[MAX_PAY];

static int receive_message(void){
  int b;

  b = receive_byte(); if (b < 0) return -1;
  uint8_t dst = (uint8_t)b;

  b = receive_byte(); if (b < 0) return -1;
  uint8_t src = (uint8_t)b;

  b = receive_byte(); if (b < 0) return -1;
  uint8_t len = (uint8_t)b;

  if (dst != CRY) {
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
  signal(SIGINT, handle_sigint); // make Ctrl+C clean the display

  pynq_init();
  uart_init(UART_CH);
  uart_reset_fifos(UART_CH);
  switchbox_set_pin(IO_AR0, SWB_UART0_RX);
  switchbox_set_pin(IO_AR1, SWB_UART0_TX);

  display_init(&g_disp);
  display_set_flip(&g_disp, true, true);
  displayFillScreen(&g_disp, RGB_BLACK);
  FontxFile fx[2];
  uint8_t glyph[FontxGlyphBufSize], fw, fh;
  InitFontx(fx, "/boot/ILGH16XB.FNT", "");
  GetFontx(fx, 0, glyph, &fw, &fh);
  displaySetFontDirection(&g_disp, TEXT_DIRECTION0);

  int x=6, y=fh*1;
  draw_line(&g_disp, fx, x, y, "CRYING MODULE", RGB_GREEN); y+=fh;
  draw_line(&g_disp, fx, x, y, "Waiting for 'C'/'A'/'R'...", RGB_WHITE); y+=fh;
  int y_val = y; y+=fh;

  uint8_t cry = 0;
  uint32_t tick = 0;

  while (1) {
    int r = receive_message();
    if (r <= 0) { sleep_msec(2); continue; }

    if (g_len >= 1) {
      uint8_t cmd = g_payload[0];

      if (cmd == 'A') {
        uint8_t rsp[] = { 'A' };
        send_message(MSTR, CRY, rsp);

      } else if (cmd == 'R') {
        uint8_t v = (uint8_t)((tick*97u + 13u) & 0xFFu);
        tick++;
        uint8_t rsp[] = { 'R', v };
        send_message(MSTR, CRY, rsp);

        clear_line(&g_disp, y_val, fh, RGB_BLACK);
        char buf[32], num[16];
        strcpy(buf, "RND=");
        itoa_u(v, num); strcat(buf, num);
        draw_line(&g_disp, fx, x, y_val, buf, RGB_YELLOW);

      } else if (cmd == 'C') {
        cry = (uint8_t)((cry + 7) % 101);
        uint8_t rsp[] = { 'C', cry };
        send_message(MSTR, CRY, rsp);

        clear_line(&g_disp, y_val, fh, RGB_BLACK);
        char buf[32], num[16];
        strcpy(buf, "PCT=");
        itoa_u(cry, num); strcat(buf, num); strcat(buf, "%");
        draw_line(&g_disp, fx, x, y_val, buf, RGB_WHITE);
      }
    }
    sleep_msec(20);
  }

  display_destroy(&g_disp);
  pynq_destroy();
  return 0;
}
