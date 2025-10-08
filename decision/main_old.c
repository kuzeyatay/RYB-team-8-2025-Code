
// sends requests to addr 1(HB), 2(CRY), 3(MOTOR) in a ring.
// [DST][SRC][LEN][PAYLOAD]  (PAYLOAD begins with 'H' / 'C' / 'M')


#include <libpynq.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <signal.h>


#define UART_CH UART0 
#define MSTR 0
#define HRTBT 1
#define CRY 2
#define MTR 3
#define TIMEOUT 20


uint8_t last_bpm = 0;
uint8_t  last_cry = 0;

// display
display_t display;  

// This function runs when Ctrl+C is pressed to clean up the screen because its annoying asf. you can add any ending stuff here
void handle_sigint(int sig __attribute__((unused))) {
    displayFillScreen(&display, RGB_BLACK);
    printf("\nExited\n");
    display_destroy(&display);
    pynq_destroy();
    exit(0);
}

static int timeouted_byte(int ms) {
    int waited = 0;
    while (waited < ms) {
        if (uart_has_data(UART_CH)) {
            return (int)uart_recv(UART_CH);
        }
        sleep_msec(1); // 1 ms
        
        waited += 1;
    }
    return -1; // timeout
}

// convenience wrapper: 10 ms per byte
static int receive_byte(void) {
    return timeouted_byte(TIMEOUT);
}    

// Receive ONE message and print its fields.
// [DST][SRC][LEN][PAYLOAD]
int receive_message(void)
{
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

    
    if (dst != MSTR) {
    // ring forwarding
    // If not for master, forward the frame as-is and exit.
    if (dst != MSTR) {
        // re-send
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
    }

  
    uint8_t payload[len];
    for (int i = 0; i < len; i++) {
        b = receive_byte();
        if (b < 0) return -2;           
        payload[i] = (uint8_t)b;
    }

    
    if (len >= 2 && payload[0] == 'H' && src == HRTBT) {
        return payload[1];               
    }
    if (len >= 2 && payload[0] == 'C' && src == CRY) {
        return payload[1];               
    }
    if (len >= 1 && payload[0] == 'A' && (src == CRY || src == HRTBT || src == MTR)) {
        return 1;                        
    }
     if (len >= 1 && payload[0] == 'R' && (src == CRY || src == HRTBT || src == MTR)) {
        return payload[1];                        
    }

   
    return 0;
}

// [DST][SRC][LEN][PAYLOAD]
void send_message(uint8_t dst, uint8_t src, const uint8_t payload[], uint8_t len)
{
    uart_send(UART_CH, dst);
    uart_send(UART_CH, src);
    uart_send(UART_CH, len);// len must be before the message for the receiver to work
    for (int i = 0; i < len; i++) {
        uart_send(UART_CH, payload[i]);
    }
    
}
#define send_message(dst, src, payload) \
    send_message(dst, src, payload, (uint8_t)sizeof(payload)) //helper macro  c doesnt have overloading i hate this

static int boot_ping(uint8_t dst) {
  uint8_t payload[] = { 'A' };
  send_message(dst, MSTR, payload);      // ask: are you alive?
  if (receive_message()==1) return 1;
  return 0;
}
// modules must return a random value in a byte
static int receive_random(uint8_t dst) {
  uint8_t payload[] = { 'R' };
  send_message(dst, MSTR, payload);      
  if (receive_message()==1) return 1;
  return 0;
}

void receive_heartbeat(){
    uint8_t payload[] = { 'H' };
    send_message(HRTBT,MSTR, payload);
    last_bpm = receive_message();
    
  }
  void receive_crying(){
    uint8_t payload[] = { 'C' };
    send_message(CRY,MSTR,payload);
    last_cry = receive_message();
   
  }

  void send_motor( uint8_t amp, uint8_t freq){
  uint8_t payload[] = { 'M', amp, freq };
  send_message(MTR,MSTR,payload);

  }

//display

static void itoa_u(unsigned v, char *out) {
    // minimal unsigned int → string
    char tmp[16];
    int n = 0;
    if (v == 0) { out[0] = '0'; out[1] = '\0'; return; }
    while (v > 0 && n < (int)sizeof(tmp)) { tmp[n++] = (char)('0' + (v % 10)); v /= 10; }
    // reverse
    for (int i = 0; i < n; i++) out[i] = tmp[n-1-i];
    out[n] = '\0';
}
static inline int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void clear_text_line(display_t *dsp, int y_baseline, int line_h, uint16_t color) {
  // Intended area: from just above the baseline to just below it
  int x1 = 0;
  int y1 = y_baseline - line_h + 2;
  int x2 = DISPLAY_WIDTH  - 1;     // inclusive max
  int y2 = y_baseline + 2;

  // Clamp to screen
  x1 = clampi(x1, 0, DISPLAY_WIDTH  - 1);
  x2 = clampi(x2, 0, DISPLAY_WIDTH  - 1);
  y1 = clampi(y1, 0, DISPLAY_HEIGHT - 1);
  y2 = clampi(y2, 0, DISPLAY_HEIGHT - 1);

  // If the box collapsed, skip drawing
  if (x2 < x1 || y2 < y1) return;

  displayDrawFillRect(dsp, x1, y1, x2, y2, color);
}

static void draw_text_line(display_t *dsp, FontxFile *fx, int x, int y, const char *s, uint16_t color) {
    displayDrawString(dsp, fx, x, y, (uint8_t*)s, color);
}

static void spinner_step(display_t *dsp, FontxFile *fx, int x, int y, int step) {
    const char spin[4] = { '|', '/', '-', '\\' };
    char s[2] = { spin[step & 3], 0 };
    // draw one char “over” the spot (you can offset x to place nicely)
    displayDrawString(dsp, fx, x, y, (uint8_t*)s, RGB_WHITE);
}

int main(void) {
  // for ctrl c to clear screen
   signal(SIGINT, handle_sigint);

  pynq_init();
  uart_init(UART_CH);
  uart_reset_fifos(UART_CH);
  switchbox_set_pin(IO_AR0, SWB_UART0_RX); 
  switchbox_set_pin(IO_AR1, SWB_UART0_TX);

  switches_init();
  

  // DISPLAY INIT
  display_init(&display);
  display_set_flip(&display, true, true);
  displayFillScreen(&display, RGB_BLACK);



  uint8_t buffer_fx16G[FontxGlyphBufSize];
  uint8_t fontW, fontH;
  FontxFile fx[2];
  InitFontx(fx, "/boot/ILGH16XB.FNT", "");
  GetFontx(fx, 0, buffer_fx16G, &fontW, &fontH);
  displaySetFontDirection(&display, TEXT_DIRECTION0);
   // cursor
  int x0 = 6;
  int y  = fontH * 1;
  bool ranDemo = true;

 while(get_switch_state(0)==1){
 

if(ranDemo){
draw_text_line(&display, fx, x0, y, "COMMUNICATION DEMO MODE", RGB_GREEN);
 y += fontH;
 draw_text_line(&display, fx, x0, y, "[BOOT]: requesting random", RGB_WHITE);
y += fontH;
 draw_text_line(&display, fx, x0, y, "data from all modules", RGB_WHITE);
  y += fontH;
 

  // Lines reserved for HB/CRY/MTR results
  int y_hb0  = y; y += fontH;
  int y_cry0 = y; y += fontH;
  int y_mtr0 = y; y += fontH;

  // simple loading spinner while pinging each device
  // HB
  clear_text_line(&display, y_hb0, fontH, RGB_BLACK);
  draw_text_line(&display, fx, x0, y_hb0, "Heartbeat @1: ", RGB_WHITE);
  int hb_ok0  = receive_random(HRTBT);
  for (int t=0; t<8; t++) { spinner_step(&display, fx, x0 + 80, y_hb0, t); sleep_msec(100); }
  clear_text_line(&display, y_hb0, fontH, RGB_BLACK);
 
  char buf[48]; 
  char num[16];

  if (hb_ok0) {
    // example: show a fake number (hb_ok0) or some other value
    itoa_u((unsigned)hb_ok0, num);
    buf[0] = 0;
    strcat(buf, "HB @1: ALIVE: ");
    strcat(buf, num);
    draw_text_line(&display, fx, x0, y_hb0, buf, RGB_GREEN);
  } else {
    draw_text_line(&display, fx, x0, y_hb0, "HB @1: FAILED", RGB_RED);
  }


  // CRY
  clear_text_line(&display, y_cry0, fontH, RGB_BLACK);
  draw_text_line(&display, fx, x0, y_cry0, "Crying @2: ", RGB_WHITE);
  int cry_ok0 = receive_random(CRY); 
  for (int t=0; t<8; t++) { spinner_step(&display, fx, x0 + 90, y_cry0, t); sleep_msec(100); }
  clear_text_line(&display, y_cry0, fontH, RGB_BLACK);
  
  if (cry_ok0) {
    itoa_u((unsigned)cry_ok0, num);
    buf[0] = 0;
    strcat(buf, "CRY @2: ALIVE: ");
    strcat(buf, num);
    draw_text_line(&display, fx, x0, y_cry0, buf, RGB_GREEN);
  } else {
    draw_text_line(&display, fx, x0, y_cry0, "CRY @2: FAILED", RGB_RED);
  }

  // MTR
  clear_text_line(&display, y_mtr0, fontH, RGB_BLACK);
  draw_text_line(&display, fx, x0, y_mtr0, "Motor @3: ", RGB_WHITE);
  int mtr_ok0 = receive_random(MTR);
  for (int t=0; t<8; t++) { spinner_step(&display, fx, x0 + 90, y_mtr0, t); sleep_msec(100); }
  clear_text_line(&display, y_mtr0, fontH, RGB_BLACK);
  
  if (mtr_ok0) {
    itoa_u((unsigned)mtr_ok0, num);
    buf[0] = 0;
    strcat(buf, "MTR @3: ALIVE: ");
    strcat(buf, num);
    draw_text_line(&display, fx, x0, y_mtr0, buf, RGB_GREEN);
  } else {
    draw_text_line(&display, fx, x0, y_mtr0, "MTR @3: FAILED", RGB_RED);
  }

  // WARNINGS (optional)
  y += fontH;
  if (!hb_ok0)  { draw_text_line(&display, fx, x0, y, "[WARN]: Heartbeat test failed", RGB_YELLOW); y += fontH; }
  if (!cry_ok0) { draw_text_line(&display, fx, x0, y, "[WARN]: Crying test failed", RGB_YELLOW); y += fontH; }
  if (!mtr_ok0) { draw_text_line(&display, fx, x0, y, "[WARN]: Motor test failed",    RGB_YELLOW); y += fontH; }
  y += fontH;
  if (!hb_ok0 && !mtr_ok0 && !cry_ok0) { draw_text_line(&display, fx, x0, y, "TEST FAILED MISERABLY",    RGB_RED); y += fontH; }
  ranDemo=false;
}
}
x0 = 6;
y  = fontH * 1;
  displayFillScreen(&display, RGB_BLACK);

  // Title
  draw_text_line(&display, fx, x0, y, "DECISION MAKING MODULE", RGB_GREEN);
  y += fontH;

  // BOOT header
  draw_text_line(&display, fx, x0, y, "[BOOT]: pinging modules...", RGB_WHITE);
  y += fontH;

  // Lines reserved for HB/CRY/MTR results
  int y_hb  = y; y += fontH;
  int y_cry = y; y += fontH;
  int y_mtr = y; y += fontH;

  // simple loading spinner while pinging each device
  // HB
  clear_text_line(&display, y_hb, fontH, RGB_BLACK);
  draw_text_line(&display, fx, x0, y_hb, "HB @1: ", RGB_WHITE);
  int hb_ok  = boot_ping(HRTBT);
  for (int t=0; t<8; t++) { spinner_step(&display, fx, x0 + 80, y_hb, t); sleep_msec(100); }
  clear_text_line(&display, y_hb, fontH, RGB_BLACK);
  draw_text_line(&display, fx, x0, y_hb, hb_ok ? "HB @1: ALIVE" : "HB @1: MISSING", hb_ok ? RGB_GREEN : RGB_RED);

  // CRY
  clear_text_line(&display, y_cry, fontH, RGB_BLACK);
  draw_text_line(&display, fx, x0, y_cry, "CRY @2: ", RGB_WHITE);
  int cry_ok = boot_ping(CRY);
  for (int t=0; t<8; t++) { spinner_step(&display, fx, x0 + 90, y_cry, t); sleep_msec(100); }
  clear_text_line(&display, y_cry, fontH, RGB_BLACK);
  draw_text_line(&display, fx, x0, y_cry, cry_ok ? "CRY @2: ALIVE" : "CRY @2: MISSING", cry_ok ? RGB_GREEN : RGB_RED);

  // MTR
  clear_text_line(&display, y_mtr, fontH, RGB_BLACK);
  draw_text_line(&display, fx, x0, y_mtr, "MTR @3: ", RGB_WHITE);
  int mtr_ok = boot_ping(MTR);
  for (int t=0; t<8; t++) { spinner_step(&display, fx, x0 + 90, y_mtr, t); sleep_msec(100); }
  clear_text_line(&display, y_mtr, fontH, RGB_BLACK);
  draw_text_line(&display, fx, x0, y_mtr, mtr_ok ? "MTR @3: ALIVE" : "MTR @3: MISSING", mtr_ok ? RGB_GREEN : RGB_RED);

  // WARNINGS (optional)
  y += fontH;
  if (!hb_ok)  { draw_text_line(&display, fx, x0, y, "[WARN]: HB missing, BPM=80.", RGB_YELLOW); y += fontH; }
  if (!cry_ok) { draw_text_line(&display, fx, x0, y, "[WARN]: CRY missing, % =0.", RGB_YELLOW); y += fontH; }
  if (!mtr_ok) { draw_text_line(&display, fx, x0, y, "[WARN]: MOTOR missing.",    RGB_YELLOW); y += fontH; }

  last_bpm = hb_ok  ? 80 : 80;
  last_cry = cry_ok ?  0 :  0;

  // live status area
  int y_live_hb  = y + fontH;     // reserve 2 lines
  int y_live_cry = y + 2*fontH;

  // MAIN LOOP (example draws the most recent values)
  while (1) {
    // If you actually poll here, call receive_heartbeat()/receive_crying() before drawing.
    // For now, just re-draw the current values.

    // draw [HB] bpm=NNN
    clear_text_line(&display, y_live_hb, fontH, RGB_BLACK);
    char buf[64]; char num[12];
    strcpy(buf, "[HB] bpm=");
    itoa_u(last_bpm, num);
    strcat(buf, num);
    draw_text_line(&display, fx, x0, y_live_hb, buf, RGB_WHITE);

    // draw [C] Cry=NN%
    clear_text_line(&display, y_live_cry, fontH, RGB_BLACK);
    strcpy(buf, "[C] Cry=");
    itoa_u(last_cry, num);
    strcat(buf, num);
    strcat(buf, "%");
    draw_text_line(&display, fx, x0, y_live_cry, buf, RGB_WHITE);

    sleep_msec(20); // ~50 Hz
  }

  display_destroy(&display);
  pynq_destroy();
  return EXIT_SUCCESS;
} 