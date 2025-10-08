// master.c — MASTER / Decision module
// Ring UART frames: [DST][SRC][LEN][PAYLOAD...]

#include <libpynq.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <signal.h>
#include <time.h>

#define UART_CH UART0
#define MSTR    0
#define HRTBT   1
#define CRY     2
#define MTR     3

#define TIMEOUT          20 //in ms
#define MAX_PAY          5 // this the maximum number of values the payload array can hold. i think 5 is more than enough since used max 3

// global variables for submodules. AKA internal signals
static uint8_t last_bpm = 0;
static uint8_t last_cry = 0;

uint8_t g_amp = 0;
uint8_t g_freq = 0;

//global message variables that are decoded
static uint8_t g_src = 0;
static uint8_t g_len = 0;
static uint8_t g_payload[MAX_PAY];


//some stuff for the display screen that gpt generated that actually works for some reason(maybe look into it someday)
static void itoa_u(unsigned v, char *out){
  char tmp[16]; int n=0;
  if(!v){ out[0]='0'; out[1]=0; return; }
  while(v && n<16){ tmp[n++]=(char)('0'+(v%10)); v/=10; }
  for(int i=0;i<n;i++) out[i]=tmp[n-1-i];
  out[n]=0;
}

static inline int clampi(int v,int lo,int hi){ if(v<lo)return lo; if(v>hi)return hi; return v; }
static void clear_text_line(display_t *d, int y, int h, uint16_t bg){
  int x1=0, y1=y-h+2, x2=DISPLAY_WIDTH-1, y2=y+2;
  x1=clampi(x1,0,DISPLAY_WIDTH-1); x2=clampi(x2,0,DISPLAY_WIDTH-1);
  y1=clampi(y1,0,DISPLAY_HEIGHT-1); y2=clampi(y2,0,DISPLAY_HEIGHT-1);
  if(x2<x1 || y2<y1) return;
  displayDrawFillRect(d, x1,y1,x2,y2,bg);
}
static void draw_text(display_t *d, FontxFile *fx, int x, int y, const char *s, uint16_t col){
  displayDrawString(d, fx, x, y, (uint8_t*)s, col);
}

//UART helpers
// added timeout so that if something is wrong the loop doesnt get stuck
//TODO: add a e stop when the timout exceeds?
static int timeouted_byte(int ms){
  int waited=0;
  while(waited<ms){
    if(uart_has_data(UART_CH)) return (int)uart_recv(UART_CH);
    sleep_msec(1); waited+=1; 
  }
  return -1;
}
static int receive_byte(void){ return timeouted_byte(TIMEOUT); } // conveniance wrapper

// [DST][SRC][LEN][PAYLOAD]
void send_message(uint8_t dst, uint8_t src, const uint8_t payload[], uint8_t len)
{
    uart_send(UART_CH, dst);
    uart_send(UART_CH, src);
    uart_send(UART_CH, len);// len must be before the message for the receiver to work
    for (int i = 0; i < len; i++) {
        uart_send(UART_CH, payload[i]);
    }

    //sends the destination, source, length of the message(necessary for c arrays i guess) and the payload which is the actual message
    
}
#define send_message(dst, src, payload) \
    send_message(dst, src, payload, (uint8_t)sizeof(payload)) //helper macro so we dont the the len stuff every time

static int receive_message(void){
  int b;

  b = receive_byte(); if(b<0) return -1;
  uint8_t dst = (uint8_t)b;//
  b = receive_byte(); if(b<0) return -1;
  uint8_t src = (uint8_t)b;
  b = receive_byte(); if(b<0) return -1;
  uint8_t len = (uint8_t)b;

  if(len > MAX_PAY) len = MAX_PAY;
  for(int i=0;i<len;i++){
    b = receive_byte();
    if(b<0) return -2;
    g_payload[i] = (uint8_t)b;
  }
  // all modules follow the same structure and since uart is serial if dst is send first then its also read first vica versa

  if(dst != MSTR) return 0; // not possible since every message is sent to the master or from the master 
  //and there is no communication between slave modules. Therefore we dont need to implement the ring communication protocol here

  //making messages equal global variables 
  g_src = src;
  g_len = len;
  return g_len;
}

// this is for production. sends a ping to each module to ensure everything is connected
static int boot_ping(uint8_t dst){
  uint8_t payload[] = { 'A' };
  send_message(dst, MSTR, payload);
  int waited=0;
  while(waited<TIMEOUT){
    int r = receive_message();
    if(r>0 && g_src==dst && g_len>=1 && g_payload[0]=='A') return 1;
    sleep_msec(1); waited+=1;
  }
  return 0;
}


// used for the communication demo to send a random value (also gpt generated)
static void motor_send_random(void){
  // seed rand() once
  static bool seeded = false;
  if (!seeded) {
    srand(time(NULL));     // seed with current time
    seeded = true;
  }

  // generate random amplitude and frequency
  g_amp  = (uint8_t)(rand() % 101);          // amplitude 0–100
  g_freq = (uint8_t)(20 + (rand() % 81));    // frequency 20–100

  // build and send UART frame
  uint8_t pl[] = { 'M', g_amp, g_freq };
  send_message(MTR, MSTR, pl);               // motor doesn’t reply
}

// used to request random data from eighter the crying submodule or the heartbeat submodule
static int request_random(uint8_t dst){
  uint8_t payload[] = { 'R' };
  send_message(dst, MSTR, payload);

  int waited = 0;
  while (waited < TIMEOUT) {
    int r = receive_message();
    if (r > 0 && g_src == dst && g_len >= 2 && g_payload[0] == 'R') {
      // For hrtbt and cry, return a single random byte
      return g_payload[1];
    }

    sleep_msec(1);
    waited += 1;
  }

  return -1; // timeout
}

//requests heartbeat
static int request_heartbeat(void){
  uint8_t payload[] = { 'H' };
  send_message(HRTBT, MSTR, payload);
  int waited=0;
  while(waited<TIMEOUT){
    int r = receive_message();
    if(r>0 && g_src==HRTBT && g_len>=2 && g_payload[0]=='H') return g_payload[1];
    sleep_msec(1); waited+=1;
  }
  return -1;
}
//requests crying
static int request_crying(void){
   uint8_t payload[] = { 'C' };
  send_message(CRY, MSTR, payload);
  int waited=0;
  while(waited<TIMEOUT){
    int r = receive_message();
    if(r>0 && g_src==CRY && g_len>=2 && g_payload[0]=='C') return g_payload[1];
    sleep_msec(1); waited+=1;
  }
  return -1;
}
// commands the motor to a certain amp and freq
static void command_motor(uint8_t amp, uint8_t freq){
  uint8_t payload[] = { 'M', amp, freq };
  send_message(MTR, MSTR, payload);
}

// global display variable also used by the sigint
static display_t g_disp;

// when ctrl+c is clicked this function works
static void handle_sigint(int sig __attribute__((unused))){
  displayFillScreen(&g_disp, RGB_BLACK);
  printf("\n Exited\n");
  display_destroy(&g_disp);
  pynq_destroy();
  exit(0);
}

int main(void){
  // initialize the ctrl+c cleaner
  signal(SIGINT, handle_sigint);

  //init stuff for the pynq
  pynq_init();
  uart_init(UART_CH);
  uart_reset_fifos(UART_CH);
  switchbox_set_pin(IO_AR0, SWB_UART0_RX);
  switchbox_set_pin(IO_AR1, SWB_UART0_TX);
  switches_init();

  display_init(&g_disp);
  display_set_flip(&g_disp, true, true);
  displayFillScreen(&g_disp, RGB_BLACK);
  FontxFile fx[2]; uint8_t glyph[FontxGlyphBufSize], fw, fh;
  InitFontx(fx, "/boot/ILGH16XB.FNT", "");
  GetFontx(fx, 0, glyph, &fw, &fh);
  displaySetFontDirection(&g_disp, TEXT_DIRECTION0);

  int x=6, y=fh*1;

  // if switch 0 is turned on before the code starts, than it auto enters the demo mode and stays in it until the swith is off
  if(get_switch_state(0)==1){
    //text stuff
    draw_text(&g_disp, fx, x, y, "COMMUNICATION DEMO MODE", RGB_GREEN); y+=fh;
    draw_text(&g_disp, fx, x, y, "[BOOT]: requesting random", RGB_WHITE); y+=fh;
    draw_text(&g_disp, fx, x, y, "data from all modules", RGB_WHITE); y+=fh;
    draw_text(&g_disp, fx, x, y, "[BOOT]: pinging modules...", RGB_WHITE); y+=fh;y+=fh;

    int v; // just a temp variable that holds the rand value from heartbeat or crying submodules
    int successCounter = 0; // i just wanted to do this

    draw_text(&g_disp, fx, x, y, "Heartbeat @1: ...", RGB_WHITE);
  
    //heartbeat
    v = request_random(HRTBT);
    clear_text_line(&g_disp, y, fh, RGB_BLACK);
    if(v>=0){
      successCounter++;
      char buf[32], num[16]; strcpy(buf,"HB @1: ALIVE:");
      itoa_u((unsigned)v, num); strcat(buf, num);
      draw_text(&g_disp, fx, x, y, buf, RGB_YELLOW);
    }else{
      draw_text(&g_disp, fx, x, y, "HB @1: FAILED", RGB_RED);
    }
    y+=fh;
    draw_text(&g_disp, fx, x, y, "Crying @2: ...", RGB_WHITE);

    //crying
    v = request_random(CRY);
    clear_text_line(&g_disp, y, fh, RGB_BLACK);
    if(v>=0){
       successCounter++;
      char buf[32], num[16]; strcpy(buf,"CRY @2: ALIVE:");
      itoa_u((unsigned)v, num); strcat(buf, num);
      draw_text(&g_disp, fx, x, y, buf, RGB_YELLOW);
    }else{
      draw_text(&g_disp, fx, x, y, "CRY @2: FAILED", RGB_RED);
    }
    y+=fh;

    //motor
draw_text(&g_disp, fx, x, y, "Motor @3: ...", RGB_WHITE);
motor_send_random();// send random A/F to motor
clear_text_line(&g_disp, y, fh, RGB_BLACK);

successCounter++;// treat as success since we sent it and idk if it worked
char buf[48], a[16], f[16];
strcpy(buf, "MTR @3: SENT A:");
itoa_u((unsigned)g_amp, a); strcat(buf, a);
strcat(buf, " F:");
itoa_u((unsigned)g_freq, f); strcat(buf, f);

draw_text(&g_disp, fx, x, y, buf, RGB_YELLOW);
    y+=fh;
    y+=fh;
    printf("%d",successCounter);
    if (successCounter==1) { draw_text(&g_disp, fx, x, y, "DEMO FAILED",    RGB_PURPLE); y += fh; }
    if (successCounter==3) { draw_text(&g_disp, fx, x, y, "DEMO PASSED",    RGB_GREEN); y += fh; }
    while(get_switch_state(0)==1) sleep_msec(10);
  }
 //The real product demo starts here if comms demo is not initiated or turned off.
  displayFillScreen(&g_disp, RGB_BLACK);
  y = fh*1;
  draw_text(&g_disp, fx, x, y, "DECISION MAKING MODULE", RGB_GREEN); y+=fh;
  draw_text(&g_disp, fx, x, y, "[BOOT]: pinging modules...", RGB_WHITE); y+=fh;
  //pinging first to see if the ring works functional

  int y_hb=y; y+=fh; // these are just spaces btf
  int y_cr=y; y+=fh;
  int y_mt=y; y+=fh;

  draw_text(&g_disp, fx, x, y_hb, "HB @1: ...", RGB_WHITE);
  int hb_ok = boot_ping(HRTBT);//send ping to heartbeat and expect ping back
  clear_text_line(&g_disp, y_hb, fh, RGB_BLACK);
  draw_text(&g_disp, fx, x, y_hb, hb_ok ? "HB @1: ALIVE" : "HB @1: MISSING", hb_ok?RGB_GREEN:RGB_RED);

  draw_text(&g_disp, fx, x, y_cr, "CRY @2: ...", RGB_WHITE);
  int cry_ok = boot_ping(CRY);//send ping to motor and expect ping back
  clear_text_line(&g_disp, y_cr, fh, RGB_BLACK);
  draw_text(&g_disp, fx, x, y_cr, cry_ok ? "CRY @2: ALIVE" : "CRY @2: MISSING", cry_ok?RGB_GREEN:RGB_RED);

  draw_text(&g_disp, fx, x, y_mt, "MTR @3: ...", RGB_WHITE);
  int mtr_ok = boot_ping(MTR);//send ping to motor and expect ping back
  clear_text_line(&g_disp, y_mt, fh, RGB_BLACK);
  draw_text(&g_disp, fx, x, y_mt, mtr_ok ? "MTR @3: ALIVE" : "MTR @3: MISSING", mtr_ok?RGB_GREEN:RGB_RED);

  if(!hb_ok) draw_text(&g_disp, fx, x, y+=fh, "[WARN] HB missing, BPM=80", RGB_YELLOW);
  if(!cry_ok)draw_text(&g_disp, fx, x, y+=fh, "[WARN] CRY missing, %=0",  RGB_YELLOW);
  if(!mtr_ok)draw_text(&g_disp, fx, x, y+=fh, "[WARN] MOTOR missing",    RGB_YELLOW);

  int y_live_hb  = y + fh;
  int y_live_cry = y + 2*fh;

  while(1){
    int vhb = request_heartbeat();
    if(vhb >= 0) last_bpm = (uint8_t)vhb;

    int vcr = request_crying();
    if(vcr >= 0) last_cry = (uint8_t)vcr;

    uint8_t amp = (last_cry > 90) ? 90 : last_cry;
    uint8_t freq = 30;
    if      (last_bpm > 120) freq = 60;
    else if (last_bpm > 100) freq = 50;
    else if (last_bpm >  80) freq = 40;

    if(mtr_ok) command_motor(amp, freq);

    clear_text_line(&g_disp, y_live_hb,  fh, RGB_BLACK);
    clear_text_line(&g_disp, y_live_cry, fh, RGB_BLACK);
    char buf[32], num[16];

    strcpy(buf, "[HB] bpm="); itoa_u(last_bpm, num); strcat(buf, num);
    draw_text(&g_disp, fx, x, y_live_hb, buf, RGB_WHITE);

    strcpy(buf, "[C] cry=");  itoa_u(last_cry, num); strcat(buf, num); strcat(buf, "%");
    draw_text(&g_disp, fx, x, y_live_cry, buf, RGB_WHITE);

    sleep_msec(200);
  }

  display_destroy(&g_disp);
  pynq_destroy();
  return EXIT_SUCCESS;
}
