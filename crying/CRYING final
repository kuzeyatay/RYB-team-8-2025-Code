//IMPORTANT
#define CRYING_PIN ADC4
// SWITCH ONE AWAY THE SCREEN FOR DEMO
// TOWARD THE SCREEN FOR TEST
// hold BUTTON 0 until MAXVOLTAGE goes away to reset maxvoltage
//end important
// crying module
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
#define CRY 2 // this module
#define MTR 3
#define TIMEOUT 20
#define MAX_PAY 5

#define SAMPLES 3000
#define TIME_BETWEEN_SAMPLES_MS 1
#define START_SAMPLES 5000
// global display so handler can access it
static display_t g_disp;

// -------- itoa_u (no sprintf) ----------
static void itoa_u(unsigned v, char *out)
{
  /*inputs are:
  the calculated bpm,
  the array in which the output should be stored

  outputs are: a value that the pynq lib can use to display the bpm
  */
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

// -------- display helpers ----------
static inline int clampi(int v, int lo, int hi)
{
  /*inputs are: int to clamp,
  minimum of int,
  maximum of int

  outputs are: int between lo and hi

  it makes sure the value inputted lays between the max and min*/
  if (v < lo)
  {
    return lo;
  }
  else if (v > hi)
  {
    return hi;
  }
  else
  {
    return v;
  }
}

static void clear_line(display_t *d, int y, int h, uint16_t bg)
{
  /*inputs are: the display object,
   the top y coord to be cleared,
   the height that needs to be cleared MOVES DOWN FROM Y,
   the color to fill it in with

   NO outputs

   it fills the line with the inputted color, starting from y down for h pixels*/
  int x1 = 0, y1 = y - h + 2, x2 = DISPLAY_WIDTH - 1, y2 = y + 2;

  // make sure x is not to big or small
  x1 = clampi(x1, 0, DISPLAY_WIDTH - 1);
  x2 = clampi(x2, 0, DISPLAY_WIDTH - 1);

  // make sure y is not to big or small
  y1 = clampi(y1, 0, DISPLAY_HEIGHT - 1);
  y2 = clampi(y2, 0, DISPLAY_HEIGHT - 1);

  if (x2 < x1 || y2 < y1)
  {
    return;
  }
  displayDrawFillRect(d, x1, y1, x2, y2, bg);
}

static void draw_line(display_t *d, FontxFile *fx, int x, int y, const char *s, uint16_t col)
{
  /*inputs are: the display object,
  the font,
  left most coord,
  top most coord,
  text,
  text color

  NO outputs

  draws the inputted text on the display.
  */

  displayDrawString(d, fx, x, y, (uint8_t *)s, col);
}

// -------- safe exit on Ctrl+C ----------
static void handle_sigint(int sig __attribute__((unused)))
{
  /*NO inputs
  NO outputs

  used for exiting the program, it shuts of the display and exits the rest safely
  */
  displayFillScreen(&g_disp, RGB_BLACK);
  printf("\n Exited\n");
  display_destroy(&g_disp);
  pynq_destroy();
  exit(0);
}

// -------- uart I/O ----------
static int timeouted_byte(int ms)
{
  /*inputs are: time that pynq will wait before detecting a timeout

  outputs are: the received byte, or -1 if timeout*/
  int waited = 0;
  while (waited < ms)
  {
    if (uart_has_data(UART_CH))
    {
      return (int)uart_recv(UART_CH);
    }
    sleep_msec(1); // 1 ms

    waited += 1;
  }
  return -1; // timeout
}

// convenience wrapper: 10 ms per byte
static int receive_byte(void)
{
  /*call to get uart input*/
  return timeouted_byte(TIMEOUT);
}

// [DST][SRC][LEN][PAYLOAD...]
void send_message(uint8_t dst, uint8_t src, const uint8_t payload[], uint8_t len)
{
  /*inputs are: destination,
  source,
  payload(the message)
  length of message

  NO outputs

  sends the message over uart in the correct format*/
  uart_send(UART_CH, dst);
  uart_send(UART_CH, src);
  uart_send(UART_CH, len); // len must be before the message for the receiver to work
  for (int i = 0; i < len; i++)
  {
    uart_send(UART_CH, payload[i]);
  }
}
/*see send_message() above*/
#define send_message(dst, src, payload) \
  send_message(dst, src, payload, (uint8_t)sizeof(payload))

// -------- globals for parsed message ----------
static uint8_t g_src = 0;
static uint8_t g_len = 0;
static uint8_t g_payload[MAX_PAY];

static int receive_message(void)
{
  /*NO inputs

  outputs are: an int containing the length of a received message, THIS CAN BE 0 OR SMALLER if there is no message

  run this function until you get a length > 0,
  else you are looking at an empty message or one that is not meant for your submodule
  */
  int b;

  b = receive_byte();
  if (b < 0)
    return -1;
  uint8_t dst = (uint8_t)b;

  b = receive_byte();
  if (b < 0)
    return -1;
  uint8_t src = (uint8_t)b;

  b = receive_byte();
  if (b < 0)
    return -1;
  uint8_t len = (uint8_t)b;

  // ring forward if not for me
  if (dst != CRY)
  {
    uart_send(UART_CH, dst);
    uart_send(UART_CH, src);
    uart_send(UART_CH, len);

    for (int i = 0; i < len; i++)
    {
      int pb = receive_byte();
      if (pb < 0)
      {
        return -2; // timeout while forwarding
      }
      uart_send(UART_CH, (uint8_t)pb);
    }
    return 0; // forwarded
  }

  // payload for me
  for (int i = 0; i < len; i++)
  {
    // check if byte has timed out
    b = receive_byte();
    if (b < 0)
    {
      return -3; // timeout during payload
    }
    g_payload[i] = (uint8_t)b;
  }
  g_src = src;
  g_len = len;
  return g_len; // >0 means we have a message for me
}

float measureMaxVoltage(int samples)
{
    //returns the maximum voltage in the samples.
    float highestVoltage = 0;
    for (int i = 0; i < samples; i++) 
    {
        float vin = adc_read_channel(CRYING_PIN);
        if (vin > highestVoltage)
        {
            highestVoltage = vin;
        }
        sleep_msec(TIME_BETWEEN_SAMPLES_MS);
    }
    return highestVoltage;
}

int cryingVolume(float maxVolume)
{
  // returns a percentage volume measured relative to the first calibrations sample.
  float highestVoltage = measureMaxVoltage(SAMPLES);
  float VolumePercentage = clampi(100*(highestVoltage / maxVolume), 0, 100);
  return VolumePercentage;
}

int main(void)
{
  signal(SIGINT, handle_sigint); // make Ctrl+C clean the display

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

  
  // interface you see at boot
  int x = 6, y = fh * 1;
  draw_line(&g_disp, fx, x, y, "CRYING MODULE", RGB_GREEN);
  y += fh;
  draw_line(&g_disp, fx, x, y, "Waiting for 'C'/'A'/'R'...", RGB_WHITE);
  y += fh;
  int y_val = y +fh;
  y += fh;


  adc_init();
  // takes an initial sample to determine max volume.
  float maxVolume = measureMaxVoltage(START_SAMPLES);

  printf("Max Volume = %.3f\n", maxVolume);
  char buf2[32], num2[16];
  strcpy(buf2, "MAX INPUT VOLTAGE="); // string copy
  itoa_u(maxVolume*1000, num2);
  strcat(buf2, num2); // string combine
  strcat(buf2, " mV"); // string combine
  draw_line(&g_disp, fx, x, y_val-fh, buf2, RGB_WHITE);



  // ---- state ----
  uint8_t cry = 0;
  uint8_t prev_cry = 0;
  uint32_t tick = 0;

  while (1)
  {
    prev_cry = cry;
    int button_state = get_button_state(0);
    if (button_state)
    {
      clear_line(&g_disp, y_val-fh, fh, RGB_BLACK);
      maxVolume = measureMaxVoltage(START_SAMPLES);

      printf("Max Volume = %.3f\n", maxVolume);
      char buf3[32], num3[16];
      strcpy(buf3, "MAX INPUT VOLTAGE="); // string copy
      itoa_u(maxVolume*1000, num3);
      strcat(buf3, num3); // string combine
      strcat(buf3, " mV"); // string combine
      draw_line(&g_disp, fx, x, y_val-fh, buf3, RGB_WHITE);
      wait_until_button_released(0);
    }


    if (get_switch_state(1) == 1)
    {
        int r = receive_message();
        if (r <= 0)
        {
        sleep_msec(2);
        continue;
        } // forwarded or timeout/error

        // We have: g_src (MSTR), g_payload[0..g_len-1]
        if (g_len >= 1)
        {
            uint8_t cmd = g_payload[0];

            if (cmd == 'A')
            {
                // Echo 'A' for boot ping
                uint8_t rsp[] = {'A'};
                send_message(MSTR, CRY, rsp);
            }
            else if (cmd == 'R')
            {
                // random output for boot ping
                uint8_t v = (uint8_t)((tick * 97u + 13u) & 0xFFu);
                tick++;
                uint8_t rsp[] = {'R', v};
                send_message(MSTR, CRY, rsp);

                // display it
                clear_line(&g_disp, y_val, fh, RGB_BLACK);
                char buf[32], num[16];
                strcpy(buf, "RND=");
                itoa_u(v, num);
                strcat(buf, num);
                draw_line(&g_disp, fx, x, y_val, buf, RGB_YELLOW);
            }
            else if (cmd == 'C')
            {
                // calculate and send crying percentage
                cry = cryingVolume(maxVolume);
                uint8_t rsp[] = {'C', cry};
                send_message(MSTR, CRY, rsp);
            }
        }
        sleep_msec(20);
    }
    else
    {
      // test setup
        cry = cryingVolume(maxVolume);
        printf("%i\n", cry);
        sleep_msec(10);
    }

    if (prev_cry != cry){
    // display it
    clear_line(&g_disp, y_val, fh, RGB_BLACK);
    char buf[32], num[16];
    strcpy(buf, "PCT="); // string copy
    itoa_u(cry, num);
    strcat(buf, num); // string combine
    strcat(buf, "%"); // string combine
    draw_line(&g_disp, fx, x, y_val, buf, RGB_WHITE);
    }
  }

  display_destroy(&g_disp);
  pynq_destroy();
  return 0;
}