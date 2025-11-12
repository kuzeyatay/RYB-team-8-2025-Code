#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <windows.h>

#define AMP_CH 0
#define FREQ_CH 1 // example, this is how its probably going to look like in production
#define HIST_MAX 2048
#define CONVERGENCE_TIME 4

double Sopt[10];
double BandLow[10];
double BandHigh[10];
int K[5][5];     // stress matrix (internal to sim; controller won't read it)
int curA = 4;    // start at A5 (row index 4)(current a)
int curF = 4;    // start at F5 (col index 4)(current f)
int curK = 9;    // start at K9(current K)
double S = 95.0; // current stress (0..100)
double heartbeat = 240;

// simulation clock & dense history sampling (no real waiting) ====
// keep my sanity: we move time ourselves so delayed reads have data to interpolate. what is time?
double sim_t = 0.0;      // simulated seconds since start
double SAMPLE_DT = 0.05; // record history every 50 ms for nice interpolation
double TAU = 10.0;       // heartbeat delay seconds (controller’s guess)

// heartbeat- this is where shit goes crazy
double hist_t[HIST_MAX]; // timestamps (seconds since start)
double hist_s[HIST_MAX]; // recorded S values
int hist_n = 0;          // number of samples stored

// Append a (time, S) sample
void record_stress_sample(double t_sec, double S_val)
{
    if (hist_n < HIST_MAX)
    {
        hist_t[hist_n] = t_sec;
        hist_s[hist_n] = S_val;
        hist_n++;
    }
    else
    {
        // simplest overwrite behavior: shift left (slow but easy)
        for (int i = 1; i < HIST_MAX; i++)
        {
            hist_t[i - 1] = hist_t[i];
            hist_s[i - 1] = hist_s[i];
        }
        hist_t[HIST_MAX - 1] = t_sec;
        hist_s[HIST_MAX - 1] = S_val;
    }
}

// drive the simulation time forward by dt and keep recording S while time passes
void advance_time(double dt)
{
    if (dt <= 0)
        return;
    double remain = dt;
    while (remain > 1e-9)
    {
        double step = (remain > SAMPLE_DT) ? SAMPLE_DT : remain;
        sim_t += step;
        record_stress_sample(sim_t, S);
        remain -= step;
    }
}

// tiny nudge to separate equal timestamps in logs when we instant-set S
void advance_epsilon(void)
{
    sim_t += 0.01; // 10 ms nudge
    record_stress_sample(sim_t, S);
}

// get "now"
double now_sec(void) { return sim_t; }

// Linear interpolation helper. This is the same as Linear approximation. Turns out that is actually really usefull irl.
double lerp(double x0, double y0, double x1, double y1, double x)
{
    if (fabs(x1 - x0) < 1e-12)
        return y0;
    double u = (x - x0) / (x1 - x0);
    return y0 + u * (y1 - y0);
}

// Return S(t - tau). If not enough history, fall back sensibly.
double stress_delayed(double now_sec_val, double tau_sec)
{
    if (hist_n == 0)
        return S;

    const double EPS = 1e-6;
    double target = now_sec_val - tau_sec;

    // clamp to history bounds
    if (target <= hist_t[0] + EPS)
        return hist_s[0];
    if (target >= hist_t[hist_n - 1] - EPS)
        return hist_s[hist_n - 1];

    // binary search for the first index with time >= target
    int lo = 0, hi = hist_n - 1;
    while (lo < hi)
    {
        int mid = (lo + hi) >> 1;
        if (hist_t[mid] < target)
            lo = mid + 1;
        else
            hi = mid;
    }
    // lo is the first index with t >= target
    int i1 = lo;     // t[i1] >= target
    int i0 = i1 - 1; // t[i0] <  target

    // if we basically hit an exact timestamp, return it
    if (fabs(hist_t[i1] - target) <= EPS)
        return hist_s[i1];
    if (fabs(hist_t[i0] - target) <= EPS)
        return hist_s[i0];

    // linear interpolate between the bracketing samples
    double x0 = hist_t[i0], y0 = hist_s[i0];
    double x1 = hist_t[i1], y1 = hist_s[i1];
    double u = (target - x0) / (x1 - x0);
    return y0 + u * (y1 - y0);
}

// MOTOR
//  Map an input percentage (0..100) to region index 1..5
int to_region(int percent)
{
    if (percent < 10)
        return 1; // [0,10)
    if (percent < 30)
        return 2; // [10,30)
    if (percent < 50)
        return 3; // [30,50)
    if (percent < 70)
        return 4; // [50,70)
    return 5;     // [70,100]
}

// Return the safe *midpoint* duty (%) for region 1..5
int region_mid_duty(int r)
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

// Stub: set PWM duty in percent on the hardware/channel
void set_pwm_percent(int channel, int percent)
{
    (void)channel;
    if (percent > 90)
    {
        printf("[EMERGENCY]");
        exit(1);
    }
    // never exceed 90% (emergency)
}

// forward decl (sim engine)
void move_to_cell(int newA, int newF);

// amp, freq are percentages (0-100).
// Any value inside an interval maps to that (A,F) cell.
// We "command" the cradle logically via move_to_cell(A-1, F-1).
// aIndex, fIndex are 0-4 (matrix indices)
void command_motor(int aIndex, int fIndex)
{
    // safety
    if (aIndex < 0 || aIndex > 4 || fIndex < 0 || fIndex > 4)
    {
        printf("[SYSTEM][ERROR] command_motor out-of-bounds A%d F%d\n",
               aIndex + 1, fIndex + 1);
        return;
    }

    // if you still want PWM mapping based on A/F levels:
    int dutyA = region_mid_duty(aIndex + 1); // 1..5
    int dutyF = region_mid_duty(fIndex + 1); // 1..5

    set_pwm_percent(AMP_CH, dutyA);
    set_pwm_percent(FREQ_CH, dutyF);

    move_to_cell(aIndex, fIndex);
}

// simulated crying based on current stress level
double get_crying()
{
    if (S <= 100 && S >= 50)
        return 100.0;
    else if (S <= 50 && S >= 10)
        return 2.5 * S - 25;
    else
        return 0;
}

double get_heartbeat(double stress_delayed_val)
{
    heartbeat = 60.0 + 1.8 * stress_delayed_val;
    return heartbeat;
}

// Force system into K9 panic and make outputs match Sopt[9]
void go_panic(const char *tag)
{
    // 1) set stress to K9's Sopt and record immediately
    S = Sopt[9];
    record_stress_sample(now_sec(), S);

    // 3) recompute outputs coherently

    heartbeat = get_heartbeat(S);

    int cry_now = (int)round(get_crying());

    // 4) log + tiny nudge to separate timestamps
    printf("[%s] PANIC -> S=%.1f, HB=%.0f, CRY=%d @t=%.2f\n",
           tag, S, heartbeat, cry_now, now_sec());

    advance_epsilon();
}

void generate_matrix()
{
    int a, f;

    srand(time(0)); // intialize the random seed

    // determine the first sopt for level K1 (idk if the sopt for k1 is always zero but well see)
    Sopt[1] = 10.0 + (rand() % 6); // 5 + a random value between 0-5

    // each next Sopt is increased by a small random step of  (7-14), capped at 98
    for (int k = 2; k <= 9; k++)
    {
        int step = 7.0 + (rand() % 5); // 7 + a random value between 0-5
        int next = (int)(Sopt[k - 1] + step);
        if (next > 98.0)
            next = 98.0;
        if (next <= Sopt[k - 1])
            next = (int)(Sopt[k - 1] + 1.0); // strictly increasing
        Sopt[k] = next;
    }
    // Sopt for K9 is Spanic (i think)

    // ideally every step has a range of 11ish. we are going to move based on that. This is just an assumption
    for (int k = 1; k <= 9; k++)
    {
        double half = 6.0 + (rand() % 7); // random value between 6-12
        BandLow[k] = Sopt[k] - half;
        BandHigh[k] = Sopt[k] + half;
        if (BandLow[k] < 0.0)
            BandLow[k] = 0.0;
        if (BandHigh[k] > 100.0)
            BandHigh[k] = 100.0;
    }

    // Guarantee: previous K's upper bound contains latter K's Sopt ---
    for (int k = 2; k <= 9; k++)
    {
        if (BandHigh[k - 1] < Sopt[k])
        {
            BandHigh[k - 1] = Sopt[k];
            if (BandHigh[k - 1] > 100.0)
                BandHigh[k - 1] = 100.0;
            if (BandLow[k - 1] > BandHigh[k - 1])
                BandLow[k - 1] = BandHigh[k - 1];
        }
    }

    // build the stress matrix(empty now)
    for (a = 0; a < 5; a++)
    {
        for (f = 0; f < 5; f++)
        {
            K[a][f] = 0;
        }
    }

    // endpoints
    K[0][0] = 1; // A1 F1
    K[4][4] = 9; // A5 F5

    /* RANDOM path from K9 K1 (LEFT/UP moves) */
    int adx = 4;  // start at A5 (row 4)
    int fdx = 4;  // start at F5 (col 4)
    int kcur = 9; // current K label at start

    int leftMoves = 4;
    int upMoves = 4;

    while (kcur > 1)
    {
        int dir; // 0 = LEFT, 1 = UP

        if (leftMoves == 0)
            dir = 1; // must go UP
        else if (upMoves == 0)
            dir = 0; // must go LEFT
        else
            dir = rand() % 2;

        if (dir == 0 && leftMoves > 0 && (fdx - 1) >= 0)
        {
            fdx = fdx - 1;
            leftMoves--;
        }
        else if (dir == 1 && upMoves > 0 && (adx - 1) >= 0)
        {
            adx = adx - 1;
            upMoves--;
        }

        kcur = kcur - 1;    // lower the k
        K[adx][fdx] = kcur; // place K on the path
    }

    /*
       fill the rest from bottom-right to top-left
    */
    for (a = 4; a >= 0; a--)
    {
        for (f = 4; f >= 0; f--)
        {
            if (K[a][f] == 0)
            {

                int m = 1;
                if (f + 1 < 5 && K[a][f + 1] > m)
                {
                    m = K[a][f + 1];
                    K[a][f] = m;
                }
            }
        }
    }

    for (a = 4; a >= 0; a--)
    {
        for (f = 4; f >= 0; f--)
        {
            if (K[a][f] == 0) // row-column
            {

                int m = 1;
                if (a + 1 < 5 && K[a + 1][f] > m)
                {
                    m = K[a + 1][f];
                    K[a][f] = m;
                }
            }
        }
    }

    // keep my sanity
    K[0][0] = 1;
    K[4][4] = 9;

    for (int k = 1; k <= 9; k++)
    {
        printf("K%d: Sopt=%5.1f  range=[%5.1f, %5.1f]\n",
               k, Sopt[k], BandLow[k], BandHigh[k]);
    }

    printf("\n");

    for (a = 0; a < 5; a++)
    {
        for (f = 0; f < 5; f++)
        {
            printf("K%d", K[a][f]);
            if (f < 4)
                printf(" ");
        }
        printf("\n");
    }
}

int in_range(int k, double v)
{
    if (k < 1 || k > 9)
        return 0;
    return (v >= BandLow[k] && v <= BandHigh[k]);
}

void print_status(const char *tag)
{
    printf("[%s] pos=A%d F%d  K%d  S=%.1f  (band %.1f-%.1f  Sopt=%.1f) @t=%.2f\n",
           tag, curA + 1, curF + 1, curK, S, BandLow[curK], BandHigh[curK], Sopt[curK], now_sec());
}

// Converge to Sopt of current K after ~2 seconds IF inside range
// we need to wait for convergence to sopt because we know sopt is guarenteed to be in the lower Ks range. or else we would cause a stress jump
void converge_now()
{
    advance_time(CONVERGENCE_TIME);
    S = Sopt[curK];
    record_stress_sample(now_sec(), S);
    print_status("[SYSTEM]converged");
}

/* called when you change cell. You MUST pass the K-label for that cell.
   newA/newF are 0..4 indexes (A1-A5 -> 0-4, F1-F5 -> 0-4). newK is 1-9.
*/
void move_to_cell(int newA, int newF)
{
    if (newA < 0 || newA > 4 || newF < 0 || newF > 4)
    {
        printf("[SYSTEM][ERROR] out-of-bounds move A%d F%d ignored.\n", newA + 1, newF + 1);
        return;
    }

    int oldA = curA;
    int oldF = curF;
    int oldK = curK;

    int targetK = K[newA][newF];

    printf("\n[SYSTEM] MOVE request: A%d F%d  K%d ---> A%d F%d  K%d \n",
           oldA + 1, oldF + 1, oldK, newA + 1, newF + 1, targetK);

    int softerA = (newA < oldA);
    int softerF = (newF < oldF);
    int harderA = (newA > oldA);
    int harderF = (newF > oldF);

    int is_soft = ((softerA || softerF) && !(harderA || harderF));
    int is_hard = ((harderA || harderF) && !(softerA || softerF));

    int overlap = 1;
    if (BandHigh[oldK] < BandLow[targetK] || BandLow[oldK] > BandHigh[targetK])
        overlap = 0;

    curA = newA;
    curF = newF;
    curK = targetK;

    if (in_range(curK, S))
    {
        printf("[SYSTEM] inside-band");
        converge_now();

        return;
    }

    if (!overlap)
    {
        if (is_soft)
        {
            go_panic("PANIC JUMP");
            return;
        }
        else if (is_hard)
        {
            go_panic("PANIC BLOCK");
            return;
        }
        else
        {
            if (S < BandLow[curK])
                S = BandLow[curK];
            if (S > BandHigh[curK])
                S = BandHigh[curK];
            record_stress_sample(now_sec(), S);
            print_status("[SYSTEM] mixed-move-converge");
            converge_now();

            return;
        }
    }

    if (S < BandLow[curK])
        S = BandLow[curK];
    if (S > BandHigh[curK])
        S = BandHigh[curK];
    record_stress_sample(now_sec(), S);
    printf("[SYSTEM][WARNING] overlap-converge. This is an unwanted message");
    converge_now();
}

// CONTROLLR LOGIC
// recuresive???

// evaluate heartbeat improvement thresholds
// controller state
// CONTROLLER STATE
int lastBPM = 0;
int thresholdBPM = 10;
int crying_started = 0; // keep for future, unused in simple rule

// remember where we came from (anchor cell)
int prevA = -1;
int prevF = -1;

// add at top-level
int anchorA_mem = -1, anchorF_mem = -1;
int triedLeftFromAnchor = 0;

// how we moved last step from (prevA,prevF) to current
// 0 = none/initial, 1 = left (F-1), 2 = up (A-1)
int lastMoveDir = 0;

// set starting state once, after you call generate_matrix(). keep my sanity.
void set_initial_state(int aIndex, int fIndex, int kLabel, double Sstart)
{
    curA = aIndex;
    curF = fIndex;
    curK = kLabel;
    S = Sstart;
    record_stress_sample(now_sec(), S);
    print_status("init");

    prevA = curA;
    prevF = curF;
    lastMoveDir = 0;
}

// Return 1 if BPM looks better than before, 0 otherwise.
// Also may flip crying_started if clearly worse.
// "Improved" = closer to calm:
// - either near rest (<= 60 + thresholdBPM)
// - or dropped by at least thresholdBPM vs lastBPM
// replace your signature or keep it the same and read globals
int heartbeat_improved(int bpm_now)
{
    // immediate improvement: lower K than where we came from
    if (curK < K[prevA][prevF])
        return 1;

    if (bpm_now <= 60 + thresholdBPM)
        return 1;
    if (lastBPM > 0 && bpm_now <= lastBPM - thresholdBPM)
        return 1;
    return 0;
}

void run_decision_once(void)
{
    // 1) Catch up to the LAST move.
    advance_time(TAU);

    // 2) Sense delayed stress -> BPM/CRY
    double S_tau = stress_delayed(now_sec(), TAU);
    int bpm_now = (int)round(get_heartbeat(S_tau));
    int cry_now = (int)round(get_crying());

    printf("[SENSE] S_tau=%.1f  BPM=%d  CRY=%d  pos=A%d F%d K%d @t=%.2f\n",
           S_tau, bpm_now, cry_now, curA + 1, curF + 1, curK, now_sec());

    // 3) Evaluate last move
    int improved = heartbeat_improved(bpm_now);

    // Ensure anchor memory is aligned with our current "home" cell when idle
    if (lastMoveDir == 0)
    {
        if (anchorA_mem != curA || anchorF_mem != curF)
        {
            anchorA_mem = curA;
            anchorF_mem = curF;
            triedLeftFromAnchor = 0; // new anchor => haven't tried LEFT here
        }
    }

    // 4) choose first trial from this anchor
    if (lastMoveDir == 0)
    {
        prevA = curA;
        prevF = curF;

        if (!triedLeftFromAnchor && curF > 0)
        {
            lastMoveDir = 1;         // LEFT
            triedLeftFromAnchor = 1; // remember we tried LEFT at this anchor
            printf("[ALGORITHM] initial/pick -> try LEFT from A%d F%d\n", curA + 1, curF + 1);
            command_motor(curA, curF - 1);
            lastBPM = bpm_now;
            return;
        }
        else if (curA > 0)
        {
            lastMoveDir = 2; // UP
            printf("[ALGORITHM] initial/pick -> try UP from A%d F%d (LEFT tried/blocked)\n", curA + 1, curF + 1);
            command_motor(curA - 1, curF);
            lastBPM = bpm_now;
            return;
        }
        else
        {
            // Nowhere softer to go
            printf("[ALGORITHM] at softest corner; waiting");
            lastBPM = bpm_now;
            return;
        }
    }

    // 5) We HAVE a last move. Determine anchor:
    //    - if improved -> current cell becomes new anchor (reset LEFT-tried flag)
    //    - if not     -> backtrack to previous (prevA, prevF) and keep LEFT-tried flag
    if (improved)
    {
        int anchorA = curA;
        int anchorF = curF;
        printf("[ALGORITHM] last move (dir=%d) IMPROVED -> new anchor at A%d F%d\n",
               lastMoveDir, anchorA + 1, anchorF + 1);

        // Refresh anchor memory and reset LEFT attempt flag
        if (anchorA_mem != anchorA || anchorF_mem != anchorF)
        {
            anchorA_mem = anchorA;
            anchorF_mem = anchorF;
            triedLeftFromAnchor = 0;
        }

        // From the new anchor, prefer LEFT; else UP
        prevA = anchorA;
        prevF = anchorF;

        if (anchorF > 0)
        {
            lastMoveDir = 1;         // LEFT
            triedLeftFromAnchor = 1; // about to try LEFT here
            printf("[ALGORITHM] improved -> next try LEFT from A%d F%d\n", anchorA + 1, anchorF + 1);
            command_motor(anchorA, anchorF - 1);
        }
        else if (anchorA > 0)
        {
            lastMoveDir = 2; // UP
            printf("[ALGORITHM] improved -> next try UP from A%d F%d\n", anchorA + 1, anchorF + 1);
            command_motor(anchorA - 1, anchorF);
        }
        

        lastBPM = bpm_now;
        return;
    }
    else
    {
        // No improvement.
        // If we just moved LEFT but K didn't change, switch direction to UP immediately.
        if (lastMoveDir == 1 && K[curA][curF] == K[prevA][prevF])
        {
            int anchorA = prevA;
            int anchorF = prevF;

            if (anchorA > 0)
            {
                // Try UP from the anchor without an extra backtrack cycle.
                printf("[ALGORITHM] left kept same K -> try UP from A%dF%d\n",
                       anchorA + 1, anchorF + 1);

                // Re-align to anchor logically
                curA = anchorA;
                curF = anchorF;

                lastMoveDir = 2; // UP
                command_motor(anchorA - 1, anchorF);
                lastBPM = bpm_now;
                return;
            }
            // If can't go UP, fall through to standard backtrack.
        }

        // Standard backtrack path (unchanged)
        int anchorA = prevA, anchorF = prevF;
        if (anchorA != curA || anchorF != curF)
        {
            printf("[ALGORITHM] last move (dir=%d) NO IMPROVEMENT -> backtrack to A%dF%d\n",
                   lastMoveDir, anchorA + 1, anchorF + 1);
            command_motor(anchorA, anchorF);
        }
        curA = anchorA;
        curF = anchorF;
        lastMoveDir = 0; // re-bootstrap next cycle
        lastBPM = bpm_now;
        return;
    }
}

void run_controller()
{
    // Drive the controller for some steps

    for (int step = 0; step < 40; ++step)
    {
        printf("\n[ALGORITHM] Controller Step %d \n", step + 1);
        run_decision_once();

        // stop if we’ve reached A1F1 and converged near K1
        if (curA == 0 && curF == 0 && curK == 1){
            printf("[ALGORITHM] rest reached");
            break;
        }

            
    }
}

int main(void)
{
    generate_matrix();
    heartbeat = get_heartbeat(Sopt[9]); // much needed on init. also reminds me
    lastBPM = heartbeat;
    // that we should wait for tau seconds at the start because if its a delayed value its not going to read anything
    // until tau seconds are actually passed

    // Start + record first sample (internal sim state)
    set_initial_state(4, 4, 9, Sopt[9]);

    run_controller();

    printf("\nfinished at t = %.3f s.\n", now_sec());
    return 0;
}
