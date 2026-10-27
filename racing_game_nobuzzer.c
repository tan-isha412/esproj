/*
 * LPC1768-Based Two-Lane Racing Game with Analog Joystick Steering
 * Board : ALS-SDA-ARMCTXM3-01 (NXP LPC1768, ARM Cortex-M3)
 * Tools : Keil uVision4, LPC17xx.h, Flash Magic
 *
 * Connections (synopsis, Table 1)
 *   16x2 LCD, 4-bit  : CNC -> CNAD, short JP16 and JP5, POT3 = contrast
 *                      P0.15-P0.18 = D4-D7, P0.19 = RS, P0.20 = EN
 *   7-segment        : CNA -> CNA2 (segments a-dp on P0.4-P0.11)
 *                      CNB -> CNB2 (BCD digit code on P1.23-P1.26, 74LS145)
 *   Joystick X       : CND pin 1 = P0.23 / AD0.0
 *   Joystick Y       : CND pin 2 = P0.24 / AD0.1
 *   Joystick button  : CND pin 7 = P2.0, active low, internal pull-up
 *   Joystick VCC/GND : board 3.3 V point / GND (CND pin 10)
 *
 * P1.31 (AD0.5) is hard-wired to POT1, so the LCD moved off CND to free
 * AD0.0-AD0.1 for the joystick. The speed level and shield state are shown
 * in the LCD's first column.
 *
 * Port 0 is written both by the Timer 0 ISR (7-segment) and by the main
 * loop (LCD). Every port write therefore goes through FIOSET/FIOCLR,
 * which only touch the bits given. Never write FIOPIN or FIOMASK here.
 */

#include <LPC17xx.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Settings to adjust on the real hardware                            */
/* ------------------------------------------------------------------ */

#define Y_UP_IS_LOW        1    /* 1: pushing up lowers the Y reading. Flip if steering is reversed  */
#define X_RIGHT_IS_HIGH    1    /* 1: pushing right raises the X reading. Flip if speed is reversed  */
#define JOY_DEAD_ZONE      700  /* ADC counts either side of centre (12-bit, centre about 0x800)     */

/* 74LS145 code for each display position, left to right. The lab manual's
 * counter drives the units digit (U8) with code 0, so U8 is the rightmost.
 * If digits come out reversed, change this to {0, 1, 2, 3}. */
static const uint8_t DIGIT_CODE[4] = {3, 2, 1, 0};

/* ------------------------------------------------------------------ */
/* Game parameters                                                    */
/* ------------------------------------------------------------------ */

#define LCD_COLS           16
#define HUD_COL            0     /* column 0: speed (top) and shield (bottom) */
#define ROAD_FIRST         1     /* road uses columns 1-15                    */
#define CAR_COL            2
#define START_LIVES        3
#define SPEED_MIN          1
#define SPEED_MAX          7
#define SPEED_REPEAT_MS    300   /* speed changes one level per 300 ms held   */
#define LEVEL_UP_SCORE     25    /* minimum speed rises every 25 points       */
#define MIN_GAP            3     /* columns between obstacles (2 empty cells) */
#define MAX_GAP            6     /* force an obstacle after this many columns */
#define SPAWN_PERCENT      45
#define SHIELD_STEPS       6     /* shield lasts 6 game steps                 */
#define SHIELD_COOLDOWN    25    /* then needs 25 steps to recharge           */
#define SCORE_MAX          999

#define INPUT_PERIOD_MS    10    /* joystick and button sampling period       */
#define DEBOUNCE_SAMPLES   3     /* 3 x 10 ms stable before accepting         */
#define LONG_PRESS_MS      800
#define COUNTDOWN_STEP_MS  700
#define CRASH_SHOW_MS      1200
#define CRASH_BLINK_MS     125

/* Game-step interval for speed levels 1..7 */
static const uint16_t STEP_MS[SPEED_MAX] = {450, 380, 320, 270, 220, 180, 140};

/* ------------------------------------------------------------------ */
/* Port lines                                                         */
/* ------------------------------------------------------------------ */

#define SEG_SHIFT    4
#define SEG_MASK     (0xFFu << SEG_SHIFT)      /* P0.4-P0.11  */
#define BTN_PIN      (1u << 0)                 /* P2.0        */
#define LCD_D_SHIFT  15
#define LCD_D_MASK   (0x0Fu << LCD_D_SHIFT)    /* P0.15-P0.18 */
#define LCD_RS       (1u << 19)                /* P0.19       */
#define LCD_EN       (1u << 20)                /* P0.20       */
#define DIG_SHIFT    23
#define DIG_MASK     (0x0Fu << DIG_SHIFT)      /* P1.23-P1.26 */

#define ADC_CH_X     0                         /* AD0.0, P0.23 */
#define ADC_CH_Y     1                         /* AD0.1, P0.24 */

/* ------------------------------------------------------------------ */
/* Display tables                                                     */
/* ------------------------------------------------------------------ */

/* Common cathode, bit 0 = a ... bit 6 = g, bit 7 = dp */
static const uint8_t SEG_DIGIT[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
#define SEG_DP     0x80
#define SEG_DASH   0x40

/* LCD custom characters (CGRAM 0-3) */
enum { CH_CAR = 0, CH_SHIELD = 1, CH_OBST = 2, CH_CRASH = 3 };
static const uint8_t CGRAM[4][8] = {
    {0x00, 0x00, 0x0C, 0x1F, 0x1F, 0x0A, 0x00, 0x00},   /* car                   */
    {0x1F, 0x1F, 0x13, 0x00, 0x00, 0x15, 0x1F, 0x1F},   /* shielded car (invert) */
    {0x00, 0x1F, 0x15, 0x0A, 0x15, 0x1F, 0x00, 0x00},   /* obstacle              */
    {0x00, 0x15, 0x0E, 0x1B, 0x0E, 0x15, 0x00, 0x00},   /* crash                 */
};

/* ------------------------------------------------------------------ */
/* State                                                              */
/* ------------------------------------------------------------------ */

typedef enum { ST_START, ST_COUNTDOWN, ST_PLAYING, ST_PAUSED, ST_COLLISION, ST_GAMEOVER } state_t;

static volatile uint32_t ms_ticks;            /* incremented every 1 ms by Timer 0 */
static volatile uint8_t  seg_buf[4];          /* segment patterns, left to right   */

static state_t  state;
static uint32_t t_state;                      /* time the current state started    */

static uint8_t  road[2][LCD_COLS];            /* 1 = obstacle; row 0 = upper lane  */
static uint8_t  lane;
static uint8_t  lives;
static uint8_t  speed, min_speed;
static uint16_t score, hi_score;              /* hi_score kept in RAM until reset  */
static uint8_t  new_hi;
static uint8_t  gap;                          /* columns since the last obstacle   */
static uint8_t  shield_steps, shield_cool;
static uint8_t  countdown;

static uint16_t x_centre = 0x800, y_centre = 0x800;
static int8_t   joy_x, joy_y;                 /* -1, 0 or +1; +1 = right / up      */
static uint8_t  btn_short, btn_long;          /* one-shot button events            */
static uint32_t rng_state = 1;

/* ------------------------------------------------------------------ */
/* Timers                                                             */
/* ------------------------------------------------------------------ */

/* Timer 0: 1 ms tick, refreshes one 7-segment digit per tick */
void TIMER0_IRQHandler(void)
{
    static uint8_t d = 0;

    LPC_TIM0->IR = 1;
    ms_ticks++;

    LPC_GPIO1->FIOSET = DIG_MASK;                         /* code 15: all digits off  */
    LPC_GPIO0->FIOCLR = SEG_MASK;
    LPC_GPIO0->FIOSET = (uint32_t)seg_buf[d] << SEG_SHIFT;
    LPC_GPIO1->FIOCLR = (uint32_t)(~DIGIT_CODE[d] & 0x0F) << DIG_SHIFT;  /* select digit */

    d = (d + 1) & 3;
}

static void timers_init(void)
{
    uint32_t pclk = SystemCoreClock / 4;                  /* PCLKSEL reset value: CCLK/4 */

    LPC_SC->PCONP |= (1u << 1) | (1u << 2);               /* power Timer 0 and Timer 1   */

    /* Timer 1: free-running 1 MHz counter for microsecond delays and the RNG seed */
    LPC_TIM1->TCR = 2;
    LPC_TIM1->PR  = pclk / 1000000 - 1;
    LPC_TIM1->TCR = 1;

    /* Timer 0: match interrupt every 1 ms */
    LPC_TIM0->TCR = 2;
    LPC_TIM0->PR  = 0;
    LPC_TIM0->MR0 = pclk / 1000 - 1;
    LPC_TIM0->MCR = 3;                                    /* interrupt and reset on MR0 */
    LPC_TIM0->IR  = 0x3F;
    NVIC_EnableIRQ(TIMER0_IRQn);
    LPC_TIM0->TCR = 1;
}

static void delay_us(uint32_t us)
{
    uint32_t start = LPC_TIM1->TC;
    while ((LPC_TIM1->TC - start) < us);
}

/* ------------------------------------------------------------------ */
/* GPIO, 7-segment                                                   */
/* ------------------------------------------------------------------ */

static void gpio_init(void)
{
    LPC_PINCON->PINSEL0 &= ~0xC0FFFF00u;                  /* P0.4-P0.11, P0.15 as GPIO */
    LPC_PINCON->PINSEL1 &= ~0x000003FFu;                  /* P0.16-P0.20 as GPIO       */
    LPC_PINCON->PINSEL3 &= ~0x003FC000u;                  /* P1.23-P1.26 as GPIO       */
    LPC_PINCON->PINSEL4 &= ~3u;                           /* P2.0 as GPIO              */
    LPC_PINCON->PINMODE4 &= ~3u;                          /* P2.0 pull-up              */

    LPC_GPIO0->FIODIR |= SEG_MASK | LCD_D_MASK | LCD_RS | LCD_EN;
    LPC_GPIO1->FIODIR |= DIG_MASK;
    LPC_GPIO2->FIODIR &= ~BTN_PIN;

    LPC_GPIO0->FIOCLR = SEG_MASK | LCD_D_MASK | LCD_RS | LCD_EN;
    LPC_GPIO1->FIOSET = DIG_MASK;                         /* all digits off */
}

/* First digit gets a raw pattern, the other three show n (000-999) */
static void seg_show(uint8_t first, uint16_t n)
{
    seg_buf[0] = first;
    seg_buf[1] = SEG_DIGIT[n / 100];
    seg_buf[2] = SEG_DIGIT[(n / 10) % 10];
    seg_buf[3] = SEG_DIGIT[n % 10];
}

/* Lives and score; the lives digit's decimal point means "shield ready" */
static void seg_game(void)
{
    uint8_t first = SEG_DIGIT[lives];

    if (state == ST_PLAYING && !shield_steps && !shield_cool)
        first |= SEG_DP;
    seg_show(first, score);
}

/* ------------------------------------------------------------------ */
/* LCD (4-bit, write only)                                            */
/* ------------------------------------------------------------------ */

static void lcd_nibble(uint8_t nib)
{
    LPC_GPIO0->FIOCLR = LCD_D_MASK;
    LPC_GPIO0->FIOSET = (uint32_t)(nib & 0x0F) << LCD_D_SHIFT;
    LPC_GPIO0->FIOSET = LCD_EN;
    delay_us(1);
    LPC_GPIO0->FIOCLR = LCD_EN;
    delay_us(1);
}

static void lcd_write(uint8_t b, uint8_t is_data)
{
    if (is_data)
        LPC_GPIO0->FIOSET = LCD_RS;
    else
        LPC_GPIO0->FIOCLR = LCD_RS;
    lcd_nibble(b >> 4);
    lcd_nibble(b & 0x0F);
    delay_us((!is_data && b <= 0x03) ? 2000 : 50);        /* clear/home need 1.64 ms */
}

#define lcd_cmd(c)   lcd_write((c), 0)
#define lcd_data(d)  lcd_write((d), 1)

static void lcd_init(void)
{
    uint8_t i, j;

    delay_us(20000);
    LPC_GPIO0->FIOCLR = LCD_RS;
    lcd_nibble(0x3); delay_us(5000);
    lcd_nibble(0x3); delay_us(200);
    lcd_nibble(0x3); delay_us(200);
    lcd_nibble(0x2); delay_us(200);                       /* now in 4-bit mode */

    lcd_cmd(0x28);                                        /* 4-bit, 2 lines, 5x8 font */
    lcd_cmd(0x0C);                                        /* display on, cursor off   */
    lcd_cmd(0x06);                                        /* increment, no shift      */
    lcd_cmd(0x01);                                        /* clear                    */

    lcd_cmd(0x40);                                        /* load custom characters   */
    for (i = 0; i < 4; i++)
        for (j = 0; j < 8; j++)
            lcd_data(CGRAM[i][j]);
    lcd_cmd(0x80);
}

static void lcd_goto(uint8_t row, uint8_t col)
{
    lcd_cmd((row ? 0xC0 : 0x80) | col);
}

/* Write a full row, padding with spaces */
static void lcd_line(uint8_t row, const char *s)
{
    uint8_t i;

    lcd_goto(row, 0);
    for (i = 0; i < LCD_COLS; i++)
        lcd_data(*s ? (uint8_t)*s++ : ' ');
}

static void put3(char *p, uint16_t n)
{
    p[0] = '0' + n / 100;
    p[1] = '0' + (n / 10) % 10;
    p[2] = '0' + n % 10;
}

/* ------------------------------------------------------------------ */
/* ADC and joystick                                                   */
/* ------------------------------------------------------------------ */

static void adc_init(void)
{
    LPC_SC->PCONP |= (1u << 12);                          /* power the ADC                */
    LPC_PINCON->PINSEL1 = (LPC_PINCON->PINSEL1 & ~0x0003C000u) | 0x00014000u;    /* P0.23 = AD0.0, P0.24 = AD0.1 */
    LPC_PINCON->PINMODE1 = (LPC_PINCON->PINMODE1 & ~0x0003C000u) | 0x00028000u;  /* no pull */
    LPC_ADC->ADCR = (1u << 21) | (4u << 8);               /* on, ADC clock = PCLK/5       */
}

static uint16_t adc_read(uint8_t ch)
{
    uint32_t r;

    LPC_ADC->ADCR = (1u << ch) | (4u << 8) | (1u << 21) | (1u << 24);   /* start now */
    do {
        r = LPC_ADC->ADGDR;
    } while (!(r & (1u << 31)));
    LPC_ADC->ADCR &= ~(7u << 24);
    return (r >> 4) & 0xFFF;
}

static uint16_t adc_avg(uint8_t ch)
{
    uint32_t sum = 0;
    uint8_t i;

    for (i = 0; i < 4; i++)
        sum += adc_read(ch);
    return sum / 4;
}

/* Joystick must be untouched while this runs */
static void joystick_calibrate(void)
{
    uint32_t sx = 0, sy = 0;
    uint8_t i;

    for (i = 0; i < 32; i++) {
        sx += adc_read(ADC_CH_X);
        sy += adc_read(ADC_CH_Y);
        delay_us(2000);
    }
    x_centre = sx / 32;
    y_centre = sy / 32;
    if (x_centre < 0x600 || x_centre > 0xA00) x_centre = 0x800;
    if (y_centre < 0x600 || y_centre > 0xA00) y_centre = 0x800;
}

static int8_t axis_dir(uint16_t v, uint16_t centre, uint8_t high_is_positive)
{
    int32_t d = (int32_t)v - (int32_t)centre;

    if (!high_is_positive)
        d = -d;
    if (d > JOY_DEAD_ZONE)
        return 1;
    if (d < -JOY_DEAD_ZONE)
        return -1;
    return 0;
}

static void joystick_read(void)
{
    joy_y = axis_dir(adc_avg(ADC_CH_Y), y_centre, !Y_UP_IS_LOW);    /* +1 = up    */
    joy_x = axis_dir(adc_avg(ADC_CH_X), x_centre, X_RIGHT_IS_HIGH); /* +1 = right */
}

/* ------------------------------------------------------------------ */
/* Button: debounce, short press on release, long press while held    */
/* ------------------------------------------------------------------ */

static void button_poll(uint32_t now)
{
    static uint8_t  stable = 0, count = 0, long_sent = 0;    /* stable: 1 = pressed */
    static uint32_t t_press;
    uint8_t raw = (LPC_GPIO2->FIOPIN & BTN_PIN) ? 0 : 1;

    if (raw != stable) {
        if (++count >= DEBOUNCE_SAMPLES) {
            stable = raw;
            count = 0;
            if (stable) {
                t_press = now;
                long_sent = 0;
            } else if (!long_sent) {
                btn_short = 1;
            }
        }
    } else {
        count = 0;
    }

    if (stable && !long_sent && (now - t_press) >= LONG_PRESS_MS) {
        btn_long = 1;
        long_sent = 1;
    }
}

/* ------------------------------------------------------------------ */
/* Game logic                                                         */
/* ------------------------------------------------------------------ */

static uint32_t rnd(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

static void new_game(void)
{
    uint8_t r, c;

    for (r = 0; r < 2; r++)
        for (c = 0; c < LCD_COLS; c++)
            road[r][c] = 0;
    lane = 1;
    lives = START_LIVES;
    score = 0;
    speed = min_speed = SPEED_MIN;
    gap = 0;
    shield_steps = shield_cool = 0;
    new_hi = 0;
    rng_state = (LPC_TIM1->TC ^ (ms_ticks << 16)) | 1u;   /* seeded by press timing */
}

static void add_score(uint8_t n)
{
    score += n;
    if (score > SCORE_MAX)
        score = SCORE_MAX;

    min_speed = SPEED_MIN + score / LEVEL_UP_SCORE;
    if (min_speed > SPEED_MAX)
        min_speed = SPEED_MAX;
    if (speed < min_speed)
        speed = min_speed;
}

/* Returns 1 if the car hit an obstacle without the shield */
static uint8_t check_collision(void)
{
    if (!road[lane][CAR_COL])
        return 0;
    road[lane][CAR_COL] = 0;
    if (shield_steps) {
        add_score(1);                                     /* shield smashes it */
        return 0;
    }
    return 1;
}

/* Move the road one column left; returns 1 on a crash */
static uint8_t game_step(void)
{
    uint8_t r, c, hit;

    for (r = 0; r < 2; r++) {
        for (c = ROAD_FIRST; c < LCD_COLS - 1; c++)
            road[r][c] = road[r][c + 1];
        road[r][LCD_COLS - 1] = 0;
    }

    /* Obstacles that just moved behind the car were passed */
    add_score(road[0][CAR_COL - 1] + road[1][CAR_COL - 1]);

    /* One obstacle per column at most, at least MIN_GAP apart: never blocked */
    if (++gap >= MIN_GAP && (gap >= MAX_GAP || rnd() % 100 < SPAWN_PERCENT)) {
        road[rnd() & 1][LCD_COLS - 1] = 1;
        gap = 0;
    }

    hit = check_collision();

    if (shield_steps)
        shield_steps--;
    else if (shield_cool)
        shield_cool--;
    return hit;
}

/* Give the player room after a crash */
static void clear_near_car(void)
{
    uint8_t c;

    for (c = ROAD_FIRST; c < ROAD_FIRST + 6; c++)
        road[0][c] = road[1][c] = 0;
}

/* ------------------------------------------------------------------ */
/* Screens                                                            */
/* ------------------------------------------------------------------ */

/* Column 0 shows the speed level (top) and shield state (bottom):
 * 'S' ready, '*' active, '.' recharging */
static void draw_road(uint8_t crashed)
{
    uint8_t r, c, ch;

    for (r = 0; r < 2; r++) {
        lcd_goto(r, 0);
        for (c = 0; c < LCD_COLS; c++) {
            if (c == HUD_COL)
                ch = r ? (shield_steps ? '*' : (shield_cool ? '.' : 'S')) : '0' + speed;
            else if (r == lane && c == CAR_COL)
                ch = crashed ? CH_CRASH : (shield_steps ? CH_SHIELD : CH_CAR);
            else if (road[r][c])
                ch = CH_OBST;
            else
                ch = ' ';
            lcd_data(ch);
        }
    }
}

static void show_start(void)
{
    char l1[] = "Press btn HI:000";

    put3(&l1[13], hi_score);
    lcd_line(0, " JOYSTICK RACER ");
    lcd_line(1, l1);
}

static void show_countdown(void)
{
    char l1[] = "                ";

    if (countdown) {
        l1[7] = '0' + countdown;
        lcd_line(1, l1);
    } else {
        lcd_line(1, "      GO!");
    }
    lcd_line(0, "   GET READY");
}

static void show_paused(void)
{
    lcd_line(0, "     PAUSED");
    lcd_line(1, "Hold btn: resume");
}

static void show_gameover(void)
{
    char l1[] = "SC:000    HI:000";

    put3(&l1[3], score);
    put3(&l1[13], hi_score);
    lcd_line(0, new_hi ? " NEW HIGH SCORE!" : "   GAME  OVER");
    lcd_line(1, l1);
}

/* ------------------------------------------------------------------ */
/* State machine                                                      */
/* ------------------------------------------------------------------ */

static void enter_state(state_t s)
{
    state = s;
    t_state = ms_ticks;
    btn_short = btn_long = 0;

    switch (s) {
    case ST_START:
        show_start();
        seg_show(SEG_DASH, hi_score);
        break;
    case ST_COUNTDOWN:
        countdown = 3;
        show_countdown();
        seg_game();
        break;
    case ST_PLAYING:
        draw_road(0);
        seg_game();
        break;
    case ST_PAUSED:
        show_paused();
        seg_game();
        break;
    case ST_COLLISION:
        draw_road(1);
        seg_game();
        break;
    case ST_GAMEOVER:
        if (score > hi_score) {
            hi_score = score;
            new_hi = 1;
        }
        show_gameover();
        seg_show(SEG_DIGIT[0], score);
        break;
    }
}

int main(void)
{
    uint32_t now, t_input = 0, t_step = 0, t_speed = 0;
    uint8_t redraw, moved, hit, crash_on, crash_shown = 1;

    SystemInit();
    SystemCoreClockUpdate();

    gpio_init();
    timers_init();
    adc_init();
    lcd_init();

    lcd_line(0, "  Calibrating");
    lcd_line(1, "Don't touch joy");
    joystick_calibrate();

    enter_state(ST_START);

    while (1) {
        now = ms_ticks;
        if (now - t_input < INPUT_PERIOD_MS)
            continue;
        t_input = now;

        joystick_read();
        button_poll(now);

        switch (state) {
        case ST_START:
        case ST_GAMEOVER:
            if (btn_short) {
                new_game();
                enter_state(ST_COUNTDOWN);
            }
            break;

        case ST_COUNTDOWN:
            if (now - t_state >= COUNTDOWN_STEP_MS) {
                t_state = now;
                if (countdown == 0) {
                    t_step = now;
                    enter_state(ST_PLAYING);
                } else {
                    countdown--;
                    show_countdown();
                }
            }
            break;

        case ST_PLAYING:
            if (btn_long) {
                enter_state(ST_PAUSED);
                break;
            }
            redraw = moved = hit = 0;

            if (btn_short && !shield_steps && !shield_cool) {
                shield_steps = SHIELD_STEPS;
                shield_cool = SHIELD_COOLDOWN;
                redraw = 1;
            }

            /* Steering: only a push past the dead zone changes lane */
            if (joy_y > 0 && lane != 0) {
                lane = 0;
                moved = 1;
            } else if (joy_y < 0 && lane != 1) {
                lane = 1;
                moved = 1;
            }
            if (moved) {
                hit = check_collision();
                redraw = 1;
            }

            /* Speed: holding left/right steps the level at a fixed rate */
            if (joy_x == 0) {
                t_speed = now - SPEED_REPEAT_MS;          /* next push acts at once */
            } else if (now - t_speed >= SPEED_REPEAT_MS) {
                t_speed = now;
                if (joy_x > 0 && speed < SPEED_MAX)
                    speed++;
                else if (joy_x < 0 && speed > min_speed)
                    speed--;
                redraw = 1;                               /* update the LCD speed digit */
            }

            if (!hit && now - t_step >= STEP_MS[speed - 1]) {
                t_step = now;
                hit = game_step();
                redraw = 1;
            }

            if (hit) {
                lives--;
                crash_shown = 1;
                enter_state(ST_COLLISION);
            } else if (redraw) {
                draw_road(0);
                seg_game();
            }
            break;

        case ST_PAUSED:
            if (btn_long) {
                t_step = now;
                enter_state(ST_PLAYING);
            }
            break;

        case ST_COLLISION:
            /* Blink the crash symbol */
            crash_on = (((now - t_state) / CRASH_BLINK_MS) & 1) ? 0 : 1;
            if (crash_on != crash_shown) {
                crash_shown = crash_on;
                lcd_goto(lane, CAR_COL);
                lcd_data(crash_on ? CH_CRASH : ' ');
            }
            if (now - t_state >= CRASH_SHOW_MS) {
                if (lives == 0) {
                    enter_state(ST_GAMEOVER);
                } else {
                    clear_near_car();
                    t_step = now;
                    enter_state(ST_PLAYING);
                }
            }
            break;
        }

        btn_short = btn_long = 0;                         /* events last one pass */
    }
}
