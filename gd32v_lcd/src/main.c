#include "lcd/lcd.h"
#include <string.h>

void init_uart0(void)
{
	/* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

	/* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

    usart_interrupt_enable(USART0, USART_INT_RBNE);
}


// Code below written by Marijn Stollenga:

void draw_mandel() {
  int const width = 160;
  int const height = 80;
  LCD_Address_Set(0, 0, width - 1, height - 1);      //设置光标位置

  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x)
    {
      float ca = 0.002 * (x - 80) / 80. - 0.7463;
      float cb = 0.002 * (y - 40) / 80. + 0.1102;
      float a = ca;
      float b = cb;
      int n = 0;
      const int factor = 16;
      for (; n < 65536 / factor; ++n) {
        float ta = a * a - b * b;
        if (ta > 2.0)
          break;
        b = cb + 2 * a * b;
        a = ca + ta;

      }
      LCD_WR_DATA(n * 3);
    }
}

uint16_t get_color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint16_t)(r >> 3) << 11) + ((uint16_t)(g >> 2) << 5) + ((uint16_t)(b >> 3));
}

void draw_angle() {
  uint16_t color = 0;
  int const width = 160;
  int const height = 80;

  while (1) {
    LCD_Address_Set(0, 0, width - 1, height - 1);      //设置光标位置
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x)
        LCD_WR_DATA(get_color(y * 256 / height, color, x * 256 / width));
    // delay_1ms(1);
  }
}

void draw_1d_ca(int rule) {
  int const width = 160;
  int const height = 80;
  uint8_t life[width * height];
  uint8_t life_line[width];
  uint8_t life_line_next[width];
  uint8_t rules[8];

  memset(life, 0, width * height);
  memset(life_line, 0, width);
  memset(life_line_next, 0, width);
  memset(rules, 0, 8);
  life_line[width / 2] = 1;
  //read the rules
  for (int n = 0; n < 8; ++n)
  {
    rules[n] = rule & 1;
    rule >>= 1;
  }

  while (1) {
    for (int n = 0; n < width; ++n)
    {
      uint8_t current_value = 0;

      if (n == 0)
        current_value = (life_line[width-1] << 2) + (life_line[0] << 1) + life_line[1];
      else if (n == width - 1)
        current_value = (life_line[width - 2] << 2) + (life_line[width - 1] << 1) + life_line[0];
      else
        current_value = (life_line[n-1] << 2) + (life_line[n] << 1) + life_line[n + 1];

      life_line_next[n] = rules[current_value];
    }

    memcpy(life, life + width, (height - 1) * width);
    memcpy(life + (height - 1) * width, life_line_next, width);
    memcpy(life_line, life_line_next, width);

    LCD_Address_Set(0, 0, width - 1, height - 1);
    for (int n = 0; n < width * height; ++n)
      LCD_WR_DATA(life[n] ? BLUE : BLACK);
  }
}


void draw_conway_life() {
  int const width = 160;
  int const height = 80;
  uint8_t life[width * height];
  uint8_t life_next[width * height];
  memset(life, 0, width * height);
  memset(life_next, 0, width * height);
  srand(123413);
  for (int n = 0; n < width * height; ++n)
    life[n] = (rand() % 7) == 1;

  while (1) {
    int idx = 0;
    LCD_Address_Set(0, 0, width - 1, height - 1);      //设置光标位置
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x, ++idx) {
        int top = (y == 0) ? height * width - width : -width;
        int bottom = (y == (height - 1)) ? width - height * width : width;
        int left = (x == 0) ? width - 1 : -1;
        int right = (x == (width - 1)) ? 1 - width : 1;
        int n_neighbours = life[idx + top] +
        life[idx + bottom] +
          life[idx + right] +
          life[idx + left] +
          life[idx + top + right] +
          life[idx + top + left] +
          life[idx + bottom + right] +
          life[idx + bottom + left];
        if (life[idx] && (n_neighbours == 2 || n_neighbours == 3)) {
          life_next[idx] = 1;
          LCD_WR_DATA(RED);
          continue;
        }
        if (!life[idx] && n_neighbours == 3) {
          life_next[idx] = 1;
          LCD_WR_DATA(WHITE);
          continue;
        }
        // if (n_neighbours == 1) {
        //   life_next[idx] = 1;
        //   LCD_WR_DATA(RED);
        //   continue;
        // }
        life_next[idx] = 0;
        LCD_WR_DATA(BLACK);
      }

    idx = 0;
    memcpy(life, life_next, width * height);
  	// for (int i = 0; i < width * height; i++)
  		// LCD_WR_DATA(life[i] ? RED : BLACK);
    // delay_1ms(40);
  }


}

int main(void)
{

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1|GPIO_PIN_2);

    init_uart0();

    Lcd_Init();			// init OLED
    LCD_Clear(BLACK);

    // draw_angle();
    // draw_mandel();
    // draw_conway_life();
    // draw_1d_ca(110);
    draw_1d_ca(150);
}

int _put_char(int ch)
{
    usart_data_transmit(USART0, (uint8_t) ch );
    while ( usart_flag_get(USART0, USART_FLAG_TBE)== RESET){
    }

    return ch;
}
