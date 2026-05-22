#include <stdint.h>

/* ================= RCC ================= */
#define RCC_CR        (*((volatile uint32_t *)0x40021000))
#define RCC_CFGR      (*((volatile uint32_t *)0x40021004))
#define RCC_APB2ENR   (*((volatile uint32_t *)0x40021018))
#define RCC_APB1ENR   (*((volatile uint32_t *)0x4002101C))

/* ================= FLASH ================= */
#define FLASH_ACR     (*((volatile uint32_t *)0x40022000))

/* ================= GPIO ================= */
#define GPIOB_CRL     (*((volatile uint32_t *)0x40010C00))
#define GPIOC_CRH     (*((volatile uint32_t *)0x40011004))
#define GPIOC_ODR     (*((volatile uint32_t *)0x4001100C))

/* ================= I2C1 (base 0x40005400) ================= */
#define I2C1_CR1      (*((volatile uint32_t *)0x40005400))
#define I2C1_CR2      (*((volatile uint32_t *)0x40005404))
#define I2C1_DR       (*((volatile uint32_t *)0x40005410))
#define I2C1_SR1      (*((volatile uint32_t *)0x40005414))
#define I2C1_SR2      (*((volatile uint32_t *)0x40005418))
#define I2C1_CCR      (*((volatile uint32_t *)0x4000541C))
#define I2C1_TRISE    (*((volatile uint32_t *)0x40005420))

/* ================= SysTick ================= */
#define SYSTICK_CTRL  (*((volatile uint32_t *)0xE000E010))
#define SYSTICK_LOAD  (*((volatile uint32_t *)0xE000E014))
#define SYSTICK_VAL   (*((volatile uint32_t *)0xE000E018))

/* ========================================================= */

static void SystemClock_Config(void) {
  /* Enable HSE (8MHz crystal) */
  RCC_CR |= (1 << 16);
  while (!(RCC_CR & (1 << 17)));

  /* Flash: 2 wait states for 72 MHz */
  FLASH_ACR = (FLASH_ACR & ~0x7) | 0x2;

  /* PLL: HSE x 9 => 72 MHz */
  RCC_CFGR &= ~((0xF << 18) | (1 << 16));
  RCC_CFGR |= (1 << 16) | (0x7 << 18);

  /* APB1 = HCLK/2 = 36 MHz */
  RCC_CFGR &= ~(0x7 << 8);
  RCC_CFGR |= (0x4 << 8);

  /* Start PLL */
  RCC_CR |= (1 << 24);
  while (!(RCC_CR & (1 << 25)));

  /* Switch system clock to PLL */
  RCC_CFGR = (RCC_CFGR & ~0x3) | 0x2;
  while ((RCC_CFGR & 0xC) != 0x8);
}

static void delay_ms(uint32_t ms) {
  /* 72 MHz => 72000 ticks per ms */
  SYSTICK_LOAD = 72000 - 1;
  SYSTICK_VAL = 0;
  SYSTICK_CTRL = 5; 
  while (ms--) {
    while (!(SYSTICK_CTRL & (1 << 16)));
  }
  SYSTICK_CTRL = 0;
}

/* ================== I2C Driver ================== */

static void I2C_Init(void) {
  /* 1. Enable Clocks for I2C1, GPIOB, and AFIO */
  RCC_APB1ENR |= (1 << 21); /* I2C1 */
  RCC_APB2ENR |= (1 << 3);  /* GPIOB */
  RCC_APB2ENR |= (1 << 0);  /* AFIO */

  /* 2. Configure PB6 (SCL) and PB7 (SDA) as Alternate Function Open Drain (50MHz) */
  /* GPIOB_CRL bits 24-31. AF OD 50MHz = 0xFF */
  GPIOB_CRL = (GPIOB_CRL & 0x00FFFFFF) | 0xFF000000;

  /* 3. Reset I2C1 */
  I2C1_CR1 |= (1 << 15);
  I2C1_CR1 &= ~(1 << 15);

  /* 4. Set Peripheral clock frequency (APB1 is 36 MHz) */
  I2C1_CR2 = 36;

  /* 5. Configure clock control (Standard mode 100kHz) -> CCR = 36MHz / (2 * 100kHz) */
  I2C1_CCR = 180;

  /* 6. Configure Maximum Rise Time -> TRISE = (1000ns / (1/36MHz)) + 1 = 37 */
  I2C1_TRISE = 37;

  /* 7. Enable I2C1 */
  I2C1_CR1 |= (1 << 0);
}

/* Note: Will hang infinitely if device is disconnected */
static void I2C_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t data) {
  /* Generate START */
  I2C1_CR1 |= (1 << 8);
  while (!(I2C1_SR1 & (1 << 0))); 

  /* Send Device Address (Write) */
  I2C1_DR = dev_addr;
  while (!(I2C1_SR1 & (1 << 1))); 
  (void)I2C1_SR1; /* Read SR1, SR2 to clear ADDR flag */
  (void)I2C1_SR2;

  /* Send Register Address */
  while (!(I2C1_SR1 & (1 << 7))); 
  I2C1_DR = reg;

  /* Send Data */
  while (!(I2C1_SR1 & (1 << 7))); 
  I2C1_DR = data;

  /* Wait for BTF (Byte Transfer Finished) */
  while (!(I2C1_SR1 & (1 << 2)));

  /* Generate STOP */
  I2C1_CR1 |= (1 << 9);
}

/* ================= PCA9685 Driver ================= */

#define PCA9685_ADDR      0x80  /* 0x40 shifted left 1 */
#define PCA9685_MODE1     0x00
#define PCA9685_PRESCALE  0xFE

static void PCA9685_Init(void) {
  /* Put into sleep mode to set the prescaler bit */
  I2C_WriteReg(PCA9685_ADDR, PCA9685_MODE1, 0x10); 
  delay_ms(5);
  
  /* Set prescaler for 50Hz (121 based on 25MHz internal oscillator) */
  I2C_WriteReg(PCA9685_ADDR, PCA9685_PRESCALE, 121); 

  /* Wake up and enable Auto-Increment */
  I2C_WriteReg(PCA9685_ADDR, PCA9685_MODE1, 0xA1); 
  delay_ms(5);
}

static void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off) {
  uint8_t start_reg = 0x06 + (4 * channel);
  I2C_WriteReg(PCA9685_ADDR, start_reg,     on & 0xFF);
  I2C_WriteReg(PCA9685_ADDR, start_reg + 1, on >> 8);
  I2C_WriteReg(PCA9685_ADDR, start_reg + 2, off & 0xFF);
  I2C_WriteReg(PCA9685_ADDR, start_reg + 3, off >> 8);
}

static void PCA9685_SetAngle(uint8_t channel, uint16_t angle) {
  if (angle > 180) angle = 180;
  
  /* PCA9685 has 4096 steps per 20ms period (50Hz) 
     1 step = 4.88us
     500us = 102
     2500us = 512 
  */
  uint16_t pwm_val = 102 + ((uint32_t)angle * (512 - 102) / 180);
  PCA9685_SetPWM(channel, 0, pwm_val);
}

/* ========================================================= */

int main(void) {
  SystemClock_Config();

  /* Enable PC13 LED output */
  RCC_APB2ENR |= (1 << 4);
  GPIOC_CRH &= ~((uint32_t)0xF << 20);
  GPIOC_CRH |= ((uint32_t)0x2 << 20);

  /* Initialize I2C and hardware */
  I2C_Init();
  PCA9685_Init();

  /* Stretch to center */
  for (int ch = 0; ch < 4; ch++) {
    PCA9685_SetAngle(ch, 90);
  }
  delay_ms(2000);

  #define STEP_DELAY_MS 15 
  int16_t angle = 90; 
  int8_t dir = -1;    

  /* Sweeping 4 channels! */
  while (1) {
    for (int ch = 0; ch < 4; ch++) {
      PCA9685_SetAngle(ch, (uint16_t)angle);
    }
    delay_ms(STEP_DELAY_MS);

    angle += dir;

    if (angle <= 0) {
      angle = 0;
      dir = 1;
      GPIOC_ODR ^= (1 << 13);
    } else if (angle >= 180) {
      angle = 180;
      dir = -1;
      GPIOC_ODR ^= (1 << 13);
    }
  }
}

unsigned int *const stack_top = (unsigned int *)0x20005000;
void Reset_Handler(void) {
  main();
  while (1);
}
__attribute__((section(".isr_vector"))) void (*const g_pfnVectors[])(void) = {
    (void (*)(void))((unsigned long)stack_top), Reset_Handler};