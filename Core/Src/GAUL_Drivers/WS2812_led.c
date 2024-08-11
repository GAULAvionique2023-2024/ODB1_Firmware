/*
 * Copyright (c) 2020, evilwombat
 *
 * Based on principles described by Martin Hubáček in:
 *  http://www.martinhubacek.cz/arm/improved-stm32-ws2812b-library
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include <GAUL_Drivers/WS2812_led.h>
#include <string.h>

DMA_HandleTypeDef hdma_tim2_update;
DMA_HandleTypeDef hdma_tim2_pwm_ch1;
DMA_HandleTypeDef hdma_tim2_pwm_ch2;

/* Generally you should not need to mess with these */
#define DMA_BUFFER_SIZE         16
#define DMA_BUFFER_FILL_SIZE    ((DMA_BUFFER_SIZE) / 2)

static uint16_t ws2812_gpio_set_bits = 0;
static uint16_t dma_buffer[DMA_BUFFER_SIZE];

static void ws2812_timer2_init(void) {
    // Activer l'horloge pour TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configurer le Prescaler et la Période
    TIM2->PSC = 0;                       // Pas de prescaler
    TIM2->ARR = WS2812_TIMER_PERIOD;     // Période

    // Configurer le Mode de Compteur
    TIM2->CR1 &= ~TIM_CR1_DIR;    // Compteur ascendant
    TIM2->CR1 &= ~TIM_CR1_CMS;    // Mode de compteur aligné sur le bord
    TIM2->CR1 |= TIM_CR1_ARPE;    // Auto-reload preload enable

    // Configurer le Mode PWM pour le canal 1
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;        // Clear output compare mode bits for channel 1
    TIM2->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1 (OC1M bits = 110)
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1PE;       // Disable preload (OC1PE bit for channel 1)
    TIM2->CCER |= TIM_CCER_CC1E;           // Enable output for channel 1
    TIM2->CCR1 = WS2812_TIMER_PWM_CH1_TIME; // Pulse width for channel 1

    // Configurer le Mode PWM pour le canal 2
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;        // Clear output compare mode bits for channel 2
    TIM2->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos); // PWM mode 1 (OC2M bits = 110)
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2PE;       // Disable preload (OC2PE bit for channel 2)
    TIM2->CCER |= TIM_CCER_CC2E;           // Enable output for channel 2
    TIM2->CCR2 = WS2812_TIMER_PWM_CH2_TIME; // Pulse width for channel 2

    // Configurer la Source d'Horloge (interne)
    TIM2->SMCR &= ~TIM_SMCR_SMS;           // Disable slave mode
    TIM2->CR2 &= ~TIM_CR2_MMS;             // Master mode selection - reset

    // Démarrer le Timer
    TIM2->CR1 |= TIM_CR1_CEN;  // Activer le timer TIM2
}

static void ws2812_dma_start(GPIO_TypeDef *gpio_bank) {
    // Activer l'horloge du DMA1
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Configuration de DMA1_Channel2 pour TIM2 Update (TIM2_UP)
    DMA1_Channel2->CCR &= ~DMA_CCR_EN; // Désactiver le canal avant configuration
    DMA1_Channel2->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_0 | DMA_CCR_CIRC | DMA_CCR_PL_1;
    DMA1_Channel2->CNDTR = DMA_BUFFER_SIZE;
    DMA1_Channel2->CPAR = (uint32_t)&gpio_bank->BSRR;
    DMA1_Channel2->CMAR = (uint32_t)&ws2812_gpio_set_bits;

    // Configuration de DMA1_Channel5 pour TIM2_CH1 PWM
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_0 | DMA_CCR_CIRC | DMA_CCR_PL_1;
    DMA1_Channel5->CNDTR = DMA_BUFFER_SIZE;
    DMA1_Channel5->CPAR = (uint32_t)&gpio_bank->BRR;
    DMA1_Channel5->CMAR = (uint32_t)dma_buffer;

    // Configuration de DMA1_Channel7 pour TIM2_CH2 PWM
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;
    DMA1_Channel7->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_0 | DMA_CCR_CIRC | DMA_CCR_PL_1;
    DMA1_Channel7->CNDTR = DMA_BUFFER_SIZE;
    DMA1_Channel7->CPAR = (uint32_t)&gpio_bank->BRR;
    DMA1_Channel7->CMAR = (uint32_t)&ws2812_gpio_set_bits;

    // Démarrer les DMA
    DMA1_Channel2->CCR |= DMA_CCR_EN;
    DMA1_Channel5->CCR |= DMA_CCR_EN;
    DMA1_Channel7->CCR |= DMA_CCR_EN;

    // Activer les DMA dans le TIM2
    TIM2->DIER |= TIM_DIER_UDE;   // DMA request enabled for update event
    TIM2->DIER |= TIM_DIER_CC1DE; // DMA request enabled for capture/compare 1 event
    TIM2->DIER |= TIM_DIER_CC2DE; // DMA request enabled for capture/compare 2 event

    // Démarrer le timer TIM2
    TIM2->CR1 |= TIM_CR1_CEN;
}

/*
 * Unpack the bits of ch_val and pack them into the bit positions of cur0-cur7 that correspond to
 * the given GPIO number. Later, cur0-cur7 will be DMAed directly to a register within our GPIO
 * bank.
 */
#define UNPACK_CHANNEL(gpio_num)                    \
    asm volatile (                                  \
    "ubfx   r0, %[ch_val], #7, #1 \n"               \
    "bfi    %[cur0], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    "ubfx   r0, %[ch_val], #6, #1 \n"               \
    "bfi    %[cur1], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    "ubfx   r0, %[ch_val], #5, #1 \n"               \
    "bfi    %[cur2], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    "ubfx   r0, %[ch_val], #4, #1 \n"               \
    "bfi    %[cur3], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    "ubfx   r0, %[ch_val], #3, #1 \n"               \
    "bfi    %[cur4], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    "ubfx   r0, %[ch_val], #2, #1 \n"               \
    "bfi    %[cur5], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    "ubfx   r0, %[ch_val], #1, #1 \n"               \
    "bfi    %[cur6], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    "ubfx   r0, %[ch_val], #0, #1 \n"               \
    "bfi    %[cur7], r0,   #" #gpio_num ", 1  \n"   \
                                                    \
    : [cur0]"+r" (cur0),                            \
        [cur1]"+r" (cur1),                          \
        [cur2]"+r" (cur2),                          \
        [cur3]"+r" (cur3),                          \
        [cur4]"+r" (cur4),                          \
        [cur5]"+r" (cur5),                          \
        [cur6]"+r" (cur6),                          \
        [cur7]"+r" (cur7)                           \
    : [ch_val]"r" (ch_val)                          \
    : "r0", "cc");  /* r0 is a temp variable */

/*
 * Unpack the bits for one byte of one channel, and pack them into the bit positions of
 * the cur0 - cur7 variables, corresponding to the GPIO number for that channel.
 * The 'if' statement will be optimized away by the compiler, depending on how many channels
 * are actually defined.
 *
 */
#define HANDLE_CHANNEL(ch_num, gpio_num)                    \
    if (ch_num < WS2812_NUM_CHANNELS) {                     \
        ch_val = channels->framebuffer[pos] ^ 0xff;  \
        UNPACK_CHANNEL(gpio_num);                           \
    }

static void fill_dma_buffer(uint16_t *dest, int pos, const struct led_channel_info *channels) {
    register uint16_t cur0 = 0, cur1 = 0, cur2 = 0, cur3 = 0, cur4 = 0, cur5 = 0, cur6 = 0, cur7 = 0;

    uint8_t ch_val;
    HANDLE_CHANNEL(0, WS2812_CH0_GPIO);

    dest[0] = cur0;
    dest[1] = cur1;
    dest[2] = cur2;
    dest[3] = cur3;
    dest[4] = cur4;
    dest[5] = cur5;
    dest[6] = cur6;
    dest[7] = cur7;
}

void WS2812_Refresh(const struct led_channel_info *channels) {
    int cycles = 0;
    int i;
    int pos = 0;
    int max_length = 0;

    /* Initialiser les bits GPIO à envoyer via le DMA */
    ws2812_gpio_set_bits = 0;

    /* Pré-remplir le buffer DMA */
    for (i = 0; i < DMA_BUFFER_SIZE; i += 8) {
        fill_dma_buffer(dma_buffer + i, pos, channels);
        pos++;
    }

    max_length = 72 + (DMA_BUFFER_SIZE / 8);
    ws2812_gpio_set_bits |= (1 << WS2812_CH0_GPIO);

    /* Désactiver les DMA du timer TIM2 */
    TIM2->DIER &= ~(TIM_DIER_UDE | TIM_DIER_CC1DE | TIM_DIER_CC2DE);

    /* Désactiver le timer TIM2 */
    TIM2->CR1 &= ~TIM_CR1_CEN;

    /* Mettre les GPIO à 0 pour générer l'impulsion de réinitialisation */
    GPIOB->BRR = ws2812_gpio_set_bits;

    /* Activer le timer TIM2 */
    TIM2->CR1 |= TIM_CR1_CEN;

    /* Attendre que le timer génère l'impulsion de réinitialisation */
    for (i = 0; i < 225; i++) {
        while (!(TIM2->SR & TIM_SR_UIF)) ; // Attendre la mise à jour
        TIM2->SR &= ~TIM_SR_UIF; // Effacer le flag d'update
    }

    /* Désactiver le timer TIM2 */
    TIM2->CR1 &= ~TIM_CR1_CEN;

    /* Démarrer le DMA pour la transmission des données */
    ws2812_dma_start(GPIOB);

    /* Positionner le compteur du timer juste avant le débordement */
    TIM2->CNT = TIM2->ARR - 10;

    /* Effacer les flags d'événements du DMA */
    DMA1->IFCR = (DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5);

    /* Réactiver le timer TIM2 */
    TIM2->CR1 |= TIM_CR1_CEN;

    while (1) {
        /* Attendre un événement de transfert complet ou de demi-transfert du DMA */
        if (!(DMA1->ISR & (DMA_ISR_TCIF5 | DMA_ISR_HTIF5))) {
            cycles++;
            continue;
        }

        uint16_t *dest = dma_buffer;

        /* Vérifier si nous remplissons la première ou la seconde moitié du buffer DMA */
        if (DMA1->ISR & DMA_ISR_TCIF5)
            dest += DMA_BUFFER_FILL_SIZE;

        /* Effacer les flags d'événements DMA */
        DMA1->IFCR = (DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5);

        /* Décompresser un nouveau byte de chaque canal dans le buffer DMA */
        for (i = 0; i < DMA_BUFFER_FILL_SIZE; i += 8) {
            fill_dma_buffer(dest + i, pos, channels);
            pos++;
        }

        if (pos > max_length)
            break;
    }

    /* Désactiver le timer TIM2 après la fin de la transmission */
    TIM2->CR1 &= ~TIM_CR1_CEN;

    /* Remettre les GPIOs à 0 */
    GPIOB->BRR = ws2812_gpio_set_bits;

    /* Désactiver les canaux DMA */
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;
}

void WS2812_Init() {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init, not that we're using it... */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);

    ws2812_timer2_init();
}

void WS2812_Solid_Colors(struct pixel *framebuffer, const struct led_channel_info *channels, uint8_t r, uint8_t g, uint8_t b) {
    int red_offset, green_offset, blue_offset, i;

    red_offset = 0;
    green_offset = (FRAMEBUFFER_SIZE / 4) * ((0) & 0x03);
    blue_offset = (FRAMEBUFFER_SIZE / 4) * ((0 >> 2) & 0x03);

    /* Just generate a different-looking psychedelic-looking pattern for each channel, to
     * test/prove that each channel is receiving a unique pattern
     */
    for (i = 0; i < FRAMEBUFFER_SIZE / 2; i++) {
        framebuffer[(i + red_offset + r) % FRAMEBUFFER_SIZE].r = i;
        framebuffer[(i + green_offset + g) % FRAMEBUFFER_SIZE].g = i;
        framebuffer[(i + blue_offset + b) % FRAMEBUFFER_SIZE].b = i;
    }

    __disable_irq();
    WS2812_Refresh(channels);
    __enable_irq();
}
