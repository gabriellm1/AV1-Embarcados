/*
 * main.c
 *
 * Created: 05/03/2019 18:00:58
 *  Author: eduardo
 */ 

#include <asf.h>
#include "tfont.h"
#include "sourcecodepro_28.h"
#include "calibri_36.h"
#include "arial_72.h"

// Bot?o Placa
#define BUT_PIO      PIOA
#define BUT_PIO_ID   ID_PIOA
#define BUT_IDX  11
#define BUT_IDX_MASK (1 << BUT_IDX)

// defines rtc
#define YEAR        2019
#define MOUNTH      4
#define DAY         8
#define WEEK        12
#define HOUR        0
#define MINUTE      0
#define SECOND      0

uint32_t hora, minuto, seg;

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);

struct ili9488_opt_t g_ili9488_display_opt;

volatile Bool f_rtt_alarme = false;

void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	
}


void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}	
}

volatile int but_flag;
void but_flag_callback(void){
	but_flag += 1;
}

void velocidade(){}

volatile Bool atualiza_temp;
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	uint32_t hour, minute, second;
	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		rtc_clear_status(RTC, RTC_SCCR_ALRCLR);

		if(atualiza_temp == 1){
			atualiza_temp = 0;
			rtc_get_time(RTC, &hour, &minute, &second);
			rtc_set_time_alarm(RTC, 1, hour, 1, minute, 1, second+1);

			
		}
		else if(atualiza_temp == 0){
			atualiza_temp = 1;
			rtc_get_time(RTC, &hour, &minute, &second);
			rtc_set_time_alarm(RTC, 1, hour, 1, minute, 1, second+1);


		}
		
			
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}

void rtc_init(){
		/* Configura o PMC */
		pmc_enable_periph_clk(ID_RTC);

		/* Default RTC configuration, 24-hour mode */
		rtc_set_hour_mode(RTC, 0);

		/* Configura data e hora manualmente */
		rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
		rtc_set_time(RTC, HOUR, MINUTE, SECOND);

		/* Configure RTC interrupts */
		NVIC_DisableIRQ(RTC_IRQn);
		NVIC_ClearPendingIRQ(RTC_IRQn);
		NVIC_SetPriority(RTC_IRQn, 0);
		NVIC_EnableIRQ(RTC_IRQn);

		/* Ativa interrupcao via alarme */
		rtc_enable_interrupt(RTC,  RTC_IER_ALREN);
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		//pin_toggle(LED_PIO, LED_IDX_MASK);    // AQUI
		f_rtt_alarme = true;                  // flag RTT alarme
	}
}

void rtt_init(){
		static float get_time_rtt(){
			uint ul_previous_time = rtt_read_timer_value(RTT);
		}

		static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
		{
			uint32_t ul_previous_time;

			/* Configure RTT for a 1 second tick interrupt */
			rtt_sel_source(RTT, false);
			rtt_init(RTT, pllPreScale);
	
			ul_previous_time = rtt_read_timer_value(RTT);
			while (ul_previous_time == rtt_read_timer_value(RTT));
	
			rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

			/* Enable RTT interrupt */
			NVIC_DisableIRQ(RTT_IRQn);
			NVIC_ClearPendingIRQ(RTT_IRQn);
			NVIC_SetPriority(RTT_IRQn, 0);
			NVIC_EnableIRQ(RTT_IRQn);
			rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
	
}

void io_init(){
	pmc_enable_periph_clk(BUT_PIO_ID);
	// Configura PIO para lidar com o pino do bot?o como entrada
	// com pull-up
	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP);
	
	pio_handler_set(BUT_PIO,
	BUT_PIO_ID,
	BUT_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_flag_callback);
}



int main(void) {
	board_init();
	sysclk_init();	
	configure_lcd();
	io_init();
	rtc_init();
	
	rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
	rtc_set_time_alarm(RTC, 1, HOUR, 1, MINUTE, 1, SECOND+1);
	uint32_t seganterior;
	char c_hora[32],c_min[32],c_seg[32];

	font_draw_text(&calibri_36, "Distancia total:", 20, 20, 1);
	font_draw_text(&calibri_36, "V.Instantanea:", 20, 180, 1);
	font_draw_text(&calibri_36, "Tempo total:", 20, 340, 1);

	while(1) {
		

			rtc_get_time(RTC, &hora, &minuto, &seg);

			sprintf(c_hora,"%d",hora);
			sprintf(c_min,"%d",minuto);
			sprintf(c_seg,"%d",seg);
			if(seganterior!=seg){
				font_draw_text(&calibri_36, "           ", 30, 390, 1);
				font_draw_text(&calibri_36, c_hora, 30, 390, 1);
				font_draw_text(&calibri_36,  c_min , 60, 390, 1);
				font_draw_text(&calibri_36, c_seg, 90, 390, 1);
				seganterior = seg;
			}


	}
}