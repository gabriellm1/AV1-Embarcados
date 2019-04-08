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



//butao 1 oled
#define EBUT1_PIO PIOD //start EXT 9 PD28
#define EBUT1_PIO_ID 16
#define EBUT1_PIO_IDX 28
#define EBUT1_PIO_IDX_MASK (1u << EBUT1_PIO_IDX)

// defines rtc
#define YEAR        2019
#define MOUNTH      4
#define DAY         8
#define WEEK        12
#define HOUR        0
#define MINUTE      0
#define SECOND      0

#define PI          3.14
#define R           0.325

uint32_t hora, minuto, seg;
int pulso = 0;
int dist= 0 ;

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

volatile Bool but_flag;
void but_flag_callback(void){
	but_flag = true;
}


int velocidade(int pulsos){
	int w = (2*PI*pulsos)/4; // dT e 4 segundos
	return w*R*3.6;
}

int distancia(int pulsos){
	return 2*PI*R*pulsos;
}

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
	pmc_enable_periph_clk(EBUT1_PIO_ID);
	// Configura PIO para lidar com o pino do bot?o como entrada
	// com pull-up
	pio_configure(EBUT1_PIO, PIO_INPUT, EBUT1_PIO_IDX_MASK, PIO_PULLUP);
	pio_set_debounce_filter(EBUT1_PIO,EBUT1_PIO_IDX_MASK,20);

	
	// Ativa interrup??o
	pio_enable_interrupt(EBUT1_PIO, EBUT1_PIO_IDX_MASK);
	
	

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 1 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(EBUT1_PIO_ID);
	NVIC_SetPriority(EBUT1_PIO_ID, 1); // Prioridade 1
	
	pio_handler_set(EBUT1_PIO,
	EBUT1_PIO_ID,
	EBUT1_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but_flag_callback);
}



int main(void) {
	board_init();
	sysclk_init();	
	configure_lcd();
	io_init();
	rtc_init();
	

	
	f_rtt_alarme = true;
	
	rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
	rtc_set_time_alarm(RTC, 1, HOUR, 1, MINUTE, 1, SECOND+1);
	uint32_t seganterior;
	char c_hora[32],c_min[32],c_seg[32];

	font_draw_text(&calibri_36, "Distancia total:", 20, 20, 1);
	font_draw_text(&calibri_36, "V.Instantanea:", 20, 180, 1);
	font_draw_text(&calibri_36, "Tempo total:", 20, 340, 1);

	while(1) {
	
	if(but_flag){
		pulso+=1;
		but_flag = false;
	}	
	
	if (f_rtt_alarme){

      uint16_t pllPreScale = (int) (((float) 32768) / 1.0);
      uint32_t irqRTTvalue  = 8;
      
      // reinicia RTT para gerar um novo IRQ
      RTT_init(pllPreScale, irqRTTvalue);         
      

      
      /*
       * CLEAR FLAG
       */
      f_rtt_alarme = false;
	  
		char distc[32];
		dist += distancia(pulso);
		sprintf(distc,"%d",dist);
		font_draw_text(&calibri_36, "         ", 30, 60, 1);
		font_draw_text(&calibri_36, distc, 30, 60, 1);
		font_draw_text(&calibri_36, "m", 70, 60, 1);
	   
		char vel[32];
		sprintf(vel,"%d",velocidade(pulso));
		font_draw_text(&calibri_36, "         ", 30, 220, 1);
		font_draw_text(&calibri_36, vel, 30, 220, 1);
		font_draw_text(&calibri_36, "km/h", 70, 220, 1);
		pulso = 0;
    }


	rtc_get_time(RTC, &hora, &minuto, &seg);

	sprintf(c_hora,"%d",hora);
	sprintf(c_min,"%d",minuto);
	sprintf(c_seg,"%d",seg);
	if(seganterior!=seg){
		font_draw_text(&calibri_36, "                                    ", 30, 390, 1);
		font_draw_text(&calibri_36, c_hora, 30, 390, 1);
		font_draw_text(&calibri_36, ":", 50, 390, 1);
		font_draw_text(&calibri_36,  c_min , 80, 390, 1);
		font_draw_text(&calibri_36, ":", 100, 390, 1);
		font_draw_text(&calibri_36, c_seg, 130, 390, 1);
		seganterior = seg;
	}
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

	}
}

