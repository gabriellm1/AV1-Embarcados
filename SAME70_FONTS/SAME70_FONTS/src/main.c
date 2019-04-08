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

struct ili9488_opt_t g_ili9488_display_opt;

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

char escreve_num(int num){
	char buffer[32];
	sprintf(buffer,"%d",num);
	return buffer;
}

int main(void) {
	board_init();
	sysclk_init();	
	configure_lcd();
	io_init();
	rtc_init();
	
	
	font_draw_text(&calibri_36, "Distancia total:", 20, 20, 1);
	font_draw_text(&calibri_36, "V.Instantanea:", 20, 180, 1);
	font_draw_text(&calibri_36, "Tempo total:", 20, 340, 1);
	font_draw_text(&calibri_36, escreve_num(20), 30, 390, 1);
	while(1) {
		
	}
}