#include "pico/cyw43_arch.h"
#include <stdio.h>
#include "pico/stdlib.h"

int init_wifi(char* ssid, char* psk){
cyw43_arch_enable_sta_mode();

	if(cyw43_arch_wifi_connect_timeout_ms(ssid,psk,CYW43_AUTH_WPA2_AES_PSK, 10000)){
		return PICO_ERROR_GENERIC;
	}
	else{
		return PICO_OK;
	}
}
