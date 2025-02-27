#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

int init_wifi(char* ssid, char* psk, uint8_t ap_mode){
	if(cyw43_arch_init()){
		return PICO_ERROR_GENERIC;
	}

	if(ap_mode){
		cyw43_arch_enable_ap_mode(ssid,psk,CYW43_AUTH_WPA2_AES_PSK);
		return PICO_OK;
	}
	else{
		cyw43_arch_enable_sta_mode();

		if(cyw43_arch_wifi_connect_timeout_ms(ssid,psk,CYW43_AUTH_WPA2_AES_PSK, 10000)){
			return PICO_ERROR_GENERIC;
		}
		else{
			return PICO_OK;
		}
	}
}



void deinit_wifi(){
	cyw43_arch_deinit();
}
