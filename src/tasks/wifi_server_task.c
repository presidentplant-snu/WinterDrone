#include "wifi_server_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "mongoose.h"

#include "wifi/wifi.h"
#include "config.h"

#include "./combined_html.h"
#include "./shared_data.h"

static void event_handler(struct mg_connection *c, int ev, void *ev_data) {
	if (ev == MG_EV_HTTP_MSG) {
		struct mg_http_message *hm = ev_data;  // Parsed HTTP request
											   //
		if (mg_match(hm->uri, mg_str("/ws"),NULL)) {
			mg_ws_upgrade(c, hm, NULL);
			return;
		}  
		mg_http_reply(c, 200, "Content-Type: text/html\r\n", "%s", combined_index_html);
	}
	else if (ev == MG_EV_WS_MSG){
		struct mg_ws_message *wm = (struct mg_ws_message *) ev_data;


		if (xSemaphoreTake(xControlSemaphore, pdMS_TO_TICKS(SEMAPHORE_DELAY_MULT*SERVER_DELAY)) == pdTRUE) {
			if(wm->data.len == 1){
				if(wm->data.buf[0] == 0x00){
					g_controlData.armed = true;
				}
				else if(wm->data.buf[0] == 0x01){
					g_controlData.armed = false;
				}
			}
			else if(wm->data.len == 5){
				if(wm->data.buf[0] == 0x02){
					g_controlData.throttle = wm->data.buf[1];
					g_controlData.pitch = wm->data.buf[2];
					g_controlData.yaw = wm->data.buf[3];
					g_controlData.roll = wm->data.buf[4];
				}
			}
			xSemaphoreGive(xControlSemaphore);
		}
		// TODO add telemetry / bidirectional comms
		//mg_ws_send(c, wm->data.buf, wm->data.len, WEBSOCKET_OP_TEXT);
	}
}

void wifi_server_task(__unused void *args) {
	init_wifi(SSID,PSK,true);

	sleep_ms(1000);

	struct mg_mgr mgr;        // Initialise Mongoose event manager
	mg_log_set(1);  // Set log level
	mg_mgr_init(&mgr);        // and attach it to the interface
	MG_INFO(("Initialising application..."));
	//web_init(&mgr);

	mg_http_listen(&mgr, "http://0.0.0.0/", event_handler, NULL);

	MG_INFO(("Starting event loop"));
	int i=0;
	for (;;) {
		i++;
		mg_mgr_poll(&mgr, SERVER_DELAY);
	}

	deinit_wifi();
}
