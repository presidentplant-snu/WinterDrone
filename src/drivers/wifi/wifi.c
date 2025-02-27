#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "mongoose.h"

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
static const char *s_web_page = 
    "<!DOCTYPE html>\n"
    "<html>\n"
    "<head>\n"
    "  <title>Mongoose WebSocket Form</title>\n"
    "  <style>\n"
    "    body { font-family: Arial, sans-serif; margin: 20px; }\n"
    "    .form-container { max-width: 300px; margin: 0 auto; }\n"
    "    input, button { margin: 10px 0; padding: 8px; width: 100%; }\n"
    "    #status { margin-top: 20px; color: #666; }\n"
    "  </style>\n"
    "</head>\n"
    "<body>\n"
    "  <div class=\"form-container\">\n"
    "    <h2>WebSocket Form</h2>\n"
    "    <div>\n"
    "      <label for=\"integerValue\">Enter an integer:</label>\n"
    "      <input type=\"number\" id=\"integerValue\" required>\n"
    "    </div>\n"
    "    <button id=\"sendButton\">Send</button>\n"
    "    <div id=\"status\">Not connected</div>\n"
    "  </div>\n"
    "\n"
    "  <script>\n"
    "    let socket;\n"
    "    \n"
    "    // Connect to WebSocket server\n"
    "    function connect() {\n"
    "      const status = document.getElementById('status');\n"
    "      \n"
    "      // Create WebSocket connection\n"
    "      socket = new WebSocket('ws://' + window.location.host + '/ws');\n"
    "      \n"
    "      // Connection opened\n"
    "      socket.addEventListener('open', function(event) {\n"
    "        status.textContent = 'Connected';\n"
    "        status.style.color = 'green';\n"
    "      });\n"
    "      socket.addEventListener('message', function(event) {\n"
    "        status.textContent = 'Received' +event.data;\n"
    "        status.style.color = 'green';\n"
    "      });\n"
    "      // Connection closed\n"
    "      socket.addEventListener('close', function(event) {\n"
    "        status.textContent = 'Disconnected';\n"
    "        status.style.color = 'red';\n"
    "        // Try to reconnect after a delay\n"
    "        setTimeout(connect, 2000);\n"
    "      });\n"
    "      \n"
    "      // Connection error\n"
    "      socket.addEventListener('error', function(event) {\n"
    "        status.textContent = 'Error: ' + event;\n"
    "        status.style.color = 'red';\n"
    "      });\n"
    "    }\n"
    "    \n"
    "    // Initialize connection when page loads\n"
    "    window.addEventListener('load', function() {\n"
    "      connect();\n"
    "      \n"
    "      // Set up button click handler\n"
    "      document.getElementById('sendButton').addEventListener('click', function() {\n"
    "        const integerValue = document.getElementById('integerValue').value;\n"
    "        \n"
    "        // Validate that we have an integer\n"
    "        if (integerValue.trim() === '') {\n"
    "          alert('Please enter an integer');\n"
    "          return;\n"
    "        }\n"
    "        \n"
    "        // Check if socket is connected\n"
    "        if (socket && socket.readyState === WebSocket.OPEN) {\n"
    "          // Send the integer as JSON\n"
    "          socket.send(integerValue);\n"
    "          document.getElementById('status').textContent = 'Sent: ' + integerValue;\n"
    "        } else {\n"
    "          alert('WebSocket is not connected');\n"
    "        }\n"
    "      });\n"
    "    });\n"
    "  </script>\n"
    "</body>\n"
    "</html>\n";
static void event_handler(struct mg_connection *c, int ev, void *ev_data) {
  if (ev == MG_EV_HTTP_MSG) {
    struct mg_http_message *hm = ev_data;  // Parsed HTTP request
										   //
	if (mg_match(hm->uri, mg_str("/ws"),NULL)) {
            mg_ws_upgrade(c, hm, NULL);
			return;
        }  
	mg_http_reply(c, 200, "Content-Type: text/html\r\n", "%s", s_web_page);
  }
  else if (ev == MG_EV_WS_MSG){
		struct mg_ws_message *wm = (struct mg_ws_message *) ev_data;
		for(int i=0; i<wm->data.len; i++){
		printf("%#010x\n", wm->data.buf[i]); 
		}
    	mg_ws_send(c, wm->data.buf, wm->data.len, WEBSOCKET_OP_TEXT);
  }
}

void mongoose(void *args) {
  struct mg_mgr mgr;        // Initialise Mongoose event manager
  mg_log_set(MG_LL_DEBUG);  // Set log level
  mg_mgr_init(&mgr);        // and attach it to the interface
  MG_INFO(("Initialising application..."));
	//web_init(&mgr);

  mg_http_listen(&mgr, "http://0.0.0.0/", event_handler, NULL);

  MG_INFO(("Starting event loop"));
  for (;;) {
    mg_mgr_poll(&mgr, 10);
  }

  (void) args;
}


 

void init_server(){

}

void deinit_wifi(){
	cyw43_arch_deinit();
}
