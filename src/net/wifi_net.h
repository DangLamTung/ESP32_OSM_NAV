/**
 * wifi_net.h — optional STA WiFi for network tile fetching.
 * The gear/settings panel has a button to connect; on success the tile source
 * switches from SD-only to AUTO so missing tiles load over the network.
 */
#ifndef WIFI_NET_H_
#define WIFI_NET_H_

#include <stdbool.h>

void wifi_net_connect(void);       /* non-blocking toggle; AUTO tiles on success */
void wifi_net_disconnect(void);    /* disconnect; back to SD-only tiles */
bool wifi_net_connected(void);
bool wifi_net_pending(void);       /* a connect is in flight (UI shows "...") */
const char *wifi_net_label(void);  /* "WiFi: off | on | ..." */

#endif // WIFI_NET_H_
