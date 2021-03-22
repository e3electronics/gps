#include <stdio.h>
#include "common/mbuf.h"
#include "common/platform.h"
#include "mgos_app.h"
#include "mgos_gpio.h"
#include "mgos_timers.h"
#include "mgos_uart.h"
#include "common/json_utils.h"
#include "mgos.h"
#include "gps.h"
#include "minmea.h"

#define uart_size 80
#define uart_rx_buf_size 1024

static int gps_uart_no = 0;
static size_t gpsDataAvailable = 0;
// static struct minmea_sentence_rmc lastFrame;
static char *refactory_sentence(char *raw_sentence);
static void gps_uart_read(void *arg);
int esp32_uart_rx_fifo_len(int uart_no);
/**
 * 
 */
char *mgos_gps_get_location()
{
    gps_uart_read(NULL);
    struct mbuf fb;
    struct json_out out = JSON_OUT_MBUF(&fb);
    //printf("GPS Request direct \n");
    mbuf_init(&fb, 50);
    float lat = minmea_tocoord(&lastFrame.latitude);
    float lon = minmea_tocoord(&lastFrame.longitude);
    float speed = minmea_tocoord(&lastFrame.speed);
    LOG(LL_INFO, ("Latitud: %f Longitud: %f Speed %f", lat, lon, speed));
    if (lat == NAN)
    {
        lat = 0.0f;
    }
    if (lon == NAN)
    {
        lon = 0.0f;
    }
    if (speed == NAN)
    {
        speed = 0.0f;
    }
    json_printf(&out, "{lat: \"%f\", lon: \"%f\", sp: \"%f\"}", lat, lon, speed);
    //mbuf_free(&fb);
    return fb.buf;
}
/**
 * Limpia la trama de los caracteres repetidos <CR><LF>  que puedan invalidar la sentencia en minmea_sentence_id() y
 * reconstruye la trama.
 * Se realiza para establecer compatibilidad con Quectel EC21 UART GNSS por enviar en la trama <CR><CR> 
 * 
 */
static char *refactory_sentence(char *raw_sentence)
{
    //char *line = "$GPGGA,130814.00,3329.769376,S,07039.465721,W,1,02,1.4,533.7,M,32.0,M,,*67\r\n";
    char *line = "$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62\r\n";
    //printf("raw sentence: %s \n", line );
    // char lineNmea[MINMEA_MAX_LENGTH];
    // strncpy(lineNmea, tmp, sizeof(lineNmea) - 1);
    // strcat(lineNmea, "\n");
    // lineNmea[sizeof(lineNmea) - 1] = '\0';
    // enum minmea_sentence_id id = minmea_sentence_id(lineNmea, false);
    return line;
}
/**
 * 
 */
/**
 *  &frame.time
 *  &frame.latitude
 *  &frame.longitude
 *  &frame.speed
 *  &frame.course
 *  &frame.date
 *  &frame.variation
 *  minmea_tocoord({-375165, 100}) => -37.860832
 *  assert_float_eq(minmea_tofloat(&(struct minmea_float) { -200, 100 }), -2.0f);
 */
static void parseGpsData(char* line){
    switch (minmea_sentence_id(line, false)) {
	case MINMEA_SENTENCE_RMC: {
	struct minmea_sentence_rmc frame;
	if (minmea_parse_rmc(&frame, line)) {
	   //D("$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
	   //frame.latitude.value, frame.latitude.scale,
	   //frame.longitude.value, frame.longitude.scale,
	   //frame.speed.value, frame.speed.scale);
	   //D("$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
	   //minmea_rescale(&frame.latitude, 1000),
	   //minmea_rescale(&frame.longitude, 1000),
	   //minmea_rescale(&frame.speed, 1000));
 	   float lat = minmea_tocoord(&frame.latitude),
	   float lon = minmea_tocoord(&frame.longitude),
	   float speed = minmea_tofloat(&frame.speed)),
	   LOG(LL_INFO, ("Latitud: %f Longitud: %f Speed %f", lat, lon, speed));
	}else {
	   LOG(LL_INFO, ("$xxRMC sentence is not parsed\n");
	 }
	} break;
 
    case MINMEA_INVALID:
    {
        break;
    }
    case MINMEA_UNKNOWN:
    {
        break;
    }
    case MINMEA_SENTENCE_GSA:
    {
        break;
    }
    case MINMEA_SENTENCE_GSV:
    {
        break;
    }
    case MINMEA_SENTENCE_GLL:
    {
        break;
    }
    case MINMEA_SENTENCE_GST:
    {
        break;
    }
    case MINMEA_SENTENCE_VTG:
    {
        break;
    }
    case MINMEA_SENTENCE_ZDA:
    {
        break;
    }
    }
}
/**
 *
 */
static void gps_uart_read(void *arg)
{
    if (gpsDataAvailable > 0)
    {
        struct mbuf rxb;
        mbuf_init(&rxb, 0);
        int c = 0;
        mgos_uart_read_mbuf(gps_uart_no, &rxb, gpsDataAvailable);
        if (rxb.len > 0)
        {
            char *pch;
            //printf(">>>>>>> %.*s", (int)rxb.len, rxb.buf);
            pch = strtok(rxb.buf, "\n");
            while (pch != NULL)
            {
                printf("%d GPS lineNmea: %s \n", c++, pch);
                parseGpsData(pch);
                pch = strtok(NULL, "\n");
            }
        }
        mbuf_free(&rxb);
        gpsDataAvailable = 0;
    }
    (void)arg;
}
/**
 * 
 */
static void uart_dispatcher(int uart_no, void *arg){
//eg1. $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
    assert(uart_no == gps_uart_no);
//    size_t rx_av = mgos_uart_read_avail(uart_no);
//    if (rx_av > 0)
//    {
//        gpsDataAvailable = rx_av;
//    }
    static byte index = 0;
    char endMarker = "\r";                                  //or maybe "\n" 
    char temp_char;
    char line[uart_size];
    while(mgos_uart_read_avail(uart_no) > 0){
       temp_char = mgos_uart_read(uart_no);
       if(temp_char != endMarker){
          line[index] = temp_char;
          index++;
       }else{
          index = 0;
          parseGpsData(line);
          memset (line,0, sizeof (line));
       }    
    }
    
    (void)arg;
}
/**
 */
bool mgos_gps_init(void)
{
    if (!mgos_sys_config_get_gps_enable())
        return true;
    struct mgos_uart_config ucfg;
    gps_uart_no = mgos_sys_config_get_gps_uart_no();
    mgos_uart_config_set_defaults(gps_uart_no, &ucfg);
    ucfg.baud_rate = mgos_sys_config_get_gps_baud_rate();
    ucfg.num_data_bits = 8;
    if (mgos_sys_config_get_gps_uart_rx_gpio() >= 0)
    {
        ucfg.dev.rx_gpio = mgos_sys_config_get_gps_uart_rx_gpio();
    }
    if (mgos_sys_config_get_gps_uart_tx_gpio() >= 0)
    {
        ucfg.dev.tx_gpio = mgos_sys_config_get_gps_uart_tx_gpio();
    }
    char b1[8], b2[8];
    LOG(LL_INFO, ("GNSS UART%d (RX:%s TX:%s)",
                  gps_uart_no, mgos_gpio_str(ucfg.dev.rx_gpio, b1),
                  mgos_gpio_str(ucfg.dev.tx_gpio, b2)));
    //ucfg.parity = MGOS_UART_PARITY_NONE;
    //ucfg.stop_bits = MGOS_UART_STOP_BITS_1;
    if (!mgos_uart_configure(gps_uart_no, &ucfg))
    {
        LOG(LL_ERROR, ("Failed to configure GNSS UART%d", gps_uart_no));
        return false;
    }
    mgos_uart_set_dispatcher(gps_uart_no, uart_dispatcher, NULL /* arg */);
    mgos_uart_set_rx_enabled(gps_uart_no, true);
    return true;
}
