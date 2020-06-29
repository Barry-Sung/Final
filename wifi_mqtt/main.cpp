#include "mbed.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

// GLOBAL VARIABLES
WiFiInterface *wifi;
InterruptIn butt(SW2);
volatile bool publish = false;
volatile int message_num = 0;

void messageArrived(MQTT::MessageData& md) {
   MQTT::Message &message = md.message;
   char msg[300];
   sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
   printf(msg);
   wait_ms(1000);
   char payload[300];
   sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
   printf(payload);
}

void publish_message() {
   publish = true;
}

int main() {

   wifi = WiFiInterface::get_default_instance();
   if (!wifi) {
      printf("ERROR: No WiFiInterface found.\r\n");
      return -1;
   }


   printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
   int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
   if (ret != 0) {
      printf("\nConnection error: %d\r\n", ret);
      return -1;
   }

   
   NetworkInterface* net = wifi;
   MQTTNetwork mqttNetwork(net);
   MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

   //TODO: revise host to your ip
   const char* host = "192.168.0.148";
   const char* topic = "Mbed";
   printf("Connecting to TCP network...\r\n");
   int rc = mqttNetwork.connect(host, 1883);
   if (rc != 0) {
      printf("Connection error.");
      return -1;
   }
   printf("Successfully connected!\r\n");

   MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
   data.MQTTVersion = 3;
   data.clientID.cstring = "Mbed";

   if ((rc = client.connect(data)) != 0){
      printf("Fail to connect MQTT\r\n");
   }
   if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
      printf("Fail to subscribe\r\n");
   }

   butt.rise(&publish_message);

   while (true) {
      if (publish) {
            message_num++;
            MQTT::Message message;
            char buff[100];
            sprintf(buff, "QoS0 Hello, Python! #%d", message_num);
            message.qos = MQTT::QOS0;
            message.retained = false;
            message.dup = false;
            message.payload = (void*) buff;
            message.payloadlen = strlen(buff) + 1;
            rc = client.publish(topic, message);

            printf("rc:  %d\r\n", rc);
            printf("Puslish message: %s\r\n", buff);
            client.yield(10);
            publish = false;
      }
   }
}
