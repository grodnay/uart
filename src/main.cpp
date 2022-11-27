#include "defines.h"
#include <Arduino.h>

#define ASYNC_UDP_TEENSY41_VERSION_MIN_TARGET "AsyncUDP_Teensy41 v1.2.1"
#define ASYNC_UDP_TEENSY41_VERSION_MIN 1002001

#include <AsyncUDP_Teensy41.h> // https://github.com/khoih-prog/AsyncUDP_Teensy41

#define max_buffer_size 8000

void start_ethernet(void)
{
#if USING_DHCP
  // Start the Ethernet connection, using DHCP
  Serial.print("Initialize Ethernet using DHCP => ");
  Ethernet.begin();
#else
  // Start the Ethernet connection, using static IP
  Serial.print("Initialize Ethernet using static IP => ");
  Ethernet.begin(myIP, myNetmask, myGW);
  Ethernet.setDNSServerIP(mydnsServer);
#endif

  if (!Ethernet.waitForLocalIP(5000))
  {
    Serial.println(F("Failed to configure Ethernet"));

    if (!Ethernet.linkStatus())
    {
      Serial.println(F("Ethernet cable is not connected."));
    }

    // Stay here forever
    while (true)
    {
      delay(1);
    }
  }
  else
  {
    Serial.print(F("Connected! IP address:"));
    Serial.println(Ethernet.localIP());
  }

#if USING_DHCP
  delay(1000);
#else
  delay(2000);
#endif
  delay(500);
};

class eth2uart
{
  AsyncUDP udp;
  HardwareSerial *uart;
  long unsigned int dropped, oldest_byte_time, max_latency;
  size_t buffer_size, packet_size;
  byte msg[max_buffer_size], uart_in_buffer[max_buffer_size], uart_out_buffer[max_buffer_size];
  void udp_cb(AsyncUDPPacket packet)
  {
    if (uart->availableForWrite() >= packet.length())
      uart->write(packet.data(), packet.length());
    else
      dropped++;
  }
  void udp_send(void)
  {
    int availableForRead = uart->available();
    unsigned long int now = millis();
    if (availableForRead == 0)
    {
      oldest_byte_time = now;
      return;
    }
    if ((availableForRead >= packet_size) || (now - oldest_byte_time > max_latency && availableForRead > 0))
    {
      uart->readBytes(msg, availableForRead);
      if (udp.write(msg, availableForRead) < availableForRead)
        dropped++;
      oldest_byte_time = now;
    }
  }

public:
  void connect(const byte IP[], unsigned int port, HardwareSerial &_uart, unsigned long int baud, int _packet_size = 60, int _max_latency = 15)
  {
    Serial.print("\nStart Async_UDPClient on ");
    Serial.println(BOARD_NAME);
    Serial.println(ASYNC_UDP_TEENSY41_VERSION);
#if defined(ASYNC_UDP_TEENSY41_VERSION_MIN)
    if (ASYNC_UDP_TEENSY41_VERSION_INT < ASYNC_UDP_TEENSY41_VERSION_MIN)
    {
      Serial.print("Warning. Must use Version equal or later than : ");
      Serial.println(ASYNC_UDP_TEENSY41_VERSION_MIN_TARGET);
    }
#endif
    dropped = 0;
    oldest_byte_time = millis();

    if (udp.connect(IPAddress(IP[0], IP[1], IP[2], IP[3]), port))
    {
      Serial.println("UDP connected");

      udp.onPacket([this](AsyncUDPPacket packet)
                   { this->udp_cb(packet); });

      // Send unicast
      delay(3000);
     
      udp.print("Hello Server!");
    }

    packet_size = _packet_size;
    buffer_size = 2 * packet_size;
    uart = &_uart;
    uart->begin(baud);
    uart->addMemoryForRead(uart_in_buffer, buffer_size);
    uart->addMemoryForWrite(uart_out_buffer, buffer_size);

    max_latency = _max_latency;
  }
  void update(void)
  {
    udp_send();
  }
};

class timer
{
  unsigned long int t0 = 0, dt = 0;
public:
  void start(unsigned long int _dt)
  {
    dt = _dt;
    restart();
  }
  void restart(void)
  {
    t0 = millis();
  }
  bool ellapsed(void)
  {
    return millis() - t0 > dt;
  }
};

eth2uart uart1, uart2;
timer my_timer;

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  start_ethernet();
  byte ip[] = {192, 168, 1, 10};
  uart1.connect(ip, 5698, Serial1, 115200);
//  uart2.connect(ip, 5699, Serial2, 115200);
  delay(1000);
  my_timer.start(1000);
}

int i1 = 0, i2 = 0;

void loop()
{
  uart1.update();
  //uart2.update();
  // if (my_timer.ellapsed())
  // {
  //   my_timer.restart();
  //   Serial1.printf("%d Hello1\n", ++i1);
  //   Serial2.printf("%d Hello2\n", ++i2);
  // }
  delay(1);
}
