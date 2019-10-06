#include <Arduino.h>
#include "WiFiMesh.h"

// node number 0 for root
#define NODE_NUM 0

// mesh network parameters
#define MESH_CHANNEL 13
#define MESH_ID "123456"

#if NODE_NUM == 0
WiFiMeshRoot mesh;
#else
WiFiMeshNode mesh;
#endif

extern "C" {
int32_t hall_sens_read();
}

void setup()
{
  Serial.begin(115200);
  mesh.begin(MESH_CHANNEL, MESH_ID);
}

void loop()
{
  char buf[128];
#if NODE_NUM == 0
  if (mesh.receive(buf, sizeof(buf) - 1)) {
    Serial.println(buf);
    int len = sprintf(buf, "ACK from root!", millis());
    mesh.send(buf, len);
  }
#else
  int len = sprintf(buf, "Hello from node #%u! %u %d", NODE_NUM, millis(), hall_sens_read());
  Serial.println(buf);
  mesh.send(buf, len);
  if (mesh.receive(buf, sizeof(buf) - 1)) {
    Serial.println(buf);
  }
  delay(1000);
#endif
}
