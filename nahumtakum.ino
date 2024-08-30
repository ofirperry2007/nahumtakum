#include "clicli.h"
#include "nahumtakum.h"

clicli cli;
nahumtakum nahumtakum(3, 4, 5);

void setup() {
  cli.begin();
  nahumtakum.begin();
}

void loop() {
  cli.run();
  nahumtakum.run();//updates the gyro values
  delay(0);
  nahumtakum.tumble(1, 5, 0.05);
  delay(0);
}
