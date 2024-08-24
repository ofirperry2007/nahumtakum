#include "clicli.h"
#include "nahumtakum.h"

clicli cli;
nahumtakum nahumtakum(5, 6, 7);

void setup() {
  cli.begin();
  nahumtakum.begin();
}

void loop() {
  cli.run();
  nahumtakum.run();//updates the gyro values
  nahumtakum.tumble();
}
