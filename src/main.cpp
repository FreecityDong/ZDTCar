#include <Arduino.h>

#include "app/app.h"

namespace {

App app;

}  // namespace

void setup() { app.begin(); }

void loop() { app.update(); }
