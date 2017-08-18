#pragma once

#include <stdbool.h>

// plays a short song using the propeller speeds to generate tones.
// uses all 6 motors, so useful as a hardware power-on self-test.
void songBegin(int I2CAddr);

void songStep(int tick);

bool songIsDone();
