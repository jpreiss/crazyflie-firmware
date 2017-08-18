#pragma once

#include <stdbool.h>

// plays a short song using the propeller speeds to generate tones.

void songBegin(int I2CAddr);

void songStep(int tick);

bool songIsDone();
