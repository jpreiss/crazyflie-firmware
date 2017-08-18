#include "tilthex_song.h"
#include "pca9685.h"

// musical tones
#define A 220
#define B 247
#define C 262
#define D 294
#define E 330
#define F 349
#define G 392
#define Ah 440
#define r 0
#define x 1

// `r` is a rest - the propeller will stop.
// `h` is a hold - no motor command will be sent.
// note that e.g. {D, h} and {D, D} are equivalent.

static int songCursor;
static int noteStartTick;
static int noteTicks;
static bool songDone;
static int I2CAddr;

// it's important to have a rest (r) at the end,
// otherwise the last tone will go on forever.

// "Redbone" by Childish Gambino
static int const song[] = {
	x, x, x, x, x, x, x, x, D, D, D, r, Ah, Ah, r, G, r, F, r, E, r, C, r, A, r, D, D, D, D, r, };

static void setTone(int hz)
{
	if (hz > 1) {
		// tones are one octave too high for 6-inch props :)
		hz = hz / 2;
	}

	uint16_t const dur = 
		(hz == r) ? 0 :
		(5 * hz) / 2 + 1471;

	if (hz != x) {
		pca9685setDurationsAsync(I2CAddr, 0, 1, &dur);
	}
}

void songBegin(int addr)
{
	songCursor = 0;
	noteStartTick = -10000;
	noteTicks = 0;
	songDone = false;
	I2CAddr = addr;
}

// assumes that tick always >= 0.
void songStep(int tick)
{
	int const nNotes = sizeof(song) / sizeof(song[0]);
	if (songCursor >= nNotes) {
		songDone = true;
	}
	else if (tick - noteStartTick >= noteTicks) {
		// tones are an octave too high for 6" props...
		setTone(song[songCursor]);
		++songCursor;
		noteStartTick = tick;
		noteTicks = 100;
	}
}

bool songIsDone()
{
	return songDone;
}
