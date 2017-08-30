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

// `r` is a rest - the propeller will stop.

static int songCursor;
static int motorCursor;
static int noteStartTick;
static int noteTicks;
static bool songDone;
static int I2CAddr;

void songBegin(int addr)
{
	songCursor = 0;
	motorCursor = 0;
	noteStartTick = -10000;
	noteTicks = 0;
	songDone = false;
	I2CAddr = addr;
}

// it's important to have a rest (r) at the end,
// otherwise the last tone will go on forever.

// "Redbone" by Childish Gambino
static int const song[] = {
	r, r, r, r, r, r, r, r, D, D, D, r, Ah, Ah, r, G, r, F, r, E, r, C, r, A, r, D, D, D, D, r, };

// only one motor is ever playing a tone... to avoid taking off
static void setTone(int motor, int hz)
{
	// at 385hz, 1500/4096 pulsewidth means motor stopped but ESC still armed
	int const d0 = 1500;

	uint16_t durations[6] = { d0, d0, d0, d0, d0, d0 };

	// tones are one octave too high for 6-inch props :)
	hz = hz / 2;
	durations[motor] = (5 * hz) / 2 + 1471;
	pca9685setDurationsAsync(I2CAddr, 0, 6, durations);
}

// assumes that tick always >= 0.
void songStep(int tick)
{
	int const nNotes = sizeof(song) / sizeof(song[0]);
	if (songCursor >= nNotes) {
		songDone = true;
	}
	else if (tick - noteStartTick >= noteTicks) {
		int const note = song[songCursor];
		int const prevNote = songCursor > 0 ? song[songCursor - 1] : r;

		if (note == r) {
			// stop all motors
			setTone(0, 0);
		}
		else if (note != prevNote) {
			setTone(motorCursor, note);
			motorCursor = (motorCursor + 1) % 6;
		}

		++songCursor;
		//noteStartTick += noteTicks;
		noteStartTick = tick;
		noteTicks = 100;
	}
}

bool songIsDone()
{
	return songDone;
}
