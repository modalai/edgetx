/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "board.h"

// The Zorro Blue has no audio hardware: hal.h defines no DAC, speaker or
// AUDIO_MUTE pins, and the target compiles none of the shared audio drivers.
//
// AUDIO is still enabled, because it is on by default and turning it off is no
// longer a supported upstream configuration -- the no-op AUDIO_* macros that
// used to cover that case lived in radio/src/buzzer.h, which was deleted in the
// 2.12 B&W refactor, so an AUDIO=NO build now fails on undefined AUDIO_PLAY and
// friends.
//
// So the audio stack builds and runs, and just needs somewhere for its buffers
// to go. A no-op is safe rather than merely convenient: AudioQueue::wakeup()
// loops only while buffersFifo.getEmptyBuffer() yields a buffer, so leaving
// them unconsumed makes the FIFO fill and the loop exit. Nothing spins, nothing
// plays.
void audioConsumeCurrentBuffer()
{
}
