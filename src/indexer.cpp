/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "flywheel.hpp"
#include "main.h"

// Queue for indexer
int indexer_queue = 0;

// Set piston
void set_indexer_piston(bool input) { indexerPiston.set_value(input); }

// Adds to indexer queue
void fire_indexer(int fire_amount) {
  indexer_on = true;
  indexer_queue += fire_amount;
}

// Indexer task
void indexer_control() {
  const int active_time = 100;  // keep in multiples of 10 ms
  const int deactive_time = 250;
  int timer = 0;
  while (true) {
    // When the timer has reached, disable piston and don't let the piston reengage for another 250ms
    if (indexer_on && timer >= active_time) {
      set_indexer_piston(false);
      if (timer >= active_time + deactive_time) {
        indexer_queue--;
        // while (!is_flywheel_at_rpm()) pros::delay(1);
        if (indexer_queue == 0) {
          indexer_on = false;
          indexer_queue = 0;
        }
        timer = 0;
      }
      timer += DELAY_TIME;
    }
    // When initially turns on, trigger piston when flywheel is at rpm
    else if (indexer_on) {
      set_indexer_piston(true);
      timer += DELAY_TIME;
    }

    pros::delay(DELAY_TIME);
  }
}
pros::Task indexerControl(indexer_control);

// Opcontrol indexer
void indexer_opcontrol() {
  if (getRPM() >= 1000) {
    if (master.get_digital_new_press(B_INDEXER_SPAM) && indexer_queue < 3)
      fire_indexer();

    if (master.get_digital_new_press(B_INDEXER_TRIPPLE) && !indexer_on)
      fire_indexer(3);
  }
}
