/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

int amount_of_fires = 0;

void set_indexer_piston(bool input) { indexerPiston.set_value(input); }

void fire_indexer(int fire_amount) {
  if (getRPM() >= 1000) {
    indexer_on = true;
    amount_of_fires += fire_amount;
  }
}

void indexer_control() {
  const int active_time = 100;  // keep in multiples of 10 ms
  const int deactive_time = 250;
  int timer = 0;
  while (true) {
    // When the timer has reached, disable piston and don't let the piston reengage for another 250ms
    if (indexer_on && timer >= active_time) {
      set_indexer_piston(false);
      if (timer >= active_time + deactive_time) {
        amount_of_fires--;
        if (amount_of_fires == 0) {
          indexer_on = false;
          amount_of_fires = 0;
        }
        timer = 0;
      }
      timer += DELAY_TIME;
    }
    // When initially turns on, trigger piston
    else if (indexer_on) {
      set_indexer_piston(true);
      timer += DELAY_TIME;
    }

    printf("%i\n", amount_of_fires);

    pros::delay(DELAY_TIME);
  }
}
pros::Task indexerControl(indexer_control);

bool last_indexer_press = false;
bool current_press = false;
void indexer_opcontrol() {
  if (master.get_digital_new_press(B_INDEXER_SPAM) && amount_of_fires < 3) {
    fire_indexer();
  }

  if (master.get_digital_new_press(B_INDEXER_TRIPPLE) && !indexer_on) {
    fire_indexer(3);
  }
}
