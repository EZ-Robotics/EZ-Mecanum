#include "main.h"

void set_indexer(bool input) { indexerPiston.set_value(input); }

void indexer_control() {
  const int active_time = 500;  // keep in multiples of 10 ms
  int timer = -1;

  while (1) {
    if (indexer_state == 1) {
      indexer_state = 2;
      timer = active_time;
    }

    while (timer >= 0) {
      timer -= DELAY_TIME;
      set_indexer(true);
      pros::delay(DELAY_TIME);
    }

    if (indexer_state == 2) indexer_state = 0;
    set_indexer(false);

    pros::delay(DELAY_TIME);
  }
}
pros::Task indexerControl(indexer_control);

void indexer_opcontrol() {
  if (master.get_digital_new_press(DIGITAL_DOWN) && (indexer_state == 0)) {
    indexer_state = 1;
  }
}