/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <string.h>

#include "main.h"

bool AUTON_RAN = true;

double clip_num(double input, double max, double min) {
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}

int sgn(double input) {
  if (input > 0)
    return 1;
  else if (input < 0)
    return -1;
  return 0;
}

std::string get_last_word(std::string text) {
  std::string word = "";
  for (int i = text.length() - 1; i >= 0; i--) {
    if (text[i] != ' ') {
      word += text[i];
    } else {
      std::reverse(word.begin(), word.end());
      return word;
    }
  }
  std::reverse(word.begin(), word.end());
  return word;
}
std::string get_rest_of_the_word(std::string text, int position) {
  std::string word = "";
  for (int i = position; i < text.length(); i++) {
    if (text[i] != ' ' && text[i] != '\n') {
      word += text[i];
    } else {
      return word;
    }
  }
  return word;
}
// All iance\n\nWE WIN THESE!!!!!
void print_to_screen(std::string text, int line) {
  int CurrAutoLine = line;
  std::vector<string> texts = {};
  std::string temp = "";

  for (int i = 0; i < text.length(); i++) {
    if (text[i] != '\n' && temp.length() + 1 > 32) {
      auto last_word = get_last_word(temp);
      if (last_word == temp) {
        texts.push_back(temp);
        temp = text[i];
      } else {
        int size = last_word.length();

        auto rest_of_word = get_rest_of_the_word(text, i);
        temp.erase(temp.length() - size, size);
        texts.push_back(temp);
        last_word += rest_of_word;
        i += rest_of_word.length();
        temp = last_word;
        if (i >= text.length() - 1) {
          texts.push_back(temp);
          break;
        }
      }
    }
    if (i >= text.length() - 1) {
      temp += text[i];
      texts.push_back(temp);
      temp = "";
      break;
    } else if (text[i] == '\n') {
      texts.push_back(temp);
      temp = "";
    } else {
      temp += text[i];
    }
  }
  for (auto i : texts) {
    if (CurrAutoLine > 7) {
      pros::lcd::clear();
      pros::lcd::set_text(line, "Out of Bounds. Print Line is too far down");
      return;
    }
    pros::lcd::clear_line(CurrAutoLine);
    pros::lcd::set_text(CurrAutoLine, i);
    CurrAutoLine++;
  }
}

// Print exit conditions
std::string exit_to_string(exit_output input) {
  switch ((int)input) {
    case RUNNING:
      return "Running";
    case SMALL_EXIT:
      return "Small";
    case BIG_EXIT:
      return "Big";
    case VELOCITY_EXIT:
      return "Velocity";
    case mA_EXIT:
      return "mA";
    case ERROR_NO_CONSTANTS:
      return "Error: Exit condition constants not set!";
    default:
      return "Error: Out of bounds!";
  }

  return "Error: Out of bounds!";
}

// Print turn types
std::string turn_types_to_string(turn_types input) {
  switch ((int)input) {
    case FAST_MOVE_REV:
      return "FAST_MOVE_REV";
    case FAST_MOVE_FWD:
      return "FAST_MOVE_FWD";
    case LOOK_AT_TARGET_FWD:
      return "LOOK_AT_TARGET_FWD";
    case LOOK_AT_TARGET_REV:
      return "LOOK_AT_TARGET_REV";
    case HOLD_ANGLE:
      return "HOLD_ANGLE";
    default:
      return "Error: Out of bounds!";
  }

  return "Error: Out of bounds!";
}
void print_path_for_python(std::vector<odom> imovements) {
  bool first = true;
  // Print subpoints
  std::cout << "raw_path = [";
  for (int i = 0; i < imovements.size(); i++) {
    if (!first) std::cout << "   ,";

    // std::cout << "[" << imovements[i].target.x << ", " << imovements[i].target.y << ", " << imovements[i].target.theta << ", \"" << turn_types_to_string(imovements[i].turn_type) << "\", " << imovements[i].max_xy_speed << ", " << imovements[i].max_turn_speed << "] \n";
    std::cout << "[" << imovements[i].target.x / 12.0 << ", " << imovements[i].target.y / 12.0 << "] \n";

    first = false;
  }
  std::cout << "]\n";
}