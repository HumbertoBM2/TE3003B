#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>

int main(int argc, char **argv){
  const char *chipname = "gpiochip0";
  struct gpiod_chip *chip;
  struct gpiod_line *lineYellow; // Yellow LED
  int i, ret, val;
  // Open GPIO chip
  chip = gpiod_chip_open_by_name(chipname);
  if (!chip) {
    perror("Open chip failed\n");
    return 1;
  }

  // Open GPIO lines
  lineYellow = gpiod_chip_get_line(chip, 216);
  if (!lineYellow) {
    perror("Get line failed\n");
    return 1;
  }

 // Open LED lines for output
  ret = gpiod_line_request_output(lineYellow,"example1",0);
  if (ret < 0) {
    perror("Request line as output failed\n");
    return 1;
  }
  // Blink LEDs in a binary pattern
  i = 0;

  while (true) {
    ret = gpiod_line_set_value(lineYellow,(i & 4) != 0);
    if (ret < 0) {
      perror("Set line output failed\n");
      return 1;
    }
    usleep(100000);
    i++;
  }
  // Release lines and chip
  gpiod_line_release(lineYellow);
  gpiod_chip_close(chip);
  return 0;
}

