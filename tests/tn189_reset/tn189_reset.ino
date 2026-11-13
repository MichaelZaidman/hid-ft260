// SPDX-License-Identifier: MIT
/*
 * Drive the FT260 RESETN pin for tn189_reset_loop.py.
 *
 * Serial at 115200 baud: 'L' holds the FT260 in reset, 'H' releases it.
 * The sketch answers each command with the same letter and a newline,
 * and ignores any other byte.
 */

#define RESET_PIN 2

void setup()
{
	digitalWrite(RESET_PIN, HIGH);
	pinMode(RESET_PIN, OUTPUT);
	Serial.begin(115200);
	Serial.println("ready");
}

void loop()
{
	int c = Serial.read();

	if (c == 'L')
		digitalWrite(RESET_PIN, LOW);
	else if (c == 'H')
		digitalWrite(RESET_PIN, HIGH);
	else
		return;
	Serial.write(c);
	Serial.write('\n');
}
