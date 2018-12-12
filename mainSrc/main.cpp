/*
 * The MIT License (MIT)

Copyright (c) 2015 Jetsonhacks

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
// servoExample.cpp
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <JHPWMPCA9685.h>

// for pipe communication
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

// Debug mode
// #define DEBUG



// Calibrated for main BLDC motor
#define PWM_FULL_REVERSE 1241 // 1ms / 3.3ms * 4096
#define PWM_NEUTRAL 1861 // 1.5ms / 3.3ms * 4096
#define PWM_FULL_FORWARD 2482 // 2ms / 3.3ms * 4096

#define SERVO_CHANNEL 10
#define ESC_CHANNEL 8

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

// Map an integer from one coordinate system to another
// This is used to map the servo values to degrees
// e.g. map(90,0,180,servoMin, servoMax)
// Maps 90 degrees to the servo value
// 체크
int map ( int x, int in_min, int in_max, int out_min, int out_max) {
    int toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
    // For debugging:
    // printf("MAPPED %d to: %d\n", x, toReturn);
    return toReturn ;
}

int main() {
    PCA9685 *pca9685 = new PCA9685();
    int err = pca9685->openPCA9685();
    if (err < 0){
        printf("Error: %d", pca9685->error);
    } else {
        printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
        pca9685->setAllPWM(0,0) ;
        pca9685->reset() ;
        pca9685->setPWMFrequency(300);
	sleep(1);

	pca9685->setPWM(ESC_CHANNEL, 0, PWM_NEUTRAL);
	pca9685->setPWM(SERVO_CHANNEL, 0, PWM_NEUTRAL);
#ifdef DEBUG
        // 27 is the ESC key
        printf("Hit ESC key to exit\n");
	int PWM = PWM_NEUTRAL;
        pca9685->setPWM(ESC_CHANNEL,0, PWM + 150);
        while(pca9685->error >= 0 && getkey() != 27){

            pca9685->setPWM(SERVO_CHANNEL,0,PWM);

	    int mode;
	    printf("Enter 1 for PWM + 100\n\t2 for PWM -100\n\t3 for Neural\n");
	    scanf("%d", &mode);
	    switch(mode){
		case 1:
		  PWM += 10; break;
		case 2:
		  PWM -= 10; break;
		case 3:
		  PWM = PWM_NEUTRAL; break;
		case 4:
		  goto exit;
	    }
        }
	exit:
#endif

#ifndef DEBUG
	
	//int yolo = mkfifo("/tmp/yoloPipe", S_IRUSR);
	int yolo = open("/tmp/yoloPipe", 0666);

	char buffer[128];
	memset(buffer, sizeof(char), 128 * sizeof(char));
        pca9685->setPWM(ESC_CHANNEL,0, PWM_NEUTRAL + 150);
	
	while(getkey() != 27){
		if (read(yolo, buffer, 128) > 0 ){
			// buffer -> 9985 ( means 0.9985)
			// 0.5 -> middle of ZED Cam.
			double pos = 0.75 - (atoi(buffer) / 10000.0); 
			int PWM = 0;
			if (pos < 0)
				PWM = PWM_NEUTRAL * pos;
			else
				PWM = PWM_NEUTRAL * (1 + pos);
			printf("XPOS : %lf, PWM: %lf\n", pos, PWM);

			if (PWM >= PWM_FULL_FORWARD) 
				PWM = PWM_FULL_FORWARD;
			else if (PWM <= PWM_FULL_REVERSE) 
				PWM = PWM_FULL_REVERSE;
	
			pca9685->setPWM(SERVO_CHANNEL, 0, PWM);
			sleep(0.8);
		}
	}
#endif

    sleep(1);
    pca9685->setPWM(SERVO_CHANNEL, 0, PWM_NEUTRAL);
    pca9685->setPWM(ESC_CHANNEL, 0, PWM_NEUTRAL);
    pca9685->closePCA9685();
    }
}
