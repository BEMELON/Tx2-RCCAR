all: PCA9685.o main.o
	g++ main.o PCA9685.o -o main

PCA9685.o:
	g++ -c ../PCA9685/JHPWMPCA9685.cpp -I ../PCA9685 -o ./PCA9685.o

main.o: main.cpp
	g++ -c main.cpp
