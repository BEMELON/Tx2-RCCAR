all: PCA9685.o main.o
	g++ main.o PCA9685.o -o main

PCA9685.o:
	g++ -c ../include/PCA9685/JHPWMPCA9685.cpp -I ../include/PCA9685 -o ./PCA9685.o

main.o: main.cpp
	g++ -c main.cpp PCA9685.o 
