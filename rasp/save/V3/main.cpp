/*! @file drone/main.cpp
 *  @version 1
 *  @date Jun 11 2021
 *
 *  @brief
 *  main for Telemetry API usage in a Linux environment.
 *  Shows example usage of the new data subscription API.
 *
 */

#include "drone_telemetry.hpp"
#include "include/Matrix.hpp"
#include "include/Vector.hpp"

/**/
#include <stdio.h>	 /* Standard input/output definitions */
#include <string.h>	 /* String function definitions */
#include <unistd.h>	 /* UNIX standard function definitions */
#include <fcntl.h>	 /* File control definitions */
#include <errno.h>	 /* Error number definitions */
#include <time.h>	 /* Waiting time */
#include <termios.h> /* POSIX terminal control definitions */
#include <chrono>	 /* Allows to follow the program execution time*/
#include <cmath>	 /* Calcul de la racine carrée */
#include <string.h>	 /* Split de la valeur lue du decawave */
#include <string>	 /* Split de la valeur lue du decawave */

#define SIZE 23 // 23 (lep) ou 137 (les)
#define NBTOUR 10

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace gpstk;

const char *portName = "/dev/ttyACM0";
unsigned char printlnWrite[] = {'\r'};
unsigned char commandWrite[] = {'l', 'e', 'p', '\r'};
struct timespec timePos, timePos2;
struct timespec timeMes, timeMes2;
int stop = 1;
Vector<double> posDrone(3);
Vector<double> posCapteur(3);

double calculs(){
	// Calcul de la distance entre les deux points
	double distance = sqrt(pow(posCapteur[0] - posDrone[0], 2) + pow(posCapteur[1] - posDrone[1], 2) + pow(posCapteur[2] - posDrone[2], 2));
	std::cout << distance << std::endl;

	// Produit scalaire entre Drone et Capteur
	double pdtScalaire = 0.0;
	int i = 0;

	for(i = 0; i < 3; i++){
		pdtScalaire += posCapteur[i] * posDrone[i];
	}
	std::cout << pdtScalaire << std::endl;
	
	// Division du produit scalaire par la distance
	double division = pdtScalaire / distance;
	std::cout << division << std::endl;

	// Obtention du cosinus
	double cosinus = cos(division);
	std::cout << cosinus << std::endl;

	// Tourner vers la droite si Xcapteur - Xdrone est positif, sinon tourner vers la droite
	double direction =posCapteur[0] -posDrone[0];
	std::cout << direction << std::endl;

	return direction;
}


void errorReading(int fdDecawave, int n)
{
	n = write(fdDecawave, commandWrite, sizeof(commandWrite));
	if (n < 0)
		fputs("write() avec 4 bytes a échoué !\n", stderr);
	else
		printf("commandWrite() successful !\n");
}

void position(int fdDecawave)
{
	int err = 0;
	int n = 0;
	int debut = 0;
	int msr = 0;

	n = write(fdDecawave, commandWrite, sizeof(commandWrite));
	if (n < 0)
		fputs("write() avec 4 bytes a échoué !\n", stderr);
	else
		printf("commandWrite() successful !\n");

	while (stop)
	{
		auto start = std::chrono::steady_clock::now();
		char bufferDecawave[SIZE] = "";
		memset(&bufferDecawave, '\0', sizeof(bufferDecawave));

		n = read(fdDecawave, &bufferDecawave, sizeof(bufferDecawave));

		if (n < 0)
		{
			printf("Error reading : %s\n", strerror(errno));
			if (n == -1)
				err++;

			if (err == 4)
			{
				errorReading(fdDecawave, n);
			}
		}
		else
		{
			// std::cout << "Read " << n << " bytes.\n\'" << bufferDecawave << "\'\n\n";
		}

		size_t size = sizeof(bufferDecawave) / sizeof(bufferDecawave[0]);
		Vector<std::string> decawavePos(5);
		char *ptr;
		if (size > 0)
		{
			ptr = strtok(bufferDecawave, ", \n\r\0");
			while (ptr != NULL)
			{
				// std::cout << ptr << std::endl;
				decawavePos << ptr;
				ptr = strtok(NULL, ", \n\r\0");
			}
		}

		for (int i = 0; i < decawavePos.size(); i++)
		{
			if (decawavePos[i] == "POS")
			{
				debut = i;
				break;
			}
			else
			{
				debut = 0;
			}
		}

		if (debut != 0)
		{
			try
			{
				if ((decawavePos.size() - debut) > 3)
				{
					posDrone[0]= std::stod(decawavePos[debut + 1]);
					posDrone[1]= std::stod(decawavePos[debut + 2]);
					posDrone[2]= std::stod(decawavePos[debut + 3]);

					calculs();
					std::cout << "X = " << posDrone[0] << " Y = " << posDrone[1] << " Z = " << posDrone[2] << std::endl; // Valeur résultat Decawave

					msr++;
				}
			}
			catch (std::exception const &e)
			{
				std::cerr << "Exception: " << e.what() << '\n';
			}
		}

		if (msr == NBTOUR)
		{
			stop = 0;
		}

		auto end = std::chrono::steady_clock::now();

		long timeFunc = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
		std::cout << "Temps d'exécution position : " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " µs" << std::endl;

		long timePosition = 100000000 - timeFunc; // 100 ms
		timePos.tv_nsec = timePosition;
		nanosleep(&timePos, &timePos2);
	}
}

void mesure(Vehicle *vehicle)
{
	while (stop)
	{
		auto start = std::chrono::steady_clock::now();
		
		getBroadcastData(vehicle);
		
		auto end = std::chrono::steady_clock::now();
		
		long timeFunc = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
		long timePosition = 100000000 - timeFunc; // 100 ms
		timeMes.tv_nsec = timePosition;

		nanosleep(&timeMes, &timeMes2);

		std::cout << "Temps d'exécution mesure : " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " µs" << std::endl;
	}
}

int main(int argc, char **argv)
{
	std::cout << "Programme début !\n";

	std::thread thrPosition;
	std::thread thrMesure;

	timePos.tv_sec = 0;
	timePos.tv_nsec = 100000000L; // 100ms
	timeMes.tv_sec = 0;
	timeMes.tv_nsec = 100000000L; // 100ms

	// Setup OSDK.
	LinuxSetup linuxEnvironment(argc, argv);
	Vehicle *vehicle = linuxEnvironment.getVehicle();
	if (vehicle == NULL)
	{
		std::cout << "Vehicle not initialized, exiting.\n";
		return -1;
	}

	int fdDecawave; /* File descriptor for the port */
	struct termios tty;

	fdDecawave = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fdDecawave < 0)
	{
		perror("Error : Unable to open /dev/ttyACMO\n");
		exit(1);
	}

	if (tcgetattr(fdDecawave, &tty) != 0)
	{
		perror("Error : tcgetattr\n");
		exit(1);
	}

	tty.c_cflag &= ~PARENB;		   // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB;		   // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE;		   // Clear all the size bits, then use one of the statements below
	tty.c_cflag |= CS8;			   // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS;	   // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO;	// Disable echo
	tty.c_lflag &= ~ECHOE;	// Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG;	// Disable interpretation of INTR, QUIT and SUSP

	tty.c_iflag &= ~(IXON | IXOFF | IXANY);										 // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

	//tty.c_oflag &= OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	//tty.c_oflag &= ONLCR; // Prevent conversion of newline to carriage return/line feed

	tty.c_cc[VTIME] = 10; // Wait for up to 10s (100 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = SIZE;

	cfsetspeed(&tty, B115200);

	// Save tty settings, also checking for error
	if (tcsetattr(fdDecawave, TCSANOW, &tty) != 0)
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));

	int n = write(fdDecawave, printlnWrite, sizeof(printlnWrite));
	if (n < 0)
		fputs("printlnWrite() a échoué !\n", stderr);
	else
		printf("printlnWrite() successful !\n");

	n = write(fdDecawave, printlnWrite, sizeof(printlnWrite));
	if (n < 0)
		fputs("printlnWrite() a échoué !\n", stderr);
	else
		printf("printlnWrite() successful !\n");

	sleep(1);

	posCapteur[0] = 4; posCapteur[1] = 6.5; posCapteur[2] = 1.2;

	thrPosition = std::thread(position, fdDecawave);
	thrMesure = std::thread(mesure, vehicle);

	thrPosition.join();
	thrMesure.join();

	std::cout << "Programme fin !\n";
	close(fdDecawave);
	return 0;
}
