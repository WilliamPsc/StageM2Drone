/*! @file drone/main.cpp
 *  @version 1
 *  @date Jun 04 2021
 *
 *  @brief
 *  main for Telemetry API usage in a Linux environment.
 *  Shows example usage of the new data subscription API.
 *
 */

#include "drone_telemetry.hpp"
#include "include/Bancroft.hpp"
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

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace gpstk;

const char *portName = "/dev/ttyACM0";
unsigned char printlnWrite[] = {'\r'};
unsigned char commandWrite[] = {'l', 'e', 'p', '\r'};
struct timespec timePos, timePos2;
struct timespec timeMes, timeMes2;

void position(int fdDecawave)
{
	int err = 0;
	int n = 0;
	int val;
	double dwm0, dwm1, dwm2, dwm3, dwm4, dwm5, dwm6;
	int row = 4;
	int col = 4;
	int mult = 1;
	double posDroneX = 0.76;
	double posDroneY = 0.43;
	double posDroneZ = 0.26;
	double rsmeBancroft = 1000;
	double rsmeDecawave = 1000;
	int debut = 0;
	double avrgError = 0;
	int msr = 0;

	Matrix<double> matriceInput(row, col);
	Vector<double> result(col);
	Bancroft banc;

	n = write(fdDecawave, commandWrite, sizeof(commandWrite));
	if (n < 0)
		fputs("write() avec 4 bytes a échoué !\n", stderr);
	else
		printf("commandWrite() successful !\n");

	while (1)
	{
		auto start = std::chrono::steady_clock::now();
		char bufferDecawave[SIZE] = "";
		memset(&bufferDecawave, '\0', sizeof(bufferDecawave));

		dwm0 = 0.77 * mult;
		// dwm1 = 1.20 * mult;
		// dwm2 = 1.40 * mult;
		// dwm3 = 1.52 * mult;
		dwm4 = 0.93 * mult;
		dwm5 = 0.90 * mult;
		dwm6 = 0.75 * mult;

		double matrice[row][col] = {
			{0 * mult, 0 * mult, 0 * mult, dwm0},			// CD24
			// {0.48 * mult, -0.62 * mult, 0.76 * mult, dwm1}, // 0024
			// {1.01 * mult, -0.83 * mult, 0.42 * mult, dwm2}, // DD23
			// {1.48 * mult, -0.73 * mult, 0.01 * mult, dwm3}, // 09AE
			{1.67 * mult, 0.00 * mult, 0.03 * mult, dwm4},	// CA10
			{1.45 * mult, 1.04 * mult, 0.07 * mult, dwm5},	// DC30
			{0.09 * mult, 0.86 * mult, 0.04 * mult, dwm6}	// DA29
		};

		for (int i = 0; i < row; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				matriceInput(i, j) = matrice[i][j];
			}
		}

		n = read(fdDecawave, &bufferDecawave, sizeof(bufferDecawave));

		if (n < 0)
		{
			printf("Error reading : %s\n", strerror(errno));
			if (n == -1)
				err++;

			if (err == 4)
			{
				n = write(fdDecawave, commandWrite, sizeof(commandWrite));
				if (n < 0)
					fputs("write() avec 4 bytes a échoué !\n", stderr);
				else
					printf("commandWrite() successful !\n");
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
			ptr = strtok(bufferDecawave, ",\n\r\0");
			while (ptr != NULL)
			{
				decawavePos << ptr;
				ptr = strtok(NULL, ",\n\r\0");
			}
		}

		val = banc.Compute(matriceInput, result);

		rsmeBancroft = sqrt((pow(result[0] - posDroneX, 2) + pow(result[1] - posDroneY, 2) + pow(result[2] - posDroneZ, 2)) / 3);

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
					std::cout << "X = " << decawavePos[debut+1] << " Y = " << decawavePos[debut+2] << " Z = " << decawavePos[debut+3] << std::endl;
					rsmeDecawave = sqrt((pow(std::stod(decawavePos[debut + 1]) - posDroneX, 2) + pow(std::stod(decawavePos[debut + 2]) - posDroneY, 2) + pow(std::stod(decawavePos[debut + 3]) - posDroneZ, 2)) / 3);
					avrgError += rsmeDecawave;
					msr++;
				}
			}
			catch (std::exception const &e)
			{
				std::cerr << "Exception: " << e.what() << '\n';
				rsmeDecawave = 1000;
			}
		}

		auto end = std::chrono::steady_clock::now();

		// std::cout << "Vecteur résultat : " << result << std::endl; // Vecteur résultat Bancroft
		std::cout << "Erreur RSME Bancroft = " << rsmeBancroft << std::endl;
		std::cout << "Erreur RSME Decawave = " << rsmeDecawave << std::endl;
		if (msr == 1000)
		{
			std::cout << "Erreur moyenne Decawave : " << avrgError / msr << std::endl;
			exit(0);
		}

		long timeFunc =  std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
		std::cout << "Elapsed time in microseconds: "
				  << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()
				  << " ns" << std::endl;


		long timePosition = 100000000 - timeFunc; // 100 ms
		timePos.tv_nsec = timePosition;
		nanosleep(&timePos, &timePos2);
	}
}

void mesure(Vehicle *vehicle)
{
	while (1)
	{
		getBroadcastData(vehicle);
		nanosleep(&timeMes, &timeMes2);
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
	/*LinuxSetup linuxEnvironment(argc, argv);
	Vehicle *vehicle = linuxEnvironment.getVehicle();
	if (vehicle == NULL)
	{
		std::cout << "Vehicle not initialized, exiting.\n";
		return -1;
	}*/

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

	thrPosition = std::thread(position, fdDecawave);
	//thrMesure = std::thread(mesure, vehicle);

	thrPosition.join();
	//thrMesure.join();

	std::cout << "Programme fin !\n";
	close(fdDecawave);
	return 0;
}
