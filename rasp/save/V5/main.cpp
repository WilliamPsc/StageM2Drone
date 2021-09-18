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
#include "flight_control_sample.hpp"
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
#include <mutex>	 /* Mutex */

#define SIZE 25
#define NBTOUR 1000
#define M_PI 3.14159265358979323846

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace gpstk;

const char *portName = "/dev/ttyACM0";
unsigned char printlnWrite[] = {'\r'};
unsigned char commandWrite[] = {'l', 'e', 'p', '\r'};
struct timespec timePos, timePos2;
struct timespec timeMes, timeMes2;
int stop = 1;
int fdDecawave;
Vector<double> posDrone(3);
Vector<double> posCapteur(3);
double angle = 0.0;
double angleReel = 0.0;
std::mutex mux;

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calculs(Vehicle *vehicle)
{

	std::cout << "----------------------------------------" << std::endl;
	mux.lock();
	std::cout << "Valeur d'angle donnée par drone : " << angle << std::endl;

	if (angle > 0.0 && angle <= 0.14) // 0 - 22.5
	{
		angleReel = map(angle, 0.0, 0.14, 0, 22.5);
	}
	else if (angle > 0.14 && angle <= 0.33)  // 22.5 - 45
	{
		angleReel = map(angle, 0.14, 0.33, 22.5, 45); 
	}
	else if (angle > 0.33 && angle <= 0.46)  // 45 - 67.5
	{
		angleReel = map(angle, 0.33, 0.46, 45, 67.5); 
	}
	else if (angle > 0.46 && angle <= 0.64)  // 67.5 - 90
	{
		angleReel = map(angle, 0.46, 0.64, 67.5, 90); 
	}
	else if (angle > 0.64 && angle <= 0.90) // 90 - 135
	{
		angleReel = map(angle, 0.64, 0.90, 90, 135);
	}
	else if (angle > 0.90 && angle <= 1.0) // 135 - 180
	{
		angleReel = map(angle, 0.90, 1.0, 135, 180);
	}
	else if(angle > -1.0 && angle <= -0.964) { // Sud - Sud/Ouest
		angleReel = map(angle, -1.0, -0.964, 180, 225);
	}
	else if(angle > -0.964 && angle <= -0.86){ // Sud/Ouest - Ouest
		angleReel = map(angle, -0.964, -0.86, 225, 270);
	}
	else if(angle > -0.86 && angle <= -0.44){  // Ouest - Nord/Ouest
		angleReel = map(angle, -0.86, -0.44, 270, 315);
	}
	else if(angle > -0.44 && angle <= -0.0){ // Nord/Ouest - Nord
		angleReel = map(angle, -0.44, -0.0, 315, 360);
	}
	else
	{
		std::cout << "Erreur " << angle << std::endl;
	}
	std::cout << "Angle calculé par rapport au Nord : " << angleReel << " °" << std::endl;

	angleReel -= 120.0;
	std::cout << "Angle calculé par rapport à l'axe des X : " << angleReel << " °" << std::endl;
	mux.unlock();

	// Calcul de la distance entre les deux points
	double distance = sqrt(pow(posCapteur[0] - posDrone[0], 2) + pow(posCapteur[1] - posDrone[1], 2));

	std::cout << "Position du capteur : " << posCapteur[0] << "," << posCapteur[1] << "," << posCapteur[2] << std::endl;
	std::cout << "Position du drone : " << posDrone[0] << "," << posDrone[1] << "," << posDrone[2] << std::endl;
	std::cout << "Distance : " << distance << std::endl;
	std::cout << "Distance en X à parcourir : " << posCapteur[0] - posDrone[0] << std::endl;
	std::cout << "Distance en Y à parcourir : " << posCapteur[1] - posDrone[1] << std::endl;

	// Obtention du cosinus / sinus
	double cosinus = cos(posCapteur[0]-posDrone[0]) * (180 / M_PI);
	std::cout << "Cosinus : " << cosinus << std::endl;

	double sinus = sin(posCapteur[1]-posDrone[1]) * (180 / M_PI);
	std::cout << "Sinus : " << sinus << std::endl;

	double angleFinal = sinus - angleReel;
	std::cout << "Angle nécessaire : " << angleFinal << " °" << std::endl;

	// Mouvements vers le capteur
	// monitoredTakeoff(vehicle);
	// moveByPositionOffset(vehicle, 0, 0, posCapteur[2], angleReel); // Mouvement en Z
	// moveByPositionOffset(vehicle, 0.10, 0.10, 0, 0); // Mouvement en XY
	// monitoredLanding(vehicle);
	
	std::cout << "----------------------------------------\n" << std::endl;

	// Retour à la base et confirme l'atterissage
	// goHomeAndConfirmLanding(vehicle, 1);
}

void errorReading(int fdDecawave, int n)
{
	n = write(fdDecawave, commandWrite, sizeof(commandWrite));
	if (n < 0)
		fputs("write() avec 4 bytes a échoué !\n", stderr);
	else
		printf("commandWrite() successful !\n");
}

void position(Vehicle *vehicle)
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

		size_t size = sizeof(bufferDecawave) / sizeof(bufferDecawave[0]);
		Vector<std::string> decawavePos(5);
		char *ptr;
		if (size > 0)
		{
			ptr = strtok(bufferDecawave, ", \n\r\0");
			while (ptr != NULL)
			{
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
					posDrone[0] = std::stod(decawavePos[debut + 1]);
					posDrone[1] = std::stod(decawavePos[debut + 2]);
					posDrone[2] = std::stod(decawavePos[debut + 3]);

					calculs(vehicle);
					std::cout << "X = " << posDrone[0] << " Y = " << posDrone[1] << " Z = " << posDrone[2] << std::endl; // Valeur résultat Decawave

					msr++;
				}
			}
			catch (std::exception const &e)
			{
				std::cerr << "Exception: " << e.what() << '\n';
			}
		}

		// if (msr == NBTOUR)
		// {
		// 	stop = 0;
		// }

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

		Telemetry::Status status;
		Telemetry::Quaternion quaternion;

		const int TIMEOUT = 20;

		// Re-set Broadcast frequencies to their default values
		ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreqDefaults(TIMEOUT);

		status = vehicle->broadcast->getStatus();
		quaternion = vehicle->broadcast->getQuaternion();

		// std::cout << "-------\n";
		// std::cout << "Flight Status                         = "
		//       << (unsigned)status.flight << "\n";
		// std::cout << "Quaternion Info = "
		// 		<< quaternion.q0 << ", " << quaternion.q1 << ", " << quaternion.q2 << ", " << quaternion.q3 << "\n";
		// std::cout << "-------\n\n";

		mux.lock();
		angle = quaternion.q3;
		mux.unlock();

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
	timePos.tv_nsec = 0;
	timeMes.tv_sec = 0;
	timeMes.tv_nsec = 0;

	// Setup OSDK.
	LinuxSetup linuxEnvironment(argc, argv);
	Vehicle *vehicle = linuxEnvironment.getVehicle();
	if (vehicle == NULL)
	{
		std::cout << "Vehicle not initialized, exiting.\n";
		return -1;
	}

	int functionTimeout = 1;
	vehicle->obtainCtrlAuthority(functionTimeout);

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


	posCapteur[0] = 2.14; posCapteur[1] = 2.55; posCapteur[2] = 1.00;

	thrPosition = std::thread(position, vehicle);
	thrMesure = std::thread(mesure, vehicle);

	thrPosition.join();
	thrMesure.join();

	std::cout << "Programme fin !\n";
	close(fdDecawave);
	return 0;
}
