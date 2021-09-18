/*! @file drone_telemetry.hpp
 *  @version 3.3
 *  @date May 06 2021
 *
 *  @brief
 *  Telemetry API usage in a Linux environment.
 *  Shows example usage of the new data subscription API.
 *
 */

#ifndef DJIOSDK_DRONETELEMETRY_HPP
#define DJIOSDK_DRONETELEMETRY_HPP

// System Includes
#include <iostream>
#include <thread>

// DJI OSDK includes
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

bool subscribeToData(DJI::OSDK::Vehicle* vehiclePtr, int responseTimeout = 1);
bool subscribeToDataForInteractivePrint(DJI::OSDK::Vehicle* vehiclePtr, int responseTimeout = 1);
bool subscribeToDataAndSaveLogToFile(DJI::OSDK::Vehicle* vehiclePtr, int responseTimeout = 1);

// Broadcast data implementation for Matrice 100
bool getBroadcastData(DJI::OSDK::Vehicle* vehicle, int responseTimeout = 1);
#endif // DJIOSDK_DRONETELEMETRY_HPP
