#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <SDL2/SDL.h>
#include <unistd.h>

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

/* make some talons for drive train */
TalonFX talLeft(2);

// [-1,1] Range
void drive(double fwd)
{
	talLeft.Set(ControlMode::PercentOutput, fwd);
}

/** simple wrapper for code cleanup */
void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {
	// Initialize CAN interface with 'can0'
	std::string interface;
	interface = "can0";
	ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

	// c_SetPhoenixDiagnosticsStartTime(-1);

	sleepApp(6000);

	// Initialize motor with 0 output
	ctre::phoenix::unmanaged::FeedEnable(10000);

	drive(0);
	printf("Motor connected!\n");

//// Begin countdown
//printf("Motor starting in...\n");
//sleepApp(1000);
//for (int i = 5; i > 0; i--) {
//	printf("%i...\n", i);
//	sleepApp(1000);
//}

	// Run motor
	drive(0.1);
	printf("Motor running for two seconds...\n");
	sleepApp(2000);

	// Motor off
	drive(0.0);
	printf("Motor off.\n");
	return 0;
}
