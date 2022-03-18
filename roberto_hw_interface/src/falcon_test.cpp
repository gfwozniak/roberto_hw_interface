#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

/* make some talons for drive train */
std::string interface = "can0";
TalonSRX talLeft(1, interface); //Use the specified interface
TalonSRX talRght(0); //Use the default interface (can0)

/** simple wrapper for code cleanup */
void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() 
{	
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);
	talLeft.Set(ControlMode::PercentOutput, .5);
	sleepApp(10000);
	talLeft.Set(ControlMode::PercentOutput, 0);

	return 0;
}
