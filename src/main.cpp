#include "main.h"
#include "clockwork/clockwork.hpp"

// Demo entry points for building/testing the CLOCKWORK library in isolation.
// This file is excluded from the shipped template (see EXCLUDE_SRC_FROM_LIB in
// the Makefile), so none of it reaches consumer projects. See README.md for
// real usage in your own robot project.

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// Example usage (uncomment with your own configured chassis):
	//
	//   clockwork::Motion motion(&chassis);
	//   chassis.setPose(0, 0, 0);
	//   motion.driveFullThenSlow(24, 12, 40, 3000);
}

void opcontrol() {}
