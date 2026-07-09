#pragma once

#include "pros/adi.hpp"
#include <cstdint>

namespace clockwork {

// A single pneumatic piston, the kind you use for a clamp, a set of wings, or a
// tilt. It wraps an ADI digital-out solenoid and remembers whether it's out, so
// you can toggle it and read its state instead of tracking a loose bool yourself.
// It reads a lot better than set_value(true)/set_value(false) sprinkled around.
class Pneumatics {
public:
    // port is an ADI port on the brain: a number 1-8, or a letter 'A'-'H'.
    // Pass startExtended if the piston should begin pushed out.
    explicit Pneumatics(std::uint8_t port, bool startExtended = false);

    void extend();       // push the piston out
    void retract();      // pull it back in
    void set(bool out);  // extend when true, retract when false
    void toggle();       // flip to whichever it isn't right now

    // True if the piston is currently extended.
    bool extended() const;

private:
    pros::adi::DigitalOut m_piston;
    bool m_extended;
};

} // namespace clockwork
