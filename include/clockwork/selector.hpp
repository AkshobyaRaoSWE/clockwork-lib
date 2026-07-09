#pragma once

#include "pros/misc.hpp" // pros::Controller
#include <functional>
#include <string>
#include <vector>

namespace clockwork {

// A no-frills autonomous selector you drive from the controller. You register
// your routines by name, the driver scrolls through them with the D-pad before
// the match, and the current pick shows on both the controller and brain
// screens. When auton starts you call run() and it fires whichever one was
// selected.
//
// It needs nothing but core PROS (no liblvgl / LLEMU), so it drops into any
// project. Routines are plain void() functions, so anything callable works: a
// free function, a lambda, a method bound with std::bind.
//
// The usual wiring:
//
//   clockwork::AutonSelector selector(&controller);
//
//   void initialize() {
//       selector.add("Left side rush",  left_rush);
//       selector.add("Right side safe", right_safe);
//       selector.add("Do nothing",      [] {});
//       selector.draw();
//   }
//
//   void competition_initialize() {
//       // let the driver pick while we wait for the match to start
//       while (true) { selector.poll(); pros::delay(20); }
//   }
//
//   void autonomous() { selector.run(); }
class AutonSelector {
public:
    // controller is what it reads the D-pad from and prints to. Pass null if you
    // only ever set the selection in code.
    explicit AutonSelector(pros::Controller* controller = nullptr);

    // Register a routine under a display name. The first one you add is selected.
    void add(const std::string& name, std::function<void()> routine);

    void next();              // forward one, wraps around, redraws
    void prev();              // back one, wraps around, redraws
    void select(int index);   // jump to a position (clamped), redraws
    bool select(const std::string& name); // jump by name; true if it existed

    int index() const;  // current position, or -1 if nothing is registered
    int count() const;  // how many routines are registered
    const std::string& name() const; // current name ("None" if empty)

    // Run the selected routine. Call this from autonomous(). Does nothing if the
    // list is empty.
    void run();

    // One pass of the interactive picker; call it in a loop. RIGHT/DOWN on the
    // D-pad move to the next routine, LEFT/UP to the previous, and it redraws on
    // a change. Does nothing without a controller. Drop it in a loop in
    // competition_initialize() (or disabled()) with a short delay so the driver
    // can pick before the match.
    void poll();

    // Push the current pick to the controller and brain screens.
    void draw();

private:
    pros::Controller* m_controller;
    std::vector<std::string> m_names;
    std::vector<std::function<void()>> m_routines;
    int m_index;
};

} // namespace clockwork
