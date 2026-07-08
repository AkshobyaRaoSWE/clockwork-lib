#pragma once

#include "pros/misc.hpp" // pros::Controller
#include <functional>
#include <string>
#include <vector>

namespace clockwork {

/**
 * @brief A no-frills autonomous selector you drive from the controller.
 *
 * You register your autonomous routines by name, then let the driver scroll
 * through them with the controller D-pad before the match. The current pick
 * shows up on the controller screen and the brain screen. When autonomous
 * starts, you call run() and it runs whichever routine was selected.
 *
 * It needs nothing but core PROS (no liblvgl / LLEMU), so it works on any
 * project. The routines are plain `void()` functions, so anything you can call
 * you can register (a lambda, a free function, a method bound with std::bind).
 *
 * Typical wiring:
 * @code
 * clockwork::AutonSelector selector(&controller);
 *
 * void initialize() {
 *     selector.add("Left side rush",  left_rush);
 *     selector.add("Right side safe", right_safe);
 *     selector.add("Do nothing",      [] {});
 *     selector.draw();
 * }
 *
 * void competition_initialize() {
 *     // Let the driver pick while we wait for the match to start.
 *     while (true) { selector.poll(); pros::delay(20); }
 * }
 *
 * void autonomous() { selector.run(); }
 * @endcode
 */
class AutonSelector {
public:
    /**
     * @brief Build a selector.
     * @param controller the controller to read the D-pad from and print to.
     *                   May be null if you only want to drive it in code.
     */
    explicit AutonSelector(pros::Controller* controller = nullptr);

    /// Register a routine under a display @p name. First one added is selected.
    void add(const std::string& name, std::function<void()> routine);

    /// Move the selection forward one (wraps around) and redraw.
    void next();
    /// Move the selection back one (wraps around) and redraw.
    void prev();
    /// Select by list position (clamped to a valid index) and redraw.
    void select(int index);
    /// Select by name. Returns true if a routine with that name existed.
    bool select(const std::string& name);

    /// Index of the current selection, or -1 if nothing is registered.
    int index() const;
    /// How many routines are registered.
    int count() const;
    /// Name of the current selection ("None" if nothing is registered).
    const std::string& name() const;

    /// Run the selected routine. Call this from autonomous(). No-op if empty.
    void run();

    /**
     * @brief One iteration of the interactive picker. Call it in a loop.
     *
     * Reads the controller D-pad: RIGHT/DOWN move to the next routine, LEFT/UP
     * move to the previous one, and it redraws on any change. Does nothing if no
     * controller was given. Put it in a loop in competition_initialize() (or
     * disabled()) with a short delay so the driver can pick before the match.
     */
    void poll();

    /// Push the current selection to the controller and brain screens.
    void draw();

private:
    pros::Controller* m_controller;
    std::vector<std::string> m_names;
    std::vector<std::function<void()>> m_routines;
    int m_index;
};

} // namespace clockwork
