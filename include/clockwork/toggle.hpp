#pragma once

namespace clockwork {

// Flips a boolean every time a button goes from released to pressed.
//
// Feed it the button state each loop and it does the edge detection for you, so
// one press toggles once instead of flickering the whole time you hold it. Handy
// for latching things on a single button: a pneumatic clamp, a speed mode, a
// wing. It's just a bool and a bit of memory, so keep one around per button.
class Toggle {
public:
    // Start the latch in a known state (default off).
    explicit Toggle(bool initial = false) : m_state(initial), m_prev(false) {}

    // Call this every loop with the button's current pressed state. It flips on
    // the press (the released-to-pressed edge) and hands back the latched value.
    bool update(bool pressed) {
        if (pressed && !m_prev) m_state = !m_state;
        m_prev = pressed;
        return m_state;
    }

    // The latched value, without touching it.
    bool state() const { return m_state; }

    // Force the latch to a value (e.g. reset a clamp to open at match start).
    void set(bool value) { m_state = value; }

private:
    bool m_state;
    bool m_prev;
};

} // namespace clockwork
