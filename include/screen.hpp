#pragma once

#include "fmt/format.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace screen {

/**
 * @brief Identifies a bottom button returned by waitForBottomButtonTap().
 *
 * @c NONE means no button was selected before the timeout expired.
 */
enum class ButtonId { NONE = -1, LEFT = 0, MIDDLE = 1, RIGHT = 2 };

/**
 * @brief Provides a simple Brain-screen layout with scrolling text and three
 * touchable bottom buttons.
 *
 * The manager reserves the space above @ref BTN_Y for text and draws three
 * unlabeled capsule buttons below it. Text is organized into independent
 * scrollback buffers: repeated calls with the same @c x and @c y append rows
 * to the same buffer, while another origin creates a separate buffer.
 *
 * @note A Manager only tracks its own drawing. Calling raw
 * pros::screen functions on the same area can overwrite its text or buttons.
 * Call clearScreen() or printToScreen(true, ...) before changing screens.
 *
 * @code
 * screen::Manager ui;
 * ui.drawBottomButtons();
 * ui.printToScreen(false, 12, 12, "Ready: {}", true);
 *
 * if (ui.waitForBottomButtonTap(3000) == screen::ButtonId::LEFT) {
 *   ui.printToScreen(false, 12, 12, "Left button selected");
 * }
 * @endcode
 */
class Manager {
public:
  // --- Configuration Constants ---

  /// Full physical width of the V5 Brain display, in pixels.
  inline static constexpr int SCREEN_WIDTH = 480;
  /// Full physical height of the V5 Brain display, in pixels.
  inline static constexpr int SCREEN_HEIGHT = 272;
  /// Height of each bottom button, in pixels.
  inline static constexpr int BTN_HEIGHT = 40;
  /// Width of each bottom button, in pixels.
  inline static constexpr int BTN_WIDTH = 100;
  /// Y-coordinate of the top edge of the bottom-button row.
  inline static constexpr int BTN_Y = 190;
  /// Exclusive lower Y-bound for text, leaving the button row unobstructed.
  inline static constexpr int TEXT_AREA_BOTTOM = BTN_Y;
  /// Extra touch area around each button, in pixels.
  inline static constexpr int PADDING = 5;
  /// Approximate width of a small-font character, used for line wrapping.
  static constexpr int CHAR_WIDTH = 8;
  /// Height of a small-font character, used to keep text above the buttons.
  static constexpr int CHAR_HEIGHT = 12;

  // UI Styling

  /// Fill color of the bottom buttons (dark gray).
  inline static constexpr uint32_t COLOR_BTN_FILL = 0x3D3D3D;
  /// Outline color of the bottom buttons (light gray).
  inline static constexpr uint32_t COLOR_BTN_BORDER = 0x707070;
  /// Maximum rows retained by one text buffer, even if more space is free.
  inline static constexpr int MAX_ROWS = 8;
  /// Distance between the top edges of consecutive text rows, in pixels.
  inline static constexpr int ROW_HEIGHT = 20;

  /// Creates an empty screen manager; no screen pixels are changed.
  Manager() = default;

  /**
   * @brief RAII wrapper for PROS mutexes.
   * @details Automatically takes the mutex on construction and gives it on
   * destruction. This is exposed for compatibility, but Manager performs its
   * own synchronization; application code normally does not need this type.
   */
  struct MutexGuard {
    pros::Mutex &m;
    bool locked = false;

    explicit inline MutexGuard(pros::Mutex &m) : m(m) { locked = m.take(); }
    explicit inline MutexGuard(pros::Mutex &m, uint32_t timeout) : m(m) { locked = m.take(timeout); }

    ~MutexGuard() { if (locked) m.give(); }

    bool isLocked() const { return locked; }
    MutexGuard(const MutexGuard &) = delete;
    MutexGuard &operator=(const MutexGuard &) = delete;
  };

  /**
   * @brief Draws the three unlabeled capsule buttons at the bottom of the
   * screen.
   *
   * Use waitForBottomButtonTap() to learn which button the user tapped. This
   * method draws only the button shapes; add labels with pros::screen::print()
   * if your UI needs them.
   *
   * @param redraw If @c true (the default), clears the whole display and all
   * tracked text before drawing. Pass @c false to add or refresh buttons while
   * keeping existing contents outside the button area.
   */
  void drawBottomButtons(bool redraw = true);

  /**
   * @brief Waits for a tap-and-release inside a bottom button.
   *
   * A tap within @ref PADDING pixels of a button is accepted. The method does
   * not require drawBottomButtons() to have been called, although callers
   * normally draw the buttons before waiting for input.
   *
   * @param timeout Maximum wait in milliseconds. Pass @c 0 to return
   * immediately, or use the default @c TIMEOUT_MAX to wait indefinitely.
   * @return The @ref ButtonId of the tapped button, or @c ButtonId::NONE if
   * timed out.
   */
  ButtonId waitForBottomButtonTap(uint32_t timeout = TIMEOUT_MAX);

  /**
   * @brief Waits for a screen touch event anywhere on the display.
   *
   * @param timeoutMs Maximum wait in milliseconds. Pass @c 0 to return
   * immediately, or use the default @c TIMEOUT_MAX to wait indefinitely.
   * @param detectTouchOnly If @c true, returns as soon as the screen is
   * pressed or held. If @c false, waits until a press is released, which is
   * useful for treating a touch as a complete tap.
   * @return @c true if the requested touch event occurs before the timeout;
   * @c false otherwise.
   * @example
   * // Wait 2 seconds for a user to tap the screen
   * if (waitForScreenTouch(2000)) startAuton();
   * @endcode
   */
  bool waitForScreenTouch(uint32_t timeoutMs = TIMEOUT_MAX,
                          bool detectTouchOnly = false);

  /**
   * @brief Clears the complete Brain display and resets every text buffer.
   *
   * This also removes buttons and any other graphics drawn on the screen.
   */
  void clearScreen();

  /**
   * @brief Formats and displays scrolling text on the Robot Brain.
   * @details Each @p x/@p y origin owns an independent scrolling buffer. Text
   * printed at another origin therefore never moves an existing buffer. A
   * buffer retains the most recent rows that fit in the text area above the
   * bottom buttons, up to @ref MAX_ROWS. Each repeated call at the same origin
   * appends a row. Newlines create rows, long rows wrap automatically, and an
   * empty string creates a blank row.
   *
   * Text uses the small PROS font. The first row is placed exactly at
   * @p x/@p y and later rows are @ref ROW_HEIGHT pixels lower. Origins that
   * cannot fit at least one small-font row in the text area are ignored.
   *
   * Formatting follows {fmt} syntax, not printf syntax: use @c "{}" or
   * @c "{:.1f}", rather than @c "%d" or @c "%.1f". To print a literal brace,
   * use @c "{{" or @c "}}".
   * @param clear If @c true, clears the entire screen and every text buffer
   * before printing.
   * @param x The left edge of this buffer in pixels. Valid origins range from
   * @c 0 through @c SCREEN_WIDTH - CHAR_WIDTH. Text that does not fit is
   * wrapped; origins too close to the right edge are ignored.
   * @param y The top edge of this buffer in pixels. Valid origins range from
   * @c 0 through @c TEXT_AREA_BOTTOM - CHAR_HEIGHT, so each row remains above
   * the buttons.
   * @param format A {fmt}-style format string.
   * @param args Arguments for the format string.
   * @example
   * // Rows appear at (100, 100) and (100, 120).
   * printToScreen(false, 100, 100, "First row");
   * printToScreen(false, 100, 100, "Second row");
   *
   * // This uses a separate buffer and does not move either row above.
   * printToScreen(false, 0, 100, "Status: {:.1f}", 10.5);
   * printToScreen(false, 0, 0, "Chassis: Position: {:.1f}", 10.5);
   * @endcode
   */
  template <typename... Args>
  void printToScreen(bool clear, uint16_t x, uint16_t y,
                     const char *format, Args &&...args) {
    MutexGuard m(sharedMutex);
    if (!m.isLocked()) return;

    if (clear) clearScreenLocked();

    // There is no drawable text area at or past either boundary. Check before
    // subtracting so the wrapping calculation cannot underflow.
    if (format == nullptr || x >= SCREEN_WIDTH || y >= TEXT_AREA_BOTTOM)
      return;

    const std::size_t maxCharsPerLine =
        static_cast<std::size_t>(SCREEN_WIDTH - x) / CHAR_WIDTH;
    const std::size_t maxRows = rowsAvailable(y);
    if (maxCharsPerLine == 0 || maxRows == 0) return;

    const std::string formatted =
        fmt::format(fmt::runtime(format), std::forward<Args>(args)...);
    const TextOrigin origin{x, y};
    TextBuffer &buffer = textBuffers[origin];
    const std::size_t oldLineCount = buffer.lines.size();
    const std::size_t oldMaxLineLength = longestLineLength(buffer.lines);

    // Split at explicit newlines, retaining empty lines.
    std::vector<std::string> logicalLines;
    size_t pos = 0;
    size_t next;
    while ((next = formatted.find('\n', pos)) != std::string::npos) {
      logicalLines.push_back(formatted.substr(pos, next - pos));
      pos = next + 1;
    }
    logicalLines.push_back(formatted.substr(pos));

    for (const std::string &line : logicalLines) {
      if (line.empty()) {
        buffer.lines.emplace_back();
        continue;
      }

      size_t start = 0;
      while (start < line.size()) {
        const size_t len = std::min(maxCharsPerLine, line.size() - start);

        if (start + len >= line.size()) {
          buffer.lines.push_back(line.substr(start));
          break;
        }

        // Search only inside the current candidate range. The old code also
        // searched the following character, which allowed wrapped rows to
        // exceed their maximum width.
        const size_t splitPos = line.rfind(' ', start + len - 1);
        if (splitPos != std::string::npos && splitPos > start) {
          buffer.lines.push_back(line.substr(start, splitPos - start));
          start = splitPos + 1;
        } else {
          buffer.lines.push_back(line.substr(start, len));
          start += len;
        }
      }
    }

    if (buffer.lines.size() > maxRows)
      buffer.lines.erase(buffer.lines.begin(),
                         buffer.lines.begin() + (buffer.lines.size() - maxRows));

    buffer.lastUpdated = ++renderSequence;
    redrawBuffer(origin, oldLineCount, oldMaxLineLength);
  }

  /**
   * @brief Appends formatted text to the default buffer at pixel origin (0, 0).
   *
   * This overload is equivalent to @c printToScreen(false, 0, 0, format,
   * args...). Use the coordinate overload to create a buffer elsewhere.
   */
  template <typename... Args>
  void printToScreen(const char *format, Args &&...args) {
    // Convenience overload for a scrollback buffer at the top-left corner.
    printToScreen(false, 0, 0, format, std::forward<Args>(args)...);
  }

private:
  struct TextOrigin {
    uint16_t x;
    uint16_t y;

    bool operator<(const TextOrigin &other) const {
      return std::tie(x, y) < std::tie(other.x, other.y);
    }
  };

  struct TextBuffer {
    std::vector<std::string> lines;
    uint64_t lastUpdated = 0;
  };

  std::map<TextOrigin, TextBuffer> textBuffers;
  uint64_t renderSequence = 0;
  pros::Mutex sharedMutex;

  void clearScreenLocked();
  static std::size_t longestLineLength(const std::vector<std::string> &lines);
  static std::size_t rowsAvailable(uint16_t y);
  void redrawBuffer(const TextOrigin &origin, std::size_t oldLineCount,
                    std::size_t oldMaxLineLength);

  // Internal coordinate calculation
  inline static constexpr int getGap() {
    return (SCREEN_WIDTH - (3 * BTN_WIDTH)) / 4;
  }
  inline static constexpr int getX(ButtonId id) {
    return (getGap() * (static_cast<int>(id) + 1)) +
           (BTN_WIDTH * static_cast<int>(id));
  }

  /**
   * @brief Helper to draw a capsule (rounded rectangle) shape.
   * @details Used internally to render the buttons.
   */
  void drawCapsule(int x, int y, int width, int height, uint32_t fillColor,
                   uint32_t borderColor);

  /// Checks if a coordinate pair lies within a specified bounding box
  bool isInside(int px, int py, int bx, int by, int w, int h);
};

} // namespace screen
