#include "screen.hpp" // parent header

namespace screen {

void Manager::clearScreen() {
  MutexGuard m(sharedMutex);
  if (!m.isLocked()) return;

  clearScreenLocked();
}

void Manager::clearScreenLocked() {
  // Callers already own sharedMutex; taking it again would deadlock because
  // pros::Mutex is not recursive.
  textBuffers.clear();
  renderSequence = 0;
  pros::screen::erase();
}

std::size_t Manager::longestLineLength(
    const std::vector<std::string> &lines) {
  std::size_t longest = 0;
  for (const std::string &line : lines)
    longest = std::max(longest, line.size());
  return longest;
}

std::size_t Manager::rowsAvailable(uint16_t y) {
  // A row must fit fully above the button strip, and each buffer still honors
  // the fixed scrollback cap even when its origin is near the top.
  if (y >= TEXT_AREA_BOTTOM || y + CHAR_HEIGHT > TEXT_AREA_BOTTOM) return 0;

  return std::min<std::size_t>(
      MAX_ROWS, 1 + (TEXT_AREA_BOTTOM - y - CHAR_HEIGHT) / ROW_HEIGHT);
}

void Manager::redrawBuffer(const TextOrigin &origin,
                           std::size_t oldLineCount,
                           std::size_t oldMaxLineLength) {
  const TextBuffer &updatedBuffer = textBuffers.at(origin);
  const std::size_t currentLineCount = updatedBuffer.lines.size();
  const std::size_t currentMaxLineLength =
      longestLineLength(updatedBuffer.lines);

  // The first render has no stale pixels to remove. Skipping the erase is
  // important: another buffer may already occupy a neighboring origin.
  if (oldLineCount == 0) {
    for (std::size_t i = 0; i < currentLineCount; ++i)
      pros::screen::print(pros::E_TEXT_SMALL, origin.x,
                          origin.y + static_cast<int>(i * ROW_HEIGHT), "%s",
                          updatedBuffer.lines[i].c_str());
    return;
  }

  const std::size_t rowsToClear = std::max(oldLineCount, currentLineCount);
  const std::size_t charsToClear =
      std::max(oldMaxLineLength, currentMaxLineLength);
  const int right = std::min(
      SCREEN_WIDTH, static_cast<int>(origin.x + charsToClear * CHAR_WIDTH));
  const int bottom = std::min(
      TEXT_AREA_BOTTOM,
      static_cast<int>(origin.y + (rowsToClear - 1) * ROW_HEIGHT + CHAR_HEIGHT));

  if (charsToClear != 0 && right > origin.x && bottom > origin.y)
    pros::screen::erase_rect(origin.x, origin.y, right, bottom);

  // Repaint every buffer whose text rectangle intersects the erased area.
  // This preserves separate scrollback streams when their coordinate regions
  // overlap, with the most recently updated stream painted on top.
  std::vector<std::pair<TextOrigin, const TextBuffer *>> affected;
  for (const auto &[candidateOrigin, candidateBuffer] : textBuffers) {
    const std::size_t candidateMaxLineLength =
        longestLineLength(candidateBuffer.lines);
    if (candidateBuffer.lines.empty() || candidateMaxLineLength == 0) continue;

    const int candidateRight = std::min(
        SCREEN_WIDTH, static_cast<int>(candidateOrigin.x +
                                       candidateMaxLineLength * CHAR_WIDTH));
    const int candidateBottom = std::min(
        TEXT_AREA_BOTTOM,
        static_cast<int>(candidateOrigin.y +
                         (candidateBuffer.lines.size() - 1) * ROW_HEIGHT +
                         CHAR_HEIGHT));

    if (candidateOrigin.x < right && candidateRight > origin.x &&
        candidateOrigin.y < bottom && candidateBottom > origin.y)
      affected.emplace_back(candidateOrigin, &candidateBuffer);
  }

  std::sort(affected.begin(), affected.end(),
            [](const auto &left, const auto &right) {
              return left.second->lastUpdated < right.second->lastUpdated;
            });

  for (const auto &[candidateOrigin, candidateBuffer] : affected)
    for (std::size_t i = 0; i < candidateBuffer->lines.size(); ++i)
      pros::screen::print(pros::E_TEXT_SMALL, candidateOrigin.x,
                          candidateOrigin.y +
                              static_cast<int>(i * ROW_HEIGHT),
                          "%s", candidateBuffer->lines[i].c_str());
}

void Manager::drawCapsule(int x, int y, int width, int height,
                          uint32_t fillColor, uint32_t borderColor) {
  int radius = height / 2;

  // Base Fill
  pros::screen::set_pen(fillColor);
  pros::screen::set_eraser(fillColor); // Set the fill color

  pros::screen::fill_circle(x + radius, y + radius, radius);
  pros::screen::fill_circle(x + width - radius, y + radius, radius);
  pros::screen::fill_rect(x + radius, y, x + width - radius, y + height);

  // --- The outlines ---
  pros::screen::set_pen(borderColor);

  // Draw the full circle outlines at both ends
  pros::screen::draw_circle(x + radius, y + radius, radius);
  pros::screen::draw_circle(x + width - radius, y + radius, radius);

  // Draw the top and bottom connecting lines
  pros::screen::draw_line(x + radius, y, x + width - radius, y);
  pros::screen::draw_line(x + radius, y + height, x + width - radius,
                          y + height);

  // This is the "Soft Rectangle" part. We use the FILL color to overwrite
  // the inner halves of the circles we just drew in Step 2.
  pros::screen::set_pen(fillColor);
  pros::screen::set_eraser(fillColor);

  // We offset by +1 and -1 so we don't erase the top/bottom border lines.
  // This removes the vertical 'seams' inside the button.
  pros::screen::fill_rect(x + radius + 1, y + 1, x + width - radius - 1,
                          y + height - 1);
}

void Manager::drawBottomButtons(bool redraw) {
  MutexGuard m(sharedMutex);
  if (!m.isLocked()) return;

  if (redraw) clearScreenLocked();

  const auto penColor = pros::screen::get_pen();
  const auto eraserColor = pros::screen::get_eraser();

  drawCapsule(getX(ButtonId::LEFT), BTN_Y, BTN_WIDTH, BTN_HEIGHT,
              COLOR_BTN_FILL, COLOR_BTN_BORDER);
  drawCapsule(getX(ButtonId::MIDDLE), BTN_Y, BTN_WIDTH, BTN_HEIGHT,
              COLOR_BTN_FILL, COLOR_BTN_BORDER);
  drawCapsule(getX(ButtonId::RIGHT), BTN_Y, BTN_WIDTH, BTN_HEIGHT,
              COLOR_BTN_FILL, COLOR_BTN_BORDER);

  pros::screen::set_pen(penColor);
  pros::screen::set_eraser(eraserColor);
}

/**
 * @brief returns if the given coords are inside of the given box. Specific for
 * Manager class
 */

bool Manager::isInside(int px, int py, int bx, int by, int w, int h) {
  // PADDING intentionally makes near-edge taps count as button taps.
  return (px >= (bx - PADDING) && px <= (bx + w + PADDING) &&
          py >= (by - PADDING) && py <= (by + h + PADDING));
}

ButtonId Manager::waitForBottomButtonTap(uint32_t timeout) {
  if (timeout == 0) return ButtonId::NONE;

  int last_release_count = pros::screen::touch_status().release_count;
  uint32_t startTime = pros::millis();
  int final_x = -1, final_y = -1;

  while (pros::millis() - startTime < timeout) {
    pros::screen_touch_status_s touch = pros::screen::touch_status();

    if (touch.touch_status == pros::E_TOUCH_PRESSED ||
        touch.touch_status == pros::E_TOUCH_HELD) {
      final_x = touch.x;
      final_y = touch.y;
    }

    if (touch.release_count > last_release_count) {
      last_release_count = touch.release_count;
      if (final_x != -1 && final_y != -1) {
        if (isInside(final_x, final_y, getX(ButtonId::LEFT), 
                     BTN_Y, BTN_WIDTH, BTN_HEIGHT))
          return ButtonId::LEFT;
        if (isInside(final_x, final_y, getX(ButtonId::MIDDLE), 
                     BTN_Y, BTN_WIDTH, BTN_HEIGHT))
          return ButtonId::MIDDLE;
        if (isInside(final_x, final_y, getX(ButtonId::RIGHT), 
                     BTN_Y, BTN_WIDTH, BTN_HEIGHT))
          return ButtonId::RIGHT;
      }
      final_x = -1;
      final_y = -1;
    }
    pros::delay(20);
  }
  return ButtonId::NONE;
}

bool Manager::waitForScreenTouch(uint32_t timeoutMs, bool detectTouchOnly) {
  if (timeoutMs == 0) return false;
  uint32_t start = pros::millis();
  int32_t initialRelease = pros::screen::touch_status().release_count;

  while ((pros::millis() - start) < timeoutMs) {
    auto status = pros::screen::touch_status();

    if (detectTouchOnly &&
        (status.touch_status == pros::E_TOUCH_PRESSED ||
         status.touch_status == pros::E_TOUCH_HELD))
      return true;
    if (!detectTouchOnly && status.release_count > initialRelease) return true;
    pros::delay(20);
  }
  return false;
}
} // namespace screen
