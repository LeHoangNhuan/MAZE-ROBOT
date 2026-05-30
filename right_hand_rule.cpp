#include "right_hand_rule.h"

// ===== THRESHOLD =====
#define FRONT_BLOCKED_MM  165//160  //130

// Hysteresis for side walls
#define SIDE_OPEN_HIGH    250 //270 //230   // mở khi > 230
#define SIDE_OPEN_LOW     170   // đóng khi < 220

// HARD BLOCK threshold for 180-turn
#define SIDE_BLOCKED_MM   170

// dừng 1s rồi mới quyết định
#define STOP_WAIT_MS      50//100

// moveActive quản lý ở motor_ctrl.cpp
extern volatile bool moveActive;



static inline void turnRight() { headingDir = (headingDir + 1) % 4; }
static inline void turnLeft()  { headingDir = (headingDir + 3) % 4; }
static inline void turnBack()  { headingDir = (headingDir + 2) % 4; }

// 0 = none, 1 = right, 2 = left
static uint8_t pendingTurn = 0;

// hysteresis states
static bool rightOpenState = false;
static bool leftOpenState  = false;

// hard blocked states (raw)
static bool rightHardBlocked = false;
static bool leftHardBlocked  = false;

// stop timer
static bool waitingAfterStop = false;
static unsigned long stopAt  = 0;

// dead-end scan state
enum DeadEndState {
  DEADEND_NONE,
  DEADEND_CHECK_RIGHT,
  DEADEND_CHECK_LEFT
};
static DeadEndState deadEndState = DEADEND_NONE;

static bool hysteresisOpen(bool prev, uint16_t dist) {
  if (prev) return dist > SIDE_OPEN_LOW;
  return dist > SIDE_OPEN_HIGH;
}

static WallState detectWalls(uint16_t fl, uint16_t fr, uint16_t left, uint16_t right) {
  // front lấy MAX theo yêu cầu
  uint16_t front = (fl > fr) ? fl : fr;

  WallState w;
  w.frontBlocked = (front < FRONT_BLOCKED_MM);

  rightOpenState = hysteresisOpen(rightOpenState, right);
  leftOpenState  = hysteresisOpen(leftOpenState, left);

  rightHardBlocked = (right < SIDE_BLOCKED_MM);
  leftHardBlocked  = (left  < SIDE_BLOCKED_MM);

  w.rightOpen = rightOpenState;
  w.leftOpen  = leftOpenState;

  // DEBUG PRINT
  Serial.print("[WALL] F:"); Serial.print(front);
  Serial.print(" L:"); Serial.print(left);
  Serial.print(" R:"); Serial.print(right);
  Serial.print(" | Fblk:"); Serial.print(w.frontBlocked);
  Serial.print(" Lopen:"); Serial.print(w.leftOpen);
  Serial.print(" Ropen:"); Serial.println(w.rightOpen);

  return w;
}

static void rightHandDecision(const WallState &w) {
  // nếu đang chờ rẽ (sau khi đi 1 ô) -> rẽ ngay
  if (pendingTurn == 1) { spinRight90(); turnRight(); pendingTurn = 0; moveActive = true; return; }
  if (pendingTurn == 2) { spinLeft90();  turnLeft();  pendingTurn = 0; moveActive = true; return; }

  bool frontClear = !w.frontBlocked;

  // Ưu tiên phải: có đường phải -> đi thẳng 1 ô rồi rẽ phải
  if (w.rightOpen) { pendingTurn = 1; goForward(); moveActive = true; return; }

  // Không có đường phải -> nếu đi thẳng được thì đi thẳng
  if (frontClear) { goForward(); moveActive = true; return; }

  // Không đi thẳng được -> nếu trái mở thì đi thẳng 1 ô rồi rẽ trái
  if (w.leftOpen) { pendingTurn = 2; goForward(); moveActive = true; return; }

  // Cả ba đều chặn -> quay đầu
  if (rightHardBlocked && leftHardBlocked) {
    spin180(); turnBack(); moveActive = true; return;
  }

  // fallback: quay đầu
  spin180(); turnBack(); moveActive = true;
}

static bool isDeadEnd(const WallState &w) {
  return w.frontBlocked && !w.rightOpen && !w.leftOpen;
}

static bool handleDeadEnd(const WallState &w) {
  bool frontClear = !w.frontBlocked;

  if (deadEndState == DEADEND_NONE) {
    if (!isDeadEnd(w)) return false;

    pendingTurn = 0;
    spinRight90();
    turnRight();
    moveActive = true;
    deadEndState = DEADEND_CHECK_RIGHT;
    return true;
  }

  if (deadEndState == DEADEND_CHECK_RIGHT) {
    if (frontClear) {
      deadEndState = DEADEND_NONE;
      rightHandDecision(w);
      return true;
    }
    // không có đường phía trước -> quay 180 để kiểm tra bên trái
    spin180();
    turnBack();
    moveActive = true;
    deadEndState = DEADEND_CHECK_LEFT;
    return true;
  }

  if (deadEndState == DEADEND_CHECK_LEFT) {
    if (frontClear) {
      deadEndState = DEADEND_NONE;
      rightHandDecision(w);
      return true;
    }
    // cả hai bên đều không có đường -> quay trái để thành quay đầu
    spinLeft90();
    turnLeft();
    moveActive = true;
    deadEndState = DEADEND_NONE;
    return true;
  }

  return false;
}

// ====== API GỌI TỪ .INO ======
void rightHandUpdate(uint16_t fl, uint16_t fr, uint16_t left, uint16_t right) {
  if (moveActive) {
    waitingAfterStop = false;
    return;
  }

  if (!waitingAfterStop) {
    stopAt = millis();
    waitingAfterStop = true;
    return;
  }

  if (millis() - stopAt < STOP_WAIT_MS) {
    return;
  }

  WallState w = detectWalls(fl, fr, left, right);

  if (handleDeadEnd(w)) {
    waitingAfterStop = false;
    return;
  }

  rightHandDecision(w);
  waitingAfterStop = false;
}