// MecanumNavigator.ino
// Autonomous navigation for Arduino Nano with mecanum wheels.
// Uses fixed-size occupancy grid, sensor smoothing, simple fusion, frontier selection, and A*.

#define USE_SIMPLE_FALLBACK 0
#define DEBUG_SERIAL 1

// Grid and navigation parameters
const uint8_t GRID_SIZE = 10;
const uint8_t GRID_UNKNOWN = 0;
const uint8_t GRID_FREE = 1;
const uint8_t GRID_OCCUPIED = 2;
const uint8_t START_X = GRID_SIZE / 2;
const uint8_t START_Y = GRID_SIZE / 2;
const uint8_t CELL_SIZE_CM = 20;
const uint8_t MAX_SENSOR_RANGE_CM = 80;
const uint8_t COLLISION_MARGIN_CM = 18;
const uint16_t MOVE_DURATION_MS = 550;
const uint16_t STOP_DELAY_MS = 120;
const uint8_t TRAVEL_SPEED = 120;

// Sensor pins (placeholders)
const int PIN_US_TRIG = 7;
const int PIN_US_ECHO = 6;
const int PIN_IR_FRONT = A0;
const int PIN_IR_LEFT = A1;
const int PIN_IR_RIGHT = A2;

const uint8_t SENSOR_BUFFER_SIZE = 5;
const uint8_t MAX_OPEN_NODES = GRID_SIZE * GRID_SIZE;
const uint8_t INVALID_INDEX = 255;

// Assumed motor functions implemented elsewhere
void moveForward(int speed);
void moveBackward(int speed);
void strafeLeft(int speed);
void strafeRight(int speed);
void rotateLeft(int speed);
void rotateRight(int speed);
void stopMotors();

struct Point {
  uint8_t x;
  uint8_t y;
};

class SensorBuffer {
public:
  SensorBuffer() : index(0), count(0) {}

  void add(uint16_t reading) {
    buffer[index] = reading;
    index = (index + 1) % SENSOR_BUFFER_SIZE;
    if (count < SENSOR_BUFFER_SIZE) {
      count++;
    }
  }

  uint16_t average() const {
    if (count == 0) {
      return 0;
    }
    uint32_t sum = 0;
    for (uint8_t i = 0; i < count; i++) {
      sum += buffer[i];
    }
    return uint16_t(sum / count);
  }

private:
  uint16_t buffer[SENSOR_BUFFER_SIZE];
  uint8_t index;
  uint8_t count;
};

const int8_t DIR_X[4] = {1, -1, 0, 0};
const int8_t DIR_Y[4] = {0, 0, 1, -1};

class OccupancyGrid {
public:
  void init() {
    for (uint8_t y = 0; y < GRID_SIZE; y++) {
      for (uint8_t x = 0; x < GRID_SIZE; x++) {
        cells[y][x] = GRID_UNKNOWN;
      }
    }
    setFree({START_X, START_Y});
  }

  bool inBounds(Point p) const {
    return p.x < GRID_SIZE && p.y < GRID_SIZE;
  }

  void setFree(Point p) {
    if (!inBounds(p)) {
      return;
    }
    if (cells[p.y][p.x] != GRID_OCCUPIED) {
      cells[p.y][p.x] = GRID_FREE;
    }
  }

  void setOccupied(Point p) {
    if (!inBounds(p)) {
      return;
    }
    cells[p.y][p.x] = GRID_OCCUPIED;
  }

  bool isFree(Point p) const {
    return inBounds(p) && cells[p.y][p.x] == GRID_FREE;
  }

  bool isOccupied(Point p) const {
    return inBounds(p) && cells[p.y][p.x] == GRID_OCCUPIED;
  }

  bool isUnknown(Point p) const {
    return inBounds(p) && cells[p.y][p.x] == GRID_UNKNOWN;
  }

  uint8_t getCell(Point p) const {
    return inBounds(p) ? cells[p.y][p.x] : GRID_OCCUPIED;
  }

  bool isTraversable(Point p, uint8_t goalIndex) const {
    if (!inBounds(p)) {
      return false;
    }
    if (pointToIndex(p) == goalIndex) {
      return !isOccupied(p);
    }
    return cells[p.y][p.x] != GRID_OCCUPIED;
  }

  bool isFrontier(Point p) const {
    if (!isUnknown(p)) {
      return false;
    }
    for (uint8_t d = 0; d < 4; d++) {
      Point neighbor = {uint8_t(int(p.x) + DIR_X[d]), uint8_t(int(p.y) + DIR_Y[d])};
      if (inBounds(neighbor) && isFree(neighbor)) {
        return true;
      }
    }
    return false;
  }

  bool findNearestFrontier(Point robot, Point &outFrontier) const {
    uint16_t bestDistance = 0xFFFF;
    bool found = false;
    for (uint8_t y = 0; y < GRID_SIZE; y++) {
      for (uint8_t x = 0; x < GRID_SIZE; x++) {
        Point candidate = {x, y};
        if (!isFrontier(candidate)) {
          continue;
        }
        uint16_t dist = manhattanDistance(robot, candidate);
        if (!found || dist < bestDistance) {
          bestDistance = dist;
          outFrontier = candidate;
          found = true;
        }
      }
    }
    return found;
  }

  void markFreeRay(Point origin, int8_t dx, int8_t dy, uint8_t steps) {
    for (uint8_t step = 1; step <= steps; step++) {
      Point current = {uint8_t(int(origin.x) + dx * step), uint8_t(int(origin.y) + dy * step)};
      if (!inBounds(current)) {
        break;
      }
      setFree(current);
    }
  }

  static Point indexToPoint(uint8_t index) {
    return {uint8_t(index % GRID_SIZE), uint8_t(index / GRID_SIZE)};
  }

  static uint8_t pointToIndex(Point p) {
    return uint8_t(p.y * GRID_SIZE + p.x);
  }

private:
  uint8_t cells[GRID_SIZE][GRID_SIZE];

  static uint16_t manhattanDistance(Point a, Point b) {
    uint16_t dx = (a.x > b.x) ? a.x - b.x : b.x - a.x;
    uint16_t dy = (a.y > b.y) ? a.y - b.y : b.y - a.y;
    return dx + dy;
  }
};

struct Path {
  Point steps[MAX_OPEN_NODES];
  uint8_t length;
};

class AStarPlanner {
public:
  void init(const OccupancyGrid &grid) {
    map = &grid;
  }

  bool plan(Point start, Point goal, Path &outPath) {
    uint8_t startIndex = pointToIndex(start);
    uint8_t goalIndex = pointToIndex(goal);
    resetSearch();

    gScore[startIndex] = 0;
    fScore[startIndex] = heuristic(start, goal);
    if (openCount < MAX_OPEN_NODES) {
      openList[openCount++] = startIndex;
      openFlags[startIndex] = true;
    }

    while (openCount > 0) {
      uint8_t currentIdx = selectLowestF();
      Point current = indexToPoint(currentIdx);
      if (currentIdx == goalIndex) {
        reconstructPath(startIndex, currentIdx, outPath);
        return true;
      }

      removeOpen(currentIdx);
      closedFlags[currentIdx] = true;

      for (uint8_t d = 0; d < 4; d++) {
        Point neighbor = {uint8_t(int(current.x) + DIR_X[d]), uint8_t(int(current.y) + DIR_Y[d])};
        if (!isPointValid(neighbor)) {
          continue;
        }
        uint8_t neighborIndex = pointToIndex(neighbor);
        if (closedFlags[neighborIndex]) {
          continue;
        }
        if (!map->isTraversable(neighbor, goalIndex)) {
          continue;
        }

        uint8_t tentativeG = gScore[currentIdx] + 1;
        if (!openFlags[neighborIndex]) {
          if (openCount < MAX_OPEN_NODES) {
            openList[openCount++] = neighborIndex;
            openFlags[neighborIndex] = true;
          }
        } else if (tentativeG >= gScore[neighborIndex]) {
          continue;
        }

        parent[neighborIndex] = currentIdx;
        gScore[neighborIndex] = tentativeG;
        fScore[neighborIndex] = tentativeG + heuristic(neighbor, goal);
      }
    }
    return false;
  }

private:
  const OccupancyGrid *map;
  uint8_t openList[MAX_OPEN_NODES];
  bool openFlags[MAX_OPEN_NODES];
  bool closedFlags[MAX_OPEN_NODES];
  uint8_t gScore[MAX_OPEN_NODES];
  uint8_t fScore[MAX_OPEN_NODES];
  uint8_t parent[MAX_OPEN_NODES];
  uint8_t openCount;

  void resetSearch() {
    openCount = 0;
    for (uint8_t i = 0; i < MAX_OPEN_NODES; i++) {
      openFlags[i] = false;
      closedFlags[i] = false;
      gScore[i] = 0xFF;
      fScore[i] = 0xFF;
      parent[i] = INVALID_INDEX;
    }
  }

  inline bool isPointValid(Point p) const {
    return p.x < GRID_SIZE && p.y < GRID_SIZE;
  }

  inline uint8_t pointToIndex(Point p) const {
    return uint8_t(p.y * GRID_SIZE + p.x);
  }

  static inline Point indexToPoint(uint8_t index) {
    return {uint8_t(index % GRID_SIZE), uint8_t(index / GRID_SIZE)};
  }

  uint8_t selectLowestF() const {
    uint8_t bestPos = 0;
    uint8_t bestValue = 0xFF;
    for (uint8_t i = 0; i < openCount; i++) {
      uint8_t candidateIndex = openList[i];
      if (fScore[candidateIndex] < bestValue) {
        bestValue = fScore[candidateIndex];
        bestPos = i;
      }
    }
    return openList[bestPos];
  }

  void removeOpen(uint8_t indexValue) {
    for (uint8_t i = 0; i < openCount; i++) {
      if (openList[i] == indexValue) {
        openFlags[indexValue] = false;
        openCount--;
        openList[i] = openList[openCount];
        return;
      }
    }
  }

  static uint8_t heuristic(Point a, Point b) {
    uint8_t dx = (a.x > b.x) ? a.x - b.x : b.x - a.x;
    uint8_t dy = (a.y > b.y) ? a.y - b.y : b.y - a.y;
    return dx + dy;
  }

  void reconstructPath(uint8_t startIndex, uint8_t currentIndex, Path &outPath) const {
    uint8_t tempLength = 0;
    while (currentIndex != startIndex && tempLength < MAX_OPEN_NODES) {
      outPath.steps[tempLength++] = indexToPoint(currentIndex);
      currentIndex = parent[currentIndex];
      if (currentIndex == INVALID_INDEX) {
        break;
      }
    }
    outPath.steps[tempLength++] = indexToPoint(startIndex);
    outPath.length = tempLength;

    for (uint8_t i = 0; i < tempLength / 2; i++) {
      Point swap = outPath.steps[i];
      outPath.steps[i] = outPath.steps[tempLength - 1 - i];
      outPath.steps[tempLength - 1 - i] = swap;
    }
  }
};

class SensorFusion {
public:
  static uint16_t fuseFront(uint16_t ultrasonicCm, uint16_t irCm) {
    if (ultrasonicCm == 0) {
      return irCm;
    }
    if (irCm == MAX_SENSOR_RANGE_CM) {
      return ultrasonicCm;
    }
    return min(ultrasonicCm, irCm);
  }
};

OccupancyGrid grid;
AStarPlanner planner;
SensorBuffer ultrasonicBuffer;
SensorBuffer irFrontBuffer;
SensorBuffer irLeftBuffer;
SensorBuffer irRightBuffer;
Point robotPosition = {START_X, START_Y};

uint16_t readUltrasonicCm() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_US_ECHO, HIGH, 20000UL);
  if (duration == 0) {
    return MAX_SENSOR_RANGE_CM;
  }
  uint16_t distanceCm = uint16_t(duration / 58UL);
  if (distanceCm > MAX_SENSOR_RANGE_CM) {
    distanceCm = MAX_SENSOR_RANGE_CM;
  }
  return distanceCm;
}

uint16_t estimateIRDistanceCm(uint16_t rawValue) {
  if (rawValue > 700) {
    return 8;
  } else if (rawValue > 560) {
    return 15;
  } else if (rawValue > 450) {
    return 25;
  } else if (rawValue > 330) {
    return 35;
  }
  return MAX_SENSOR_RANGE_CM;
}

void updateOccupancyFromSensors(uint16_t frontCm, uint16_t leftCm, uint16_t rightCm) {
  uint8_t forwardSteps = min(uint8_t(frontCm / CELL_SIZE_CM), uint8_t(GRID_SIZE - 1));
  grid.markFreeRay(robotPosition, 0, -1, forwardSteps);

  if (frontCm < MAX_SENSOR_RANGE_CM && forwardSteps < GRID_SIZE - 1) {
    Point obstacle = {robotPosition.x, uint8_t(int(robotPosition.y) - int(forwardSteps) - 1)};
    if (grid.inBounds(obstacle)) {
      grid.setOccupied(obstacle);
    }
  }

  if (robotPosition.x > 0) {
    Point leftCell = {uint8_t(robotPosition.x - 1), robotPosition.y};
    if (leftCm < MAX_SENSOR_RANGE_CM && leftCm < COLLISION_MARGIN_CM) {
      grid.setOccupied(leftCell);
    } else {
      grid.setFree(leftCell);
    }
  }

  if (robotPosition.x + 1 < GRID_SIZE) {
    Point rightCell = {uint8_t(robotPosition.x + 1), robotPosition.y};
    if (rightCm < MAX_SENSOR_RANGE_CM && rightCm < COLLISION_MARGIN_CM) {
      grid.setOccupied(rightCell);
    } else {
      grid.setFree(rightCell);
    }
  }
}

void moveStep(Point nextCell) {
  int8_t dx = int8_t(nextCell.x) - int8_t(robotPosition.x);
  int8_t dy = int8_t(nextCell.y) - int8_t(robotPosition.y);

  if (dx == 1 && dy == 0) {
    strafeRight(TRAVEL_SPEED);
  } else if (dx == -1 && dy == 0) {
    strafeLeft(TRAVEL_SPEED);
  } else if (dx == 0 && dy == -1) {
    moveForward(TRAVEL_SPEED);
  } else if (dx == 0 && dy == 1) {
    moveBackward(TRAVEL_SPEED);
  } else {
    rotateRight(TRAVEL_SPEED);
    delay(100);
    stopMotors();
    return;
  }

  delay(MOVE_DURATION_MS);
  stopMotors();
  delay(STOP_DELAY_MS);
  robotPosition = nextCell;
  grid.setFree(robotPosition);
}

bool navigateToFrontier() {
  Point target;
  if (!grid.findNearestFrontier(robotPosition, target)) {
    return false;
  }

#if USE_SIMPLE_FALLBACK
  Point step = robotPosition;
  if (target.x > robotPosition.x) {
    step.x++;
  } else if (target.x < robotPosition.x) {
    step.x--;
  } else if (target.y > robotPosition.y) {
    step.y++;
  } else if (target.y < robotPosition.y) {
    step.y--;
  }
  if (!grid.inBounds(step) || grid.isOccupied(step)) {
    return false;
  }
  moveStep(step);
  return true;
#else
  Path path;
  if (!planner.plan(robotPosition, target, path) || path.length < 2) {
    return false;
  }
  moveStep(path.steps[1]);
  return true;
#endif
}

void printGrid() {
#if DEBUG_SERIAL
  Serial.println(F("Grid map:"));
  for (uint8_t y = 0; y < GRID_SIZE; y++) {
    for (uint8_t x = 0; x < GRID_SIZE; x++) {
      if (robotPosition.x == x && robotPosition.y == y) {
        Serial.print(F("R "));
      } else {
        Point p = {x, y};
        uint8_t cell = grid.getCell(p);
        if (cell == GRID_UNKNOWN) {
          Serial.print(F("? "));
        } else if (cell == GRID_FREE) {
          Serial.print(F(". "));
        } else {
          Serial.print(F("X "));
        }
      }
    }
    Serial.println();
  }
  Serial.println();
#endif
}

void setup() {
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  pinMode(PIN_IR_FRONT, INPUT);
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);

#if DEBUG_SERIAL
  Serial.begin(9600);
  Serial.println(F("Mecanum auto navigation started."));
#endif

  grid.init();
  planner.init(grid);
}

void loop() {
  uint16_t rawUltrasonic = readUltrasonicCm();
  ultrasonicBuffer.add(rawUltrasonic);
  uint16_t smoothUltrasonic = ultrasonicBuffer.average();

  uint16_t rawIrFront = analogRead(PIN_IR_FRONT);
  irFrontBuffer.add(rawIrFront);
  uint16_t smoothIrFront = estimateIRDistanceCm(irFrontBuffer.average());

  uint16_t rawIrLeft = analogRead(PIN_IR_LEFT);
  irLeftBuffer.add(rawIrLeft);
  uint16_t smoothIrLeft = estimateIRDistanceCm(irLeftBuffer.average());

  uint16_t rawIrRight = analogRead(PIN_IR_RIGHT);
  irRightBuffer.add(rawIrRight);
  uint16_t smoothIrRight = estimateIRDistanceCm(irRightBuffer.average());

  uint16_t fusedFront = SensorFusion::fuseFront(smoothUltrasonic, smoothIrFront);

  updateOccupancyFromSensors(fusedFront, smoothIrLeft, smoothIrRight);

  if (fusedFront < COLLISION_MARGIN_CM) {
    Point ahead = {robotPosition.x, uint8_t(int(robotPosition.y) - 1)};
    if (grid.inBounds(ahead)) {
      grid.setOccupied(ahead);
    }
  }

  bool moved = navigateToFrontier();
  if (!moved) {
    stopMotors();
#if DEBUG_SERIAL
    Serial.println(F("No valid movement step available. Replanning..."));
    printGrid();
#endif
    delay(400);
  }

  delay(50);
}
