// Robot characteristics

const int BASE_WIDTH = 200; // in millimeters
const int WHEEL_RADIUS = 0.1; // in meters
const float MAX_WIDTH = 5.0; // width limit of map in meters
const float MAX_HEIGHT = 3.25; // height limit of map in meters
const float SIZE_ROBOT = 0.4; // size max of robot in meters
int MAX_SPEED = 10; // speed in radian/s of wheels by default

// Camera description

const static float STD_DEV = 2.87; // std of sensor
const static float V_MIN = 0.1; // value min
const static float V_MAX = 10.0; // value max
