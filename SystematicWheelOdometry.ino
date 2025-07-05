/*
    Authors: Group B378,
    Date: 10/9/2022 10:55PM
*/

#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <math.h>

#define MOTOR_SPEED 100
#define NUM_SENSORLEVELS 6
#define DEBUG_MODE false

Zumo32U4LCD LCD;
Zumo32U4IMU IMU;
Zumo32U4ButtonA ButtonA;
Zumo32U4ProximitySensors ProximitySensors;

uint16_t BrightnessLevels[] = { 1,1,3,3,5,5 };

//Booleans that help the robot know how far in the process it is
bool Rotating = false;
bool ReachedDestination = false;
bool NavFinished = false;
bool Navigating = false;
bool SensorReads = true;
bool Bounce = true;
bool Reset = true;

////////////////////////Odometry Constants and variables/////////////////////////
int WheelDiameter = 4; // 4cm
int AxleLength = 9;
int DeltaT = 25;
double CurrentX = 0;
double CurrentY = 0;
double DeltaX = 0;
double DeltaY = 0;
int GlobalTheta; // Orientation
double MomentaryTheta, TargetTheta, DeltaD;
double VelocityR = 0;
double VelocityL = 0;
double FWDVelocity = 0;

//////////////////////// MOTOR ////////////////////////
Zumo32U4Motors Motors;
Zumo32U4Encoders WheelEncoders;
int DrivenDistance;
int Angle;

////////////////Gyro setup/////////////////////
int TurnAngleDegrees;

/* turnAngle is a 32-bit unsigned integer representing the amount
the robot has turned since the last time turnSensorReset was
called.  This is computed solely using the Z axis of the gyro, so
it could be inaccurate if the robot is rotated about the X or Y
axes.
Our convention is that a value of 0x20000000 represents a 45
degree counter-clockwise rotation.  This means that a uint32_t
can represent any angle between 0 degrees and 360 degrees.  If
you cast it to a signed 32-bit integer by writing
(int32_t)turnAngle, that integer can represent any angle between
-180 degrees and 180 degrees. */
uint32_t CurrentTurnAngle = 0;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t GyroLastUpdate = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
// GyroOffset is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t TurnRate, GyroOffset;

/// <summary>
/// A single waypoint struct that represents a waypoint within a 2d frame
/// </summary>
struct Waypoint
{
    int X;
    int Y;
    Waypoint* next; // Chain reference to the next waypoint in queue (If queue is not NULL)

    Waypoint(int x, int y) {
        X = x;
        Y = y;
        next = NULL;
    }
};

/// <summary>
/// Debug log function that only logs if DEBUG_MODE is set to true.
/// </summary>
/// <param name="text"></param>
void debugLog(String text) {
    if (DEBUG_MODE)
        Serial.println("[DEBUG]<" + (String)__TIME__ + ">: " + text);
}

/// <summary>
/// The overall struct for the waypoint queue that contains a rear and front end for Waypoints.
/// </summary>
struct WaypointQueue
{
    Waypoint* Front, * Rear;

    WaypointQueue() {
        Front = Rear = NULL;
    }

    /// <summary>
    /// Adds a waypoint to the queue
    /// </summary>
    /// <param name="x"></param>
    /// <param name="y"></param>
    void EnQueue(int x, int y)
    {
        Waypoint* temp = new Waypoint(x, y);

        if (Rear == NULL) {
            Front = Rear = temp;
            return;
        }

        Rear->next = temp;
        Rear = temp;
        debugLog("Waypoint has been enqueued. Data (X): " + (String)temp->X + ", (Y): " + (String)temp->Y);
    }

    /// <summary>
    /// Removes a waypoint from the queue
    /// </summary>
    void DeQueue()
    {
        if (Front == NULL) {
            return;
        }

        Waypoint* temp = Front;
        Front = Front->next;

        if (Front == NULL) {
            Rear = NULL;
        }

        //debugLog("Removed waypoint from queue. Data (X): " + (String)temp->X + ", (Y): " + (String)temp->Y);
        delete(temp);
        //debugLog("Removed. X value for the next waypoint in queue: " + (String)front->X)
    }
};

WaypointQueue WaypointQueue_;

// the setup function runs once when you press reset or power the board
void setup() {
    ConfigureComponents();
}

// the loop function runs over and over again until power down or reset
void loop() {
    WaypointQueue_.DeQueue();
    // If there is a waypoint within the queue
    if (WaypointQueue_.Front != NULL) {
        MoveTo(WaypointQueue_.Front->X, WaypointQueue_.Front->Y);
    }
    // If there is no waypoints left in the queue
    else {
        NavFinished = true;
    }
    // If a reset is indicated
    if (NavFinished && Reset) {
        Reset = false;
        TurnAngle(0, 'R');
        Motors.setSpeeds(-MOTOR_SPEED, -MOTOR_SPEED);
        delay(1500);
        Motors.setSpeeds(0, 0);
    }
}

/// <summary>
/// Configures the components and sets the initial values and properties required for the code to run
/// properly
/// </summary>
void ConfigureComponents() {
    Serial.begin(9600);
    GyroscopeSetup();
    delay(500);
    GyroscopeReset();
    LCD.clear();
    ButtonA.waitForPress();
    delay(250);
    ProximitySensors.initThreeSensors();
    ProximitySensors.setBrightnessLevels(BrightnessLevels, sizeof(BrightnessLevels) / 2);
    CurrentX = -23;
    CurrentY = 10;
    EnQueueWaypoints();
    MoveTo(WaypointQueue_.Front->X, WaypointQueue_.Front->Y);
}

/// <summary>
/// Enqueues hardcoded waypoints that are made for the purpose of demonstration
/// </summary>
void EnQueueWaypoints() {
    WaypointQueue_.EnQueue(0, 11);
    WaypointQueue_.EnQueue(0, 45);
    WaypointQueue_.EnQueue(8, 45);
    WaypointQueue_.EnQueue(8, 10);
    WaypointQueue_.EnQueue(16, 10);
    WaypointQueue_.EnQueue(16, 45);
    WaypointQueue_.EnQueue(24, 45);
    WaypointQueue_.EnQueue(24, 10);
    WaypointQueue_.EnQueue(32, 10);
    WaypointQueue_.EnQueue(32, 45);
    WaypointQueue_.EnQueue(40, 45);
    WaypointQueue_.EnQueue(40, 10);
    WaypointQueue_.EnQueue(48, 10);
    WaypointQueue_.EnQueue(48, 45);
    WaypointQueue_.EnQueue(56, 45);
    WaypointQueue_.EnQueue(56, 10);
    WaypointQueue_.EnQueue(64, 10);
    WaypointQueue_.EnQueue(64, 40);
    WaypointQueue_.EnQueue(-23, 10);
}

/// <summary>
/// The initial setup function for configuration the IMU's gyroscope
/// </summary>
void GyroscopeSetup() {
    Wire.begin();
    IMU.init();
    IMU.enableDefault();
    IMU.configureForTurnSensing();
    LCD.clear();
    LCD.print(F("Gyro cal"));

    // Turn on the yellow LED in case the LCD is not available.
    ledYellow(1);
    // Calibrate the gyro.
    int32_t total = 0;
    int sampleIterations = 1024;
    for (uint16_t i = 0; i < sampleIterations; i++) {
        // Wait for new data to be available, then read it.
        while (!IMU.gyroDataReady()) {} // Is this needed????
        IMU.readGyro();
        // Add the Z axis reading to the total.
        total += IMU.g.z;
    }
    ledYellow(0);
    GyroOffset = total / sampleIterations;
    delay(250);
    // Display the angle (in degrees from -180 to 180)
    LCD.clear();
}

/// <summary>
/// Resetting the current turned angle of the robot and the last update counter for the
/// gyroscope
/// </summary>
void GyroscopeReset() {
    GyroLastUpdate = micros();
    CurrentTurnAngle = 0;
}

/// <summary>
/// The turn function for the robot that turns to a given angle in degrees in a given direction
/// that is represented by a char.
/// </summary>
/// <param name="goalAngle"></param>
/// <param name="Direction"></param>
void TurnAngle(int goalAngle, char direction)
{
    switch (direction)
    {
        // Turn in a right direction
        case 'R':
            GyroscopeUpdate();

            while (!Rotating) {
                UpdatePosition();
                if (GlobalTheta >= goalAngle && (goalAngle) >= GlobalTheta) {
                    Motors.setSpeeds(0, 0);
                    Rotating = true;
                }
                else {
                    Motors.setSpeeds(MOTOR_SPEED, -MOTOR_SPEED * 0.85);
                }
            }
            Rotating = false;
            break;

        // Turn in a left direction
        case 'L':
            GyroscopeUpdate();
            while (!Rotating)
            {
                UpdatePosition();
                if (GlobalTheta >= goalAngle && (goalAngle) >= GlobalTheta) {
                    Motors.setSpeeds(0, 0);
                    Rotating = true;
                }
                else {
                    Motors.setSpeeds(-MOTOR_SPEED, MOTOR_SPEED);
                }
            }
            Rotating = false;
            break;
    }
}

/// <summary>
/// Returns a boolean value that represents whether the robot is at the desired
/// location or not. This function also handles the difference between waypoints and current
/// positions in the frame when it is moving from a negative position to a negative waypoint.
/// </summary>
/// <param name="negX"></param>
/// <param name="negY"></param>
/// <param name="x"></param>
/// <param name="y"></param>
/// <param name="inverseX"></param>
/// <param name="inverseY"></param>
/// <returns></returns>
bool VerifyCurrentDestination(bool negX, bool negY, int x, int y, bool xInversed, bool yInversed)
{
    // Dear reader, we apologize for this cracked code but we are happy to inform you it works perfectly!
    bool ret = false;
    int equationLeftX = xInversed ? x : CurrentX;
    int equationLeftY = yInversed ? y : CurrentY;
    int equationRightX = xInversed ? CurrentX : x;
    int equationRightY = yInversed ? CurrentY : y;

    if (negX && negY) {
        debugLog("Moving to a pos where: " + (String)equationLeftY + " <= " + (String)equationRightY);
        ret = equationLeftX <= equationRightX && equationLeftY <= equationRightY ? true : false;
    }
    else if (!negX && !negY) {
        debugLog("Moving to a pos where: " + (String)equationLeftY + " >= " + (String)equationRightY);
        ret = equationLeftX >= equationRightX && equationLeftY >= equationRightY ? true : false;
    }
    else if (!negX && negY) {
        debugLog("Moving to a pos where: " + (String)equationLeftY + " <= " + (String)equationRightY);
        ret = equationLeftX >= equationRightX && equationLeftY <= equationRightY ? true : false;
    }
    else if (negX && !negY) {
        debugLog("Moving to a pos where: " + (String)equationLeftY + " >= " + (String)equationRightY);
        ret = equationLeftX <= equationRightX && equationLeftY >= equationRightY ? true : false;
    }
    return ret;
}

/// <summary>
/// Rotates the robot and calls for a forward movement to the desired waypoint
/// </summary>
/// <param name="x"></param>
/// <param name="y"></param>
void MoveTo(int x, int y) 
{
    Navigating = true;
    int destx = x - CurrentX;
    int desty = y - CurrentY;
    char rotationalDirection;

    // Converts from radians to degrees
    int angleDeg = atan2(desty, destx) * 180 / PI;
    Angle = (angleDeg + 360) % 360;

    if (GlobalTheta > 245 && Angle < 90)
    {
        rotationalDirection = 'L';
    }
    else
    {
        rotationalDirection = 0 < Angle && Angle < 180 ? 'L' : 'R';
    }

    bool destXNeg = x < 0 ? true : false;
    bool destYNeg = y < 0 ? true : false;
    ReachedDestination = false;
    TurnAngle(Angle, rotationalDirection);
    MoveForward(x, y, destXNeg, destYNeg, Angle);
}

/// <summary>
/// Moves the robot in a forward direction until it reaches a given destination
/// </summary>
/// <param name="x"></param>
/// <param name="y"></param>
/// <param name="destXneg"></param>
/// <param name="destYneg"></param>
/// <param name="angle"></param>
void MoveForward(int x, int y, bool destXneg, bool destYneg, int angle)
{
    debugLog("Moving to x: " + (String)x + ", y: " + (String)y);
    Motors.setSpeeds(MOTOR_SPEED, MOTOR_SPEED);
    bool xInverse = CurrentX > (x - 5) && destXneg || CurrentX > (x + 5) && !destXneg ? true : false;
    bool yInverse = CurrentY > (y - 5) && destYneg || CurrentY > (y + 5) && !destYneg ? true : false;
    debugLog("X inverse: " + (String)xInverse + ", Y inverse: " + (String)yInverse);
    while (!VerifyCurrentDestination(destXneg, destYneg, x, y, xInverse, yInverse) && !ReachedDestination)
    {
        ProximitySensors.read();
        if (ProximitySensors.countsFrontWithLeftLeds() >= 4 || ProximitySensors.countsFrontWithRightLeds() >= 4)
        {
            Motors.setSpeeds(0, 0);
            TurnAngle(angle - 90, 'R');
            Motors.setSpeeds(MOTOR_SPEED, MOTOR_SPEED);
            delay(1000);
            TurnAngle(angle, 'L');
            Motors.setSpeeds(MOTOR_SPEED, MOTOR_SPEED);
            delay(1750);
            TurnAngle(angle + 89, 'L');
            Motors.setSpeeds(MOTOR_SPEED, MOTOR_SPEED);
            delay(1000);
            TurnAngle(angle, 'R');
            MoveTo(x, y);
        }
        if (0 < WheelEncoders.getCountsLeft() || 0 < WheelEncoders.getCountsRight()) {
            UpdatePosition();
        }
    }
    Motors.setSpeeds(0, 0);
    ReachedDestination = true;
    Navigating = false;
    debugLog("Done! Current pos x: " + (String)CurrentX + ", y: " + (String)CurrentY);
}

/// <summary>
/// Updates all the variables that is related to the gyroscope and position
/// </summary>
void GyroscopeUpdate()
{
    // Read the measurements from the gyro.
    IMU.readGyro();
    TurnRate = IMU.g.z - GyroOffset;

    // Figure out how much time has passed since the last update (dt)
    uint16_t m = micros();
    uint16_t dt = m - GyroLastUpdate;
    GyroLastUpdate = m;

    // Multiply dt by turnRate in order to get an estimation of how
    // much the robot has turned since the last update.
    // (angular change = angular velocity * time)
    int32_t d = (int32_t)TurnRate * dt;

    // The units of d are gyro digits times microseconds.  We need
    // to convert those to the units of turnAngle, where 2^29 units
    // represents 45 degrees.  The conversion from gyro digits to
    // degrees per second (dps) is determined by the sensitivity of
    // the gyro: 0.07 degrees per second per digit.
    //
    // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
    // = 14680064/17578125 unit/(digit*us)
    CurrentTurnAngle += (int64_t)d * 14680064 / 17578125; // TODO: Add some comment

    // Converts angle interval from [-180;180] to [0;360]
    TurnAngleDegrees = ((((int32_t)CurrentTurnAngle >> 16) * 360) >> 16);

    // Compensates for 360 degrees of freedom with angels that range from -360 to +360
    if (TurnAngleDegrees <= 0 && TurnAngleDegrees >= -180) {
        GlobalTheta = (360 + TurnAngleDegrees);
    }
    if (TurnAngleDegrees >= 0 && TurnAngleDegrees <= 180) {
        GlobalTheta = TurnAngleDegrees;
    }
}

/// <summary>
/// Updates all the variables relevant to position and wheel encoders
/// </summary>
void UpdatePosition()
{
    GyroscopeUpdate();
    float encL = WheelEncoders.getCountsLeft();
    float encR = WheelEncoders.getCountsRight();
    FWDVelocity = (((encL + encR) / 2) / 909.7 * (PI * WheelDiameter));
    VelocityR = ((encR) / 909.7 * (PI * WheelDiameter));
    VelocityL = ((encL) / 909.7 * (PI * WheelDiameter));
    DeltaD = (VelocityR + VelocityL) / 2;
    WheelEncoders.getCountsAndResetLeft();
    WheelEncoders.getCountsAndResetRight();
    DeltaX = (DeltaD * cos(GlobalTheta * (PI / 180)));
    DeltaY = (DeltaD * sin(GlobalTheta * (PI / 180)));

    CurrentX = CurrentX + DeltaX;
    CurrentY = CurrentY + DeltaY;

    LCD.clear();
    LCD.gotoXY(0, 0);
    LCD.print("x: " + (String)CurrentX);
    LCD.gotoXY(0, 1);
    LCD.print("y: " + (String)CurrentY);
}
