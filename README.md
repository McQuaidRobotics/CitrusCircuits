# Universal Offset Swerve Base </br>

**Basic Swerve Code for a Swerve Module using Falcon Motors, a CTRE CANCoder, and a CTRE Pigeon Gyro** </br>
This code was designed with Swerve Drive Specialties MK3, MK4, and MK4i style modules in mind, but should be easily adaptable to other styles of modules. Keep in mind that this code is also using the CTRE Pheonix v6 vendordeps, which means your robot must be running CTRE Pheonix v6 firmware to work!</br>

**Setting Constants**
----
The following things must be adjusted to your robot and module's specific constants in the Constants.java file (all distance units must be in meters, and rotation units in radians):</br>

1. Gyro Settings: ```pigeonID``` and ```invertGyro``` (ensure that the gyro rotation is CCW+ (Counter Clockwise Positive)

2. ```trackWidth```: Center to Center distance of left and right modules in meters.

3. ```wheelBase```: Center to Center distance of front and rear module wheels in meters.

4. ```wheelCircumference```: Cirumference of the wheel (including tread) in meters. <br><b>If you are using a supported module, this value will be automatically set.</b>

5. ```driveGearRatio```: Total gear ratio for the drive motor. <br>

6. ```angleGearRatio```: Total gear ratio for the angle motor. <br>

7. ```angleMotorInvert```: Must be set to CW+. <br>

8. ```driveMotorInvert```: Must be set to CCW+.</br>

9. ```Module Specific Constants```: set the Can Id's of the motors and CANCoders for the respective modules, see the next step for setting offsets.

10. Setting Offsets
    * For finding the offsets, rotate each swerve wheel to roughy a 45 degree angle so that the bevel gear of each module is facing outwards.
    * When all the wheels are rotated, place the <b>G A F U S O 😏</b> ("Get Angle For Universal Swerve Offset", this acronym was created back when we used v5 with Pheonix Tuner which used angles/degrees for reporting absolute positions, now we use v6 and Tuner X which reports absolute position in rotations not angles/degrees) into the corner of the module so that the triangular shape of the GAFUSO fits with the triangular face of the wheel's module.
        * The GAFUSO is a 3d print. It is a ```.stl``` file that is located in the <b> util </b> folder.
    * Slightly rotate the wheel until the face of the GAFUSO is flush against the fork of the swerve module. Double check that the bevel gear is facing outwards.
    * Open Tuner X (the new version of Pheonix Tuner) and run a Self Test Snapshot on all the CAN-Coders for each module. The Absolute Position (which will be in rotations) is your offset. <b> DO NOT USE NETWORK TABLES TO FIND THE OFFSET! </b> It can be innacurate and inconsistent. 
    * Once you have your offsets, connect to your robot's roboRIO and open Smart Dashboard. Using Smart Dashboard add 4 keys to the preferences of your RIO, each key should be a key type that holds a number. The names of the 4 keys should see like the following example:
        * ```Module # Offset Universal``` - Where # represents the module numbers from 0-3
    * Once the Universal Offset preferences are set in your roboRIO's Networktable Preferences, open Shuffleboard or Smart Dashboard and edit the preference values that corespond to the module the offset was recorded from. If you are using Shuffleboard you will find these preference values underneath ```preferences``` in Networktables.
    <br>The module preferences should be named ```Module # Offset Universal``` if you set them up correctly. Changing the value of this changes the offset (to see the offsets take affect restart your robot code).

11. Angle Motor PID Values: <br><b>These will be set for you already inside of Constants.java, however if you prefer a more or less aggressive response, you can use the below instructions to tune the PID values.</b> 
    * To tune start with a low P value (0.01).
    * Multiply by 10 until the module starts oscilating around the set point
    * Scale back by searching for the value (for example, if it starts oscillating at a P of 10, then try (10 -> 5 -> 7.5 -> etc)) until the module doesn't oscillate around the setpoint.
    * If there is any overshoot you can add in some D by repeating the same process, leave at 0 if not. Always leave I at 0.

12. ```maxSpeed```: In Meters Per Second. ```maxAngularVelocity```: In Radians Per Second. For these you can use the theoretical values, but it is better to physically drive the robot and find the actual max values.

13. Get the drive characterization values (KS, KV, KA) by using the WPILib characterization tool, found [here](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html). You will need to lock your modules straight forward, and complete the characterization as if it was a standard tank drive.

14. ```driveKP```: 
<br>After completeing characterization and inserting the KS, KV, and KA values into the code, tune the drive motor kP until it doesn't overshoot and doesnt oscilate around a target velocity.
<br>Leave ```driveKI```, ```driveKD```, and ```driveKF``` at 0.0.




**Controller Mappings**
----
This code is natively setup to use an xbox controller to control the swerve drive. </br>
* Left Stick: Translation Control (forwards and sideways movement)
* Right Stick: Rotation Control </br>
* Start Button: Zero Gyro (useful if the gyro drifts mid match, just rotate the robot forwards, and press START to rezero the gryo)
* Left Bumper: Switches to Robot Centric Control while held
