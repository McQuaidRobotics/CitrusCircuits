package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.NTpreferences;
import frc.robot.util.SwerveModuleConstants;
import frc.robot.util.NTpreferences.Module;

public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double LEFT_JOYSTICK_DAEDBAND = 0.1;
    public static final double RIGHT_JOYSTICK_DEADBAND = 0.2;
  }

  public static class Swerve {
    public static final int PIGEON_ID = 33;
    public static final boolean INVERT_GYRO = false;
    public static final String CANBUS = "McQDriveBus";

    /* Drivetrain Constants */
    public static final double TRACK_WIDTH = Units.inchesToMeters(21.73); 
    public static final double WHEEL_BASE = Units.inchesToMeters(21.73); 
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    /* Swerve Kinematics 
        * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    /* Module Gear Ratios */
    public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0);
    public static final double ANGLE_GEAR_RATIO = ((150.0 / 7.0) / 1.0);

    /* Motor Inverts */
    public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

    /* Swerve Current Limiting */
    public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
    public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
    public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

    
    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
    public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
        * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /* Angle Motor PID Values */
    public static final double ANGLE_KP = 2;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.0;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.05;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;

    /* Drive Motor Characterization Values 
        * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double DRIVE_KS = (0.32 / 12);
    public static final double DRIVE_KV = (1.51 / 12);
    public static final double DRIVE_KA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double MAX_SPEED = 4.5;
    /** Radians per Second */
    public static final double MAX_ANGULAR_VELOCITY = 10.0; 

    /* Neutral Modes */
    public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final class Mod0 {
        public static final int DRIVE_MOTOR_ID = 1;
        public static final int ANGLE_MOTOR_ID = 2;
        public static final int CANCODER_ID = 21;
        public static final Rotation2d ROTATION_OFFSET = Rotation2d.fromRotations(NTpreferences.getRotationOffset(Module.u0));
        public static final SwerveModuleConstants CONSTANTS = 
            new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ROTATION_OFFSET);
    }
    public static final class Mod1 {
        public static final int DRIVE_MOTOR_ID = 5;
        public static final int ANLGE_MOTOR_ID = 6;
        public static final int CANCODER_ID = 23;
        public static final Rotation2d ROTATION_OFFSET = Rotation2d.fromRotations(NTpreferences.getRotationOffset(Module.u1));
        public static final SwerveModuleConstants CONSTANTS = 
            new SwerveModuleConstants(DRIVE_MOTOR_ID, ANLGE_MOTOR_ID, CANCODER_ID, ROTATION_OFFSET);
    }
    public static final class Mod2 {
        public static final int DRIVE_MOTOR_ID = 7;
        public static final int ANGLE_MOTOR_ID = 8;
        public static final int CANCODER_ID = 24;
        public static final Rotation2d ROTATION_OFFSET = Rotation2d.fromRotations(NTpreferences.getRotationOffset(Module.u2));
        public static final SwerveModuleConstants CONSTANTS = 
            new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ROTATION_OFFSET);
    }
    
    public static final class Mod3 {
        public static final int DRIVE_MOTOR_ID = 3;
        public static final int ANGLE_MOTOR_ID = 4;
        public static final int CANCODER_ID = 22;
        public static final Rotation2d ROTATION_OFFSET = Rotation2d.fromRotations(NTpreferences.getRotationOffset(Module.u3));
        public static final SwerveModuleConstants CONSTANTS = 
            new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ROTATION_OFFSET);
    }
  }
}
