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
    public static final int kDriverControllerPort = 0;
    public static final double leftStickDeadband = 0.1;
    public static final double rightStickDeadband = 0.2;
  }

  public static class Swerve {

    //TODO {Maddox} Double check all physical constants and CAN ID's
    public static final int pigeonID = 33;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
    public static final String canBus = "McQDriveBus";

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.73); 
    public static final double wheelBase = Units.inchesToMeters(21.73); 
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    /* Swerve Kinematics 
        * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    //TODO {Maddox} Double check ratios
    public static final double driveGearRatio = (6.75 / 1.0);
    public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);

    /* Motor Inverts */
    //TODO {Maddox} Double check, may not be inverted correctly
    public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;

    /* Angle Encoder Invert */
    //TODO {Maddox} Double check, may not be inverted correctly
    public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    
    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
        * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.3;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    /* Drive Motor Characterization Values 
        * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5;
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0; 

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
    public static final class Mod0 {
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 21;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(NTpreferences.getAngleOffset(Module.u0));
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    public static final class Mod1 {
        public static final int driveMotorID = 5;
        public static final int angleMotorID = 6;
        public static final int canCoderID = 23;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(NTpreferences.getAngleOffset(Module.u1));
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    public static final class Mod2 {
        public static final int driveMotorID = 7;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 24;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(NTpreferences.getAngleOffset(Module.u2));
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    public static final class Mod3 {
        public static final int driveMotorID = 3;
        public static final int angleMotorID = 4;
        public static final int canCoderID = 22;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(NTpreferences.getAngleOffset(Module.u3));
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }
}
