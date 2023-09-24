package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.NTpreferences;
import frc.robot.util.SwerveModuleConstants;
import frc.robot.util.NTpreferences.Module;

public final class Constants {
    public static final double dPlaceholder = 0.0;
    public static final int iPlaceholder = 0;
    public static final boolean bPlaceholder = false;

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double LEFT_JOYSTICK_DAEDBAND = 0.1;
        public static final double RIGHT_JOYSTICK_DEADBAND = 0.2;
    }

    public static class kSuperStructure {

        public static final class kWrist {
            public static final int MOTOR_ID = 12;
            public static final double MOTOR_kP = 1.0;
            public static final double MOTOR_kI = 0.0;
            public static final double MOTOR_kD = 0.0;

            public static final double MOTOR_kS = dPlaceholder;
            public static final double MOTOR_kV = dPlaceholder;

            /** For every {@value} rotations of the motor the mechanism moves 1 rotation */
            // motor -> (10t -> 72t) -> (20t -> 72t) -> (24t -> 48t)
            public static final double MOTOR_TO_MECHANISM_RATIO = (10.0 / 72.0) * (20.0 / 72.0) * (24.0 / 48.0);

            public static final double MAX_VELOCITY = 100;
            public static final double MAX_ACCELERATION = 750;
            public static final double MAX_JERK = 5000;

            public static final boolean INVERTED = true;

            /**
             * Zero is parallel with the elevator
             * <p>
             * Alias for {@link Specs#WRIST_MAX_ANGLE}
             */
            public static final double MAX_DEGREES = Specs.WRIST_MAX_ANGLE;
            /**
             * Zero is parallel with the floor
             * <p>
             * Alias for {@link Specs#WRIST_MIN_ANGLE}
             */
            public static final double MIN_DEGREES = Specs.WRIST_MIN_ANGLE;

            public static final boolean ENABLE_SOFTLIMITS = false;

            /**
             * the ammount of current the motor needs to pull to
             * be recognized at mechanical limit.
             */
            public static final double CURRENT_PEAK_FOR_ZERO = 15.0;
        }

        public static final class kIntake {
            public static final int MOTOR_ID = 11;

            public static final boolean INVERTED = true;
            public static final boolean BREAK_DEFAULT = true;

            public static final double CURRENT_LIMIT = 40.0;
        }

        public static final class kPivot {
            public static final int LEFT_MOTOR_ID = 13;
            public static final int RIGHT_MOTOR_ID = 14;

            public static final double MOTOR_kP = 1.0;
            public static final double MOTOR_kI = 0;
            public static final double MOTOR_kD = 0;

            public static final double MAX_VELOCITY = 100;
            public static final double MAX_ACCELERATION = 750;
            public static final double MAX_JERK = 5000;

            /**
             * Zero is parallel with the floor
             * <p>
             * Alias for {@link Specs#PIVOT_MIN_ANGLE}
             */
            public static final double MIN_DEGREES = Specs.PIVOT_MIN_ANGLE;
            /**
             * Zero is parallel with the floor
             * <p>
             * Alias for {@link Specs#PIVOT_MAX_ANGLE}
             */
            public static final double MAX_DEGREES = Specs.PIVOT_MAX_ANGLE;

            public static final boolean ENABLE_SOFTLIMITS = false;

            /** For every {@value} rotations of the motor the mechanism moves 1 rotation */
            // motor -> gbx(25:1) -> (30t -> 64t) -> (12t -> 54t)
            public static final double MOTOR_TO_MECHANISM_RATIO = (1.0 / 25.0) * (30.0 / 64.0) * (12.0 / 54.0);

            public static final boolean INVERTED = false;

            /**
             * The max voltage of the motors to behave more predictably
             * throughout the match.
             */
            public static final double VOLTAGE_COMP = 11.8;

            /**
             * The ammount of current the motor needs to pull to
             * be recognized at mechanical limit.
             */
            public static final double CURRENT_PEAK_FOR_ZERO = dPlaceholder;
        }

        public static final class kElevator {
            public static final int ELEVATOR_LEFT_MOTOR_ID = 16;
            public static final int ELEVATOR_RIGHT_MOTOR_ID = 17;

            public static final double MOTOR_kP = 1.0;
            public static final double MOTOR_kD = 0.0;
            public static final double MOTOR_kI = 0.0;
            public static final double MOTOR_kS = dPlaceholder;
            public static final double MOTOR_kV = dPlaceholder;

            public static final boolean ENABLE_SOFTLIMITS = false;

            public static final boolean INVERTED = bPlaceholder;

            public static final double MAX_VELOCITY = 100;
            public static final double MAX_ACCELERATION = 750;
            public static final double MAX_JERK = 5000;

            public static final double MOTOR_TO_MECHANISM_RATIO = 1.0/3.0;
            public static final double MECHANISM_DIAMETER_METERS = 0.042164;

            /**Alias for {@link Specs#ELEVATOR_MIN_METERS} */
            public static final double HOME_METERS = Specs.ELEVATOR_MIN_METERS;

            /**Alias for {@link Specs#ELEVATOR_MIN_METERS} */
            public static final double MIN_METERS = Specs.ELEVATOR_MIN_METERS;
            /**Alias for {@link Specs#ELEVATOR_MAX_METERS} */
            public static final double MAX_METERS = Specs.ELEVATOR_MAX_METERS;
        }

        public static final class Specs {
            public static final double WRIST_MASS_GRAMS = 3250;
            public static final double ARM_MASS_GRAMS = 9500;

            public static final double ELEVATOR_MIN_METERS = 0.51562;
            public static final double ELEVATOR_MAX_METERS = 1.39192;

            public static final double PIVOT_MIN_ANGLE = -7.0;
            public static final double PIVOT_MAX_ANGLE = 90.0;

            //arbitrary min, not mechanical limit(which is ~15 less)
            public static final double WRIST_MIN_ANGLE = -66.04;
            public static final double WRIST_MIN_ANGLE_FLOOR = 21.1;
            public static final double WRIST_MAX_ANGLE = 149.39;

            public static final Transform3d PIVOT_OFFSET_METERS = new Transform3d(
                    new Translation3d(0.0, -0.232953, -0.252125),
                    new Rotation3d());

            public static final double RIM_ABOVE_FLOOR_METERS = 0.09525;
        }

    }

    public static class kSwerve {
        public static final int PIGEON_ID = 33;
        public static final boolean INVERT_GYRO = false;
        public static final String CANBUS = "McQDriveBus";

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.73);
        public static final double WHEEL_BASE = Units.inchesToMeters(21.73);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                Mod0.CHASSIS_OFFSET, Mod1.CHASSIS_OFFSET, Mod2.CHASSIS_OFFSET, Mod3.CHASSIS_OFFSET);

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

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
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

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
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
            public static final Rotation2d ROTATION_OFFSET = NTpreferences.getRotationOffset(Module.u0);
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ROTATION_OFFSET, CHASSIS_OFFSET);
        }

        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANLGE_MOTOR_ID = 4;
            public static final int CANCODER_ID = 22;
            public static final Rotation2d ROTATION_OFFSET = NTpreferences.getRotationOffset(Module.u1);
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANLGE_MOTOR_ID, CANCODER_ID, ROTATION_OFFSET, CHASSIS_OFFSET);
        }

        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CANCODER_ID = 23;
            public static final Rotation2d ROTATION_OFFSET = NTpreferences.getRotationOffset(Module.u2);
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ROTATION_OFFSET, CHASSIS_OFFSET);
        }

        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CANCODER_ID = 24;
            public static final Rotation2d ROTATION_OFFSET = NTpreferences.getRotationOffset(Module.u3);
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ROTATION_OFFSET, CHASSIS_OFFSET);
        }
    }
}