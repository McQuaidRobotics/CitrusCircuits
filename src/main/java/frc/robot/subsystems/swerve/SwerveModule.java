package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.kSwerve;
import frc.robot.util.SwerveModuleConstants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder angleEncoder;

    public final int moduleNumber;
    private final Rotation2d rotationOffset;
    @SuppressWarnings("unused")
    private final Translation2d moduleChassisPose;
    private Rotation2d lastAngle = new Rotation2d();

    private final StatusSignal<Double> encoderPosStatus;
    private final StatusSignal<Double> encoderVeloStatus;

    public SwerveModule(SwerveModuleConstants moduleConstants, int moduleNumber) {
        this.moduleNumber = moduleNumber;
        this.rotationOffset = moduleConstants.angleOffset;
        this.moduleChassisPose = moduleConstants.moduleChassisPose;
        
        driveMotor = new TalonFX(moduleConstants.driveMotorID, kSwerve.CANBUS);
        angleMotor = new TalonFX(moduleConstants.angleMotorID, kSwerve.CANBUS);
        angleEncoder = new CANcoder(moduleConstants.cancoderID, kSwerve.CANBUS);

        configureDriveMotor();
        configureAngleMotor();
        configureCANcoder();

        this.encoderPosStatus = angleEncoder.getAbsolutePosition();
        this.encoderVeloStatus = angleEncoder.getVelocity();
    }

    public void configureDriveMotor() {
        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = kSwerve.DRIVE_ENABLE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLimit = kSwerve.DRIVE_CONTINUOUS_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentThreshold = kSwerve.DRIVE_PEAK_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyTimeThreshold = kSwerve.DRIVE_PEAK_CURRENT_DURATION;
        driveConfig.MotorOutput.Inverted = kSwerve.DRIVE_MOTOR_INVERT;
        driveConfig.MotorOutput.NeutralMode = kSwerve.DRIVE_NEUTRAL_MODE;
        driveConfig.Slot0.kP = kSwerve.DRIVE_KP;
        driveConfig.Slot0.kI = kSwerve.DRIVE_KI;
        driveConfig.Slot0.kD = kSwerve.DRIVE_KD;
        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = kSwerve.OPEN_LOOP_RAMP;
        driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = kSwerve.CLOSED_LOOP_RAMP;

        driveMotor.getConfigurator().apply(driveConfig);
    }

    public void configureAngleMotor() {
        var angleConfig = new TalonFXConfiguration();
        angleConfig.MotorOutput.Inverted = kSwerve.ANGLE_MOTOR_INVERT;
        angleConfig.MotorOutput.NeutralMode = kSwerve.ANGLE_NEUTRAL_MODE;
        angleConfig.Slot0.kP = kSwerve.ANGLE_KP;
        angleConfig.Slot0.kI = kSwerve.ANGLE_KI;
        angleConfig.Slot0.kD = kSwerve.ANGLE_KD;
        angleConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        angleConfig.Feedback.RotorToSensorRatio = kSwerve.ANGLE_GEAR_RATIO;
        angleConfig.Feedback.SensorToMechanismRatio = 1.0;
        angleConfig.ClosedLoopGeneral.ContinuousWrap = true;

        angleMotor.getConfigurator().apply(angleConfig);
    }

    public void configureCANcoder() {
        var canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        canCoderConfig.MagnetSensor.SensorDirection = kSwerve.CANCODER_INVERT;
        canCoderConfig.MagnetSensor.MagnetOffset = -rotationOffset.getRotations();

        angleEncoder.getConfigurator().apply(canCoderConfig);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveMotor.getRotorPosition().getValue() * (kSwerve.WHEEL_CIRCUMFERENCE / kSwerve.DRIVE_GEAR_RATIO),
                getAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = optimize(desiredState, getAngle());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(
                BaseStatusSignal.getLatencyCompensatedValue(encoderPosStatus.refresh(), encoderVeloStatus.refresh()));
    }

    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_SPEED * 0.01)) ? lastAngle
                : desiredState.angle;

        var controlRequest = new PositionDutyCycle(angle.getRotations());
        angleMotor.setControl(controlRequest);
        lastAngle = angle;
    }

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kSwerve.MAX_SPEED;
            var controlRequest = new DutyCycleOut(percentOutput);
            driveMotor.setControl(controlRequest);
        } else {
            double rps = Math.min(desiredState.speedMetersPerSecond, kSwerve.MAX_SPEED)
                / (kSwerve.WHEEL_CIRCUMFERENCE / kSwerve.DRIVE_GEAR_RATIO);
            var veloRequest = new VelocityTorqueCurrentFOC(rps);
            driveMotor.setControl(veloRequest);
        }
    }

    private double driveRotationsToMeters(double rotations) {
        return (rotations / kSwerve.DRIVE_GEAR_RATIO) * (kSwerve.WHEEL_DIAMETER * Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveRotationsToMeters(driveMotor.getVelocity().getValue()),
                getAngle());
    }

    public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double currAngle = scope0To360(currentAngle.getDegrees());
        double targetAngle = desiredState.angle.getDegrees() + 180;
        double targetSpeed = desiredState.speedMetersPerSecond;
        double targetAngleDelta = targetAngle - currAngle;
        if (Math.abs(targetAngleDelta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = targetAngleDelta > 90 ? targetAngle - 180 : targetAngle + 180;
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    public double scope0To360(double angle) {
        if (angle < 0) {
            angle = 360 - (Math.abs(angle) % 360);
        } else {
            angle %= 360;
        }
        return angle;
    }
}
