package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.kSwerve;
import frc.robot.Robot;
import frc.robot.util.SwerveModuleConstants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder angleEncoder;

    public final int moduleNumber;
    private final Rotation2d rotationOffset;
    private final Translation2d moduleChassisPose;
    private Rotation2d lastAngle = new Rotation2d();

    private final StatusSignal<Double> encoderPosStatus;
    private final StatusSignal<Double> encoderVeloStatus;

    public SwerveModule(SwerveModuleConstants moduleConstants, int moduleNumber) {
        this.moduleNumber = moduleNumber;
        this.rotationOffset = moduleConstants.angleOffset;
        this.moduleChassisPose = moduleConstants.moduleChassisPose;

        SmartDashboard.putNumber("Mod " + this.moduleNumber + " Rotation Offset", rotationOffset.getDegrees());

        
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
        driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        var mDriveConfig = new MotorOutputConfigs();
        mDriveConfig.Inverted = kSwerve.DRIVE_MOTOR_INVERT;
        mDriveConfig.NeutralMode = kSwerve.DRIVE_NEUTRAL_MODE;
        driveMotor.getConfigurator().apply(mDriveConfig);
    }

    public void configureAngleMotor() {
        angleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        var mAngleConfig = new TalonFXConfiguration();
        mAngleConfig.MotorOutput.Inverted = kSwerve.ANGLE_MOTOR_INVERT;
        mAngleConfig.MotorOutput.NeutralMode = kSwerve.ANGLE_NEUTRAL_MODE;
        mAngleConfig.Slot0.kP = kSwerve.ANGLE_KP;
        mAngleConfig.Slot0.kI = kSwerve.ANGLE_KI;
        mAngleConfig.Slot0.kD = kSwerve.ANGLE_KD;
        mAngleConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        mAngleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        mAngleConfig.Feedback.RotorToSensorRatio = kSwerve.ANGLE_GEAR_RATIO;
        mAngleConfig.Feedback.SensorToMechanismRatio = 1.0;
        mAngleConfig.ClosedLoopGeneral.ContinuousWrap = true;
        angleMotor.getConfigurator().apply(mAngleConfig);
    }

    public void configureCANcoder() {
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
        var CANCoderConfig = new CANcoderConfiguration();
        CANCoderConfig.MagnetSensor.MagnetOffset = -rotationOffset.getRotations();
        angleEncoder.getConfigurator().apply(CANCoderConfig);
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
        SmartDashboard.putNumber("Mod " + this.moduleNumber + " Desired State", desiredState.angle.getDegrees());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public Rotation2d getAngle() {
        BaseStatusSignal.waitForAll(0, encoderPosStatus, encoderVeloStatus);
        return Rotation2d.fromRotations(
                BaseStatusSignal.getLatencyCompensatedValue(encoderPosStatus, encoderVeloStatus));
    }

    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_SPEED * 0.01)) ? lastAngle
                : desiredState.angle;

        var controlRequest = new PositionDutyCycle(angle.getRotations());
        angleMotor.setControl(controlRequest);
        lastAngle = angle;
    }

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        double percentOutput = desiredState.speedMetersPerSecond / kSwerve.MAX_SPEED;
        var controlRequest = new DutyCycleOut(percentOutput);
        if (!isOpenLoop) {
            var slot0Config = new Slot0Configs();
            slot0Config.kP = kSwerve.DRIVE_KP;
            slot0Config.kI = kSwerve.DRIVE_KI;
            slot0Config.kD = kSwerve.DRIVE_KD;
            driveMotor.getConfigurator().apply(slot0Config);
        }
        driveMotor.setControl(controlRequest);
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
