package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Swerve;
import frc.robot.Robot;
import frc.robot.util.SwerveModuleConstants;

public class SwerveModule {
    private TalonFX mDriveMotor;
    private TalonFX mAngleMotor;
    private CANcoder mAngleEncoder;

    public int moduleNumber;
    private Rotation2d rotationOffset;
    private Rotation2d lastAngle = new Rotation2d();

    public SwerveModule(SwerveModuleConstants moduleConstants, int moduleNumber) {
        this.moduleNumber = moduleNumber;
        this.rotationOffset = moduleConstants.angleOffset;

        SmartDashboard.putNumber("Mod " + this.moduleNumber + " Rotation Offset", rotationOffset.getDegrees());

        mAngleEncoder = new CANcoder(moduleConstants.cancoderID, Swerve.CANBUS);
        configureCANcoder();

        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Swerve.CANBUS);
        configureDriveMotor();

        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, Swerve.CANBUS);
        configureAngleMotor();
    }

    public void configureDriveMotor() {
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        var mDriveConfig = new MotorOutputConfigs();
        mDriveConfig.Inverted = Swerve.DRIVE_MOTOR_INVERT;
        mDriveConfig.NeutralMode = Swerve.DRIVE_NEUTRAL_MODE;
        mDriveMotor.getConfigurator().apply(mDriveConfig);
    }

    public void configureAngleMotor() {
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        var mAngleConfig = new TalonFXConfiguration();
        mAngleConfig.MotorOutput.Inverted = Swerve.ANGLE_MOTOR_INVERT;
        mAngleConfig.MotorOutput.NeutralMode = Swerve.ANGLE_NEUTRAL_MODE;
        mAngleConfig.Slot0.kP = Swerve.ANGLE_KP;
        mAngleConfig.Slot0.kI = Swerve.ANGLE_KI;
        mAngleConfig.Slot0.kD = Swerve.ANGLE_KD;
        mAngleConfig.Feedback.FeedbackRemoteSensorID = mAngleEncoder.getDeviceID();
        mAngleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        mAngleConfig.Feedback.RotorToSensorRatio = Swerve.ANGLE_GEAR_RATIO;
        mAngleConfig.Feedback.SensorToMechanismRatio = 1.0;
        mAngleConfig.ClosedLoopGeneral.ContinuousWrap = true;
        mAngleMotor.getConfigurator().apply(mAngleConfig);
    }

    public void configureCANcoder() {
        mAngleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
        var CANCoderConfig = new CANcoderConfiguration();
        CANCoderConfig.MagnetSensor.MagnetOffset = -rotationOffset.getRotations();
        mAngleEncoder.getConfigurator().apply(CANCoderConfig);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(mAngleEncoder.getAbsolutePosition().getValue());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                mDriveMotor.getRotorPosition().getValue() * (Swerve.WHEEL_CIRCUMFERENCE / Swerve.DRIVE_GEAR_RATIO),
                Rotation2d.fromRotations(mAngleMotor.getRotorPosition().getValue() / Swerve.ANGLE_GEAR_RATIO));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = optimize(desiredState, getAbsoluteAngle());
        SmartDashboard.putNumber("Mod " + this.moduleNumber + " Desired State", desiredState.angle.getDegrees());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double currAngle = scope0To360(currentAngle.getDegrees());
        double targetAngle = desiredState.angle.getDegrees() + 180;
        double targetSpeed = desiredState.speedMetersPerSecond;
        double targetAngleDelta = targetAngle - currAngle;
        if (Math.abs(targetAngleDelta) > 90){
            targetSpeed = -targetSpeed;
            targetAngle = targetAngleDelta > 90 ? targetAngle - 180 : targetAngle + 180;
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    } 
    
    public double scope0To360(double angle) {
        if (angle < 0) {
            angle = 360-(Math.abs(angle)%360);
        } 
        else {
            angle %= 360;
        }
        return angle;
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(mAngleMotor.getRotorPosition().getValue());
    }

    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Swerve.MAX_SPEED * 0.01)) ? lastAngle
                : desiredState.angle;
        var controlRequest = new PositionDutyCycle(angle.getRotations());
        mAngleMotor.setControl(controlRequest);
        lastAngle = angle;
    }

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        double percentOutput = desiredState.speedMetersPerSecond / Swerve.MAX_SPEED;
        var controlRequest = new DutyCycleOut(percentOutput);
        if (!isOpenLoop) {
            var slot0Config = new Slot0Configs();
            slot0Config.kP = Swerve.DRIVE_KP;
            slot0Config.kI = Swerve.DRIVE_KI;
            slot0Config.kD = Swerve.DRIVE_KD;
            mDriveMotor.getConfigurator().apply(slot0Config);
        }
        mDriveMotor.setControl(controlRequest);
    }

    private double driveRotationsToMeters(double rotations) {
        return (rotations / Swerve.DRIVE_GEAR_RATIO) * (Swerve.WHEEL_DIAMETER * Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveRotationsToMeters(mDriveMotor.getVelocity().getValue()),
                getAbsoluteAngle());
    }
}
