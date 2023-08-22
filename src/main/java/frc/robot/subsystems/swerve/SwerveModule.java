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
import frc.robot.util.CTREModuleState;
import frc.robot.util.SwerveModuleConstants;

public class SwerveModule {
    private TalonFX mDriveMotor;
    private TalonFX mAngleMotor;
    private CANcoder angleEncoder;

    public int moduleNumber;
    private Rotation2d rotationOffset;
    private Rotation2d lastAngle = new Rotation2d();

    public SwerveModule(SwerveModuleConstants moduleConstants, int moduleNumber) {
        this.moduleNumber = moduleNumber;
        this.rotationOffset = moduleConstants.angleOffset;

        SmartDashboard.putNumber("Mod " + this.moduleNumber + " Rotation Offset", rotationOffset.getDegrees());

        angleEncoder = new CANcoder(moduleConstants.cancoderID, Swerve.CANBUS);
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
        mAngleConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        mAngleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        mAngleConfig.Feedback.RotorToSensorRatio = Swerve.ANGLE_GEAR_RATIO;
        mAngleConfig.Feedback.SensorToMechanismRatio = 1.0;
        mAngleConfig.ClosedLoopGeneral.ContinuousWrap = true;
        mAngleMotor.getConfigurator().apply(mAngleConfig);
    }

    public void configureCANcoder() {
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
        var CANCoderConfig = new CANcoderConfiguration();
        SmartDashboard.putNumber("bleh" + this.moduleNumber, rotationOffset.getDegrees());
        CANCoderConfig.MagnetSensor.MagnetOffset = -rotationOffset.getRotations();
        angleEncoder.getConfigurator().apply(CANCoderConfig);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                mDriveMotor.getRotorPosition().getValue() * (Swerve.WHEEL_CIRCUMFERENCE / Swerve.DRIVE_GEAR_RATIO),
                Rotation2d.fromRotations(mAngleMotor.getRotorPosition().getValue() / Swerve.ANGLE_GEAR_RATIO));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getAngle());
        SmartDashboard.putNumber("Mod " + this.moduleNumber + " Desired State", desiredState.angle.getDegrees());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
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
                getAngle());
    }
}
