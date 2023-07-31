package frc.robot.subsystems.swerve;

import java.io.Console;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.CTREModuleState;
import frc.robot.util.SwerveModuleConstants;

public class SwerveModule {
    private TalonFX mDriveMotor;
    private TalonFX mAngleMotor;
    private CANcoder angleEncoder;

    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    public SwerveModule(SwerveModuleConstants moduleConstants, int moduleNumber) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        angleEncoder = new CANcoder(moduleConstants.cancoderID, Constants.Swerve.canBus);
        configureCANcoder();

        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.Swerve.canBus);
        configureDriveMotor();

        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.Swerve.canBus);
        configureAngleMotor();
    }

    public void configureDriveMotor() {
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        var mDriveConfig = new MotorOutputConfigs();
        mDriveConfig.Inverted = Constants.Swerve.driveMotorInvert;
        mDriveConfig.NeutralMode = Constants.Swerve.driveNeutralMode;
        mDriveMotor.getConfigurator().apply(mDriveConfig);
    }

    public void configureAngleMotor() {
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        var mAngleConfig = new MotorOutputConfigs();
        mAngleConfig.Inverted = Constants.Swerve.angleMotorInvert;
        mAngleConfig.NeutralMode = Constants.Swerve.angleNeutralMode;
        resetToAbsolute();
        mAngleMotor.getConfigurator().apply(mAngleConfig);
    }

    public void configureCANcoder() {
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute() {
        double absolutePosition = getCanCoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setRotorPosition(absolutePosition);
    }

    public SwerveModulePosition getPosition() {
        //TODO {Maddox} May need to fix conversion, needs to convert motor rotations to a distance in meters, 
        //and motor rotation to an angle in degrees inside a rotation2d object
        return new SwerveModulePosition(
            mDriveMotor.getRotorPosition().getValue() * (Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio),
            Rotation2d.fromRotations(mAngleMotor.getRotorPosition().getValue() / Constants.Swerve.angleGearRatio)
        );
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getAngle());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public Rotation2d getAngle() {
        //TODO {Maddox} Double check conversion
        return Rotation2d.fromRotations(mAngleMotor.getRotorPosition().getValue() / Constants.Swerve.angleGearRatio);
    }

    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle;

        //TODO {Maddox} Double check conversion
        var controlRequest = new PositionDutyCycle(desiredState.angle.getRotations() * Constants.Swerve.angleGearRatio);
        mAngleMotor.setControl(controlRequest);
        lastAngle = angle;
    }

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        //TODO {Maddox} Don't really understand the math behind how this percent output is calculated
        double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
        var controlRequest = new DutyCycleOut(percentOutput);
        if (!isOpenLoop) {
            var slot0Config = new Slot0Configs();
            slot0Config.kP = Constants.Swerve.driveKP;
            slot0Config.kI = Constants.Swerve.driveKI;
            slot0Config.kD = Constants.Swerve.driveKD;
            mDriveMotor.getConfigurator().apply(slot0Config);
        }
        mDriveMotor.setControl(controlRequest);
    }

    private double driveRotationsToMeters(double rotations) {
        //TODO {Maddox} Double check conversions
        return (rotations / Constants.Swerve.driveGearRatio) * (Constants.Swerve.wheelDiameter * Math.PI);
    }

    public SwerveModuleState getState() {
        //TODO {Maddox} Double check conversions
        return new SwerveModuleState(
            driveRotationsToMeters(mDriveMotor.getVelocity().getValue()),
            getAngle()
        );
    }
}
