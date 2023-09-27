package frc.robot.subsystems.super_structure.pivot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.Constants.kSuperStructure.*;
import frc.robot.subsystems.super_structure.Errors.*;

public class PivotReal implements Pivot {

    /** Left */
    private final TalonFX leaderMotor;
    /** Right */
    private final TalonFX followerMotor;

    private final StatusSignal<Double> motorRots, motorVelo, motorAmps, motorVolts;

    private Boolean isHomed = false;
    private Boolean softLimitsEnabled = true;

    public PivotReal() {
        leaderMotor = new TalonFX(kPivot.LEFT_MOTOR_ID);
        followerMotor = new TalonFX(kPivot.RIGHT_MOTOR_ID);
        leaderMotor.getConfigurator().apply(getMotorConfig());
        followerMotor.getConfigurator().apply(getMotorConfig());

        motorRots = leaderMotor.getRotorPosition();
        motorVelo = leaderMotor.getRotorVelocity();
        motorAmps = leaderMotor.getStatorCurrent();
        motorVolts = leaderMotor.getSupplyVoltage();

        followerMotor.setControl(
            new Follower(kPivot.LEFT_MOTOR_ID, true)
        );
    }

    private Double mechDegreesToMotorRots(Double mechanismDegrees) {
        return (mechanismDegrees / 360.0) / kWrist.MOTOR_TO_MECHANISM_RATIO;
    }

    private TalonFXConfiguration getMotorConfig() {
        TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.Slot0.kP = kPivot.MOTOR_kP;
        motorCfg.Slot0.kI = kPivot.MOTOR_kI;
        motorCfg.Slot0.kD = kPivot.MOTOR_kD;

        motorCfg.MotionMagic.MotionMagicCruiseVelocity = kPivot.MAX_VELOCITY;
        motorCfg.MotionMagic.MotionMagicAcceleration = kPivot.MAX_ACCELERATION;
        motorCfg.MotionMagic.MotionMagicJerk = kPivot.MAX_JERK;

        motorCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = kPivot.ENABLE_SOFTLIMITS;
        motorCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = kPivot.ENABLE_SOFTLIMITS;
        motorCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = mechDegreesToMotorRots(
                kPivot.MAX_DEGREES);
        motorCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = mechDegreesToMotorRots(
                kPivot.MIN_DEGREES);

        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        motorCfg.MotorOutput.Inverted = kPivot.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        motorCfg.Voltage.PeakForwardVoltage = kPivot.VOLTAGE_COMP;
        motorCfg.Voltage.PeakReverseVoltage = -kPivot.VOLTAGE_COMP;

        return motorCfg;
    }

    @Override
    public Boolean setMechanismDegrees(Double degrees) {
        if (degrees > kPivot.MAX_DEGREES) {
            new SetpointTooHigh(kPivot.MAX_DEGREES, degrees).log();
            return false;
        } else if (degrees < kPivot.MIN_DEGREES) {
            new SetpointTooLow(kPivot.MIN_DEGREES, degrees).log();
            return false;
        }
        if (!this.softLimitsEnabled) {
            this.massSoftLimits(true, leaderMotor);
            this.softLimitsEnabled = true;
        }
        isHomed = false;
        var posControlRequest = new MotionMagicTorqueCurrentFOC(mechDegreesToMotorRots(degrees));
        this.leaderMotor.setControl(posControlRequest);
        return Math.abs(degrees - getMechanismDegrees()) < kPivot.TOLERANCE;
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        isHomed = false;
        var percentControlRequest = new DutyCycleOut(percentOut, true, false);
        this.leaderMotor.setControl(percentControlRequest);
    }

    @Override
    public void stopMechanism() {
        this.leaderMotor.setVoltage(0.0);
    }

    @Override
    public Double getMechanismDegrees() {
        return motorRots.refresh().getValue() * 360.0 * kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    @Override
    public Boolean homeMechanism() {
        if (isHomed) {
            return true;
        }
        if (this.softLimitsEnabled) {
            this.massSoftLimits(false, leaderMotor);
            this.softLimitsEnabled = false;
        }
        //TODO: move to 0 via pose then switch to open loop
        this.manualDriveMechanism(-0.15);
        if (motorAmps.refresh().getValue() > kPivot.CURRENT_PEAK_FOR_ZERO) {
            this.stopMechanism();
            this.leaderMotor.setRotorPosition(-15.0);//mechDegreesToMotorRots(kPivot.HOME_DEGREES - 10.0);
            isHomed = true;
        }
        return isHomed;
    }

    @Override
    public void periodic() {}

    @Override
    public void setupShuffleboard(ShuffleboardContainer tab) {
        tab.addNumber("Pivot Motor Rots", () -> motorRots.refresh().getValue());
        tab.addNumber("Pivot Motor Velo", () -> motorVelo.refresh().getValue());
        tab.addNumber("Pivot Motor Amps", () -> motorAmps.refresh().getValue());
        tab.addNumber("Pivot Motor Volts", () -> motorVolts.refresh().getValue());
        tab.addBoolean("Pivot Homed", () -> isHomed);
    }
}
