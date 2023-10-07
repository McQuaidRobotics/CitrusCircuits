package frc.robot.subsystems.super_structure.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.Constants.kSuperStructure.*;
import frc.robot.subsystems.super_structure.Errors.*;

public class WristReal implements Wrist {

    private final TalonFX wristMotor, intakeMotor;

    private final StatusSignal<Double> wristMotorRots, wristMotorVelo, wristMotorAmps, wristMotorVolts;
    private final StatusSignal<Double> intakeMotorAmps, intakeMotorVolts, intakeMotorVelo;

    private Boolean isStowed = false;
    private Double cachedWristDegrees, cachedIntakeVolts = 0.0;

    public WristReal(Double startingDegrees) {
        wristMotor = new TalonFX(kWrist.MOTOR_ID);
        wristMotor.getConfigurator().apply(getWristMotorConfig());

        wristMotorRots = wristMotor.getRotorPosition();
        wristMotorVelo = wristMotor.getRotorVelocity();
        wristMotorAmps = wristMotor.getStatorCurrent();
        wristMotorVolts = wristMotor.getSupplyVoltage();

        wristMotor.setRotorPosition(mechDegreesToMotorRots(startingDegrees));
        cachedWristDegrees = startingDegrees;

        intakeMotor = new TalonFX(kIntake.MOTOR_ID);
        intakeMotor.getConfigurator().apply(getIntakeMotorConfig());

        intakeMotorAmps = intakeMotor.getStatorCurrent();
        intakeMotorVolts = intakeMotor.getSupplyVoltage();
        intakeMotorVelo = intakeMotor.getVelocity();
    }

    private Double mechDegreesToMotorRots(Double mechanismDegrees) {
        return (mechanismDegrees / 360.0) / kWrist.MOTOR_TO_MECHANISM_RATIO;
    }

    private Double motorRotsToMechDegrees(Double motorRots) {
        return motorRots * 360.0 * kWrist.MOTOR_TO_MECHANISM_RATIO;
    }

    /**
     * Constructs a TalonFXConfiguration object only from values
     * from {@link frc.robot.Constants.kWrist}
     * 
     * @return the TalonFXConfiguration object
     */
    private TalonFXConfiguration getWristMotorConfig() {
        TalonFXConfiguration wristMotorCfg = new TalonFXConfiguration();
        wristMotorCfg.Slot0.kP = kWrist.MOTOR_kP;
        wristMotorCfg.Slot0.kI = kWrist.MOTOR_kI;
        wristMotorCfg.Slot0.kD = kWrist.MOTOR_kD;
        // wristMotorCfg.Slot0.kS = kWrist.MOTOR_kS;
        // wristMotorCfg.Slot0.kV = kWrist.MOTOR_kV;

        wristMotorCfg.MotionMagic.MotionMagicCruiseVelocity = kWrist.MAX_VELOCITY;
        wristMotorCfg.MotionMagic.MotionMagicAcceleration = kWrist.MAX_ACCELERATION;
        wristMotorCfg.MotionMagic.MotionMagicJerk = kWrist.MAX_JERK;

        wristMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wristMotorCfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;

        wristMotorCfg.MotorOutput.Inverted = kWrist.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        wristMotorCfg.MotorOutput.PeakForwardDutyCycle = 0.7;
        wristMotorCfg.MotorOutput.PeakReverseDutyCycle = -0.7;

        return wristMotorCfg;
    }

    /**
     * Constructs a TalonFXConfiguration object only from values
     * from {@link frc.robot.Constants.kIntake}
     * 
     * @return the TalonFXConfiguration object
     */
    private TalonFXConfiguration getIntakeMotorConfig() {
        TalonFXConfiguration intakeMotorCfg = new TalonFXConfiguration();

        intakeMotorCfg.MotorOutput.Inverted = kIntake.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        return intakeMotorCfg;
    }

    @Override
    public Boolean setMechanismDegrees(Double degrees) {
        if (degrees > kWrist.MAX_DEGREES) {
            new SetpointTooHigh(kWrist.MIN_DEGREES, degrees).log();
            return false;
        } else if (degrees < kWrist.MIN_DEGREES) {
            new SetpointTooLow(kWrist.MIN_DEGREES, degrees).log();
            return false;
        }
        isStowed = false;
        var posControlRequest = new MotionMagicDutyCycle(mechDegreesToMotorRots(degrees));
        this.wristMotor.setControl(posControlRequest);
        return Math.abs(degrees - getMechanismDegrees()) < kWrist.TOLERANCE;
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        isStowed = false;
        var percentControlRequest = new DutyCycleOut(percentOut);
        this.wristMotor.setControl(percentControlRequest);
    }

    @Override
    public void stopMechanism() {
        this.wristMotor.setVoltage(0.0);
    }

    @Override
    public Double getMechanismDegrees() {
        return cachedWristDegrees;
    }

    @Override
    public void runIntake(Double percentOut) {
        var percentControlRequest = new DutyCycleOut(percentOut);
        this.intakeMotor.setControl(percentControlRequest);
    }

    @Override
    public Double getIntakeVoltage() {
        return cachedIntakeVolts;
    }

    @Override
    public void setIntakeCurrentLimits(Double limit) {
        var cfg = new CurrentLimitsConfigs();
        cfg.SupplyCurrentLimitEnable = true;
        cfg.SupplyCurrentLimit = limit;
        cfg.SupplyCurrentThreshold = limit;
        cfg.SupplyTimeThreshold = 0.2;
    }

    @Override
    public Boolean stowMechanism(Boolean toZero) {
        if (isStowed) {
            return true;
        }

        // run until current limit is hit, then start to unwind
        this.manualDriveMechanism(0.2);
        if (wristMotorAmps.getValue() > kWrist.CURRENT_PEAK_FOR_ZERO) {
            this.stopMechanism();
            if (toZero) {
                this.wristMotor.setRotorPosition(mechDegreesToMotorRots(kWrist.HOME_DEGREES + kWrist.HARD_OFFSET));
            }
            isStowed = true;
        }
        return isStowed;
    }

    @Override
    public void setupShuffleboard(ShuffleboardContainer tab) {
        tab.addDouble("Wrist Amps", () -> wristMotorAmps.refresh().getValue());
        tab.addDouble("Wrist Volts", () -> wristMotorVolts.refresh().getValue());
        tab.addDouble("Wrist Rots", () -> wristMotorRots.refresh().getValue());
        tab.addDouble("Wrist Velo", () -> wristMotorVelo.refresh().getValue());
        tab.addBoolean("Wrist Homed", () -> isStowed);

        tab.addDouble("Intake Amps", () -> intakeMotorAmps.refresh().getValue());
        tab.addDouble("Intake Volts", () -> intakeMotorVolts.refresh().getValue());
    }

    @Override
    public void periodic() {
        this.cachedWristDegrees = motorRotsToMechDegrees(
                BaseStatusSignal.getLatencyCompensatedValue(wristMotorRots.refresh(), wristMotorVelo.refresh()));
        this.cachedIntakeVolts = intakeMotorVolts.refresh().getValue();
    }
}
