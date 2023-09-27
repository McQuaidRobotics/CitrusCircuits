package frc.robot.subsystems.super_structure.wrist;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.Constants.kSuperStructure.*;
import frc.robot.subsystems.super_structure.Errors.*;

public class WristReal implements Wrist {

    private final TalonFX wristMotor, intakeMotor;

    private final StatusSignal<Double> wristMotorRots, wristMotorVelo, wristMotorAmps, wristMotorVolts;
    private final StatusSignal<Double> intakeMotorAmps, intakeMotorVolts;

    private Boolean isHomed = false;
    private Boolean softLimitsEnabled = true;

    public WristReal() {
        wristMotor = new TalonFX(kWrist.MOTOR_ID);
        wristMotor.getConfigurator().apply(getWristMotorConfig());

        wristMotorRots = wristMotor.getRotorPosition();
        wristMotorVelo = wristMotor.getRotorVelocity();
        wristMotorAmps = wristMotor.getStatorCurrent();
        wristMotorVolts = wristMotor.getSupplyVoltage();

        intakeMotor = new TalonFX(kIntake.MOTOR_ID);
        intakeMotor.getConfigurator().apply(getIntakeMotorConfig());

        intakeMotorAmps = intakeMotor.getStatorCurrent();
        intakeMotorVolts = intakeMotor.getSupplyVoltage();
    }

    private Double mechDegreesToMotorRots(Double mechanismDegrees) {
        return (mechanismDegrees / 360.0) / kWrist.MOTOR_TO_MECHANISM_RATIO;
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

        wristMotorCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = kWrist.ENABLE_SOFTLIMITS;
        wristMotorCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = kWrist.ENABLE_SOFTLIMITS;
        wristMotorCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = mechDegreesToMotorRots(
                kWrist.MAX_DEGREES);
        wristMotorCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = mechDegreesToMotorRots(
                kWrist.MIN_DEGREES);

        wristMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        wristMotorCfg.MotorOutput.Inverted = kWrist.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

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

        intakeMotorCfg.CurrentLimits.SupplyCurrentLimitEnable = kIntake.CURRENT_LIMIT != 0;
        intakeMotorCfg.CurrentLimits.SupplyCurrentThreshold = kIntake.CURRENT_LIMIT;
        intakeMotorCfg.CurrentLimits.SupplyTimeThreshold = 0.5;

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
        isHomed = false;
        if (!this.softLimitsEnabled) {
            this.massSoftLimits(true, wristMotor);
            this.softLimitsEnabled = true;
        }
        var posControlRequest = new MotionMagicTorqueCurrentFOC(mechDegreesToMotorRots(degrees));
        this.wristMotor.setControl(posControlRequest);
        return Math.abs(degrees - getMechanismDegrees()) < kWrist.TOLERANCE;
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        isHomed = false;
        var percentControlRequest = new DutyCycleOut(percentOut, true, false);
        this.wristMotor.setControl(percentControlRequest);
    }

    @Override
    public void stopMechanism() {
        this.wristMotor.setVoltage(0.0);
    }

    @Override
    public Double getMechanismDegrees() {
        return BaseStatusSignal.getLatencyCompensatedValue(wristMotorRots.refresh(), wristMotorVelo.refresh())
                * 360.0
                * kWrist.MOTOR_TO_MECHANISM_RATIO;
    }

    @Override
    public void runIntake(Double percentOut) {
        var percentControlRequest = new DutyCycleOut(percentOut, true, false);
        this.intakeMotor.setControl(percentControlRequest);
    }

    @Override
    public Double getIntakeVoltage() {
        return intakeMotor.getSupplyVoltage().getValue();
    }

    @Override
    public void enableIntakeCurrentLimits(Boolean enable) {
        var cfg = new CurrentLimitsConfigs();
        this.intakeMotor.getConfigurator().refresh(cfg);
        cfg.SupplyCurrentLimitEnable = enable;
        this.intakeMotor.getConfigurator().apply(cfg);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void setupShuffleboard(ShuffleboardContainer tab) {
        tab.addDouble("Wrist Amps", () -> wristMotorAmps.refresh().getValue());
        tab.addDouble("Wrist Volts", () -> wristMotorVolts.refresh().getValue());
        tab.addDouble("Wrist Rots", () -> wristMotorRots.refresh().getValue());
        tab.addDouble("Wrist Velo", () -> wristMotorVelo.refresh().getValue());
        tab.addBoolean("Wrist Homed", () -> isHomed);

        tab.addDouble("Intake Amps", () -> intakeMotorAmps.refresh().getValue());
        tab.addDouble("Intake Volts", () -> intakeMotorVolts.refresh().getValue());
    }

    @Override
    public Boolean homeMechanism() {
        if (isHomed) {
            return true;
        }
        if (this.softLimitsEnabled) {
            this.massSoftLimits(false, wristMotor);
            this.softLimitsEnabled = false;
        }
        this.manualDriveMechanism(0.2);
        if (wristMotorAmps.getValue() > kWrist.CURRENT_PEAK_FOR_ZERO) {
            this.stopMechanism();
            this.wristMotor.setRotorPosition(mechDegreesToMotorRots(kWrist.HOME_DEGREES));
            isHomed = true;
        }
        return isHomed;
    }
}
