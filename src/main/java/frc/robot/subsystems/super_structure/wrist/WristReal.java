package frc.robot.subsystems.super_structure.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants.kSuperStructure.kWrist;
import frc.robot.Constants.kSuperStructure.kIntake;
import frc.robot.subsystems.super_structure.Errors.*;
import frc.robot.util.ShuffleboardApi.ShuffleLayout;

public class WristReal implements Wrist {

    private final TalonFX wristMotor, intakeMotor;

    private final StatusSignal<Double> wristMotorRots, wristMotorVelo, wristMotorAmps, wristMotorVolts;
    private final StatusSignal<Double> intakeMotorAmps, intakeMotorVolts;

    private final LinearFilter ampWindow = LinearFilter.movingAverage(25);
    private Double ampWindowVal = 0.0;

    private Boolean isStowed = false;
    private Double cachedWristDegrees, cachedIntakeVolts = 0.0, intakeCurrentLimit;

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
        wristMotorCfg.MotorOutput.Inverted = kWrist.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        wristMotorCfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;


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
    public Boolean setWristDegrees(Double degrees) {
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
        return Math.abs(degrees - getWristDegrees()) < kWrist.TOLERANCE;
    }

    @Override
    public void manualDriveWrist(Double percentOut) {
        isStowed = false;
        var percentControlRequest = new DutyCycleOut(percentOut);
        this.wristMotor.setControl(percentControlRequest);
    }

    @Override
    public void stopMechanism() {
        this.wristMotor.setVoltage(0.0);
    }

    @Override
    public Double getWristDegrees() {
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
        if (limit != this.intakeCurrentLimit) {
            var cfg = new CurrentLimitsConfigs();
            cfg.SupplyCurrentLimitEnable = true;
            cfg.SupplyCurrentLimit = limit;
            cfg.SupplyCurrentThreshold = limit;
            cfg.SupplyTimeThreshold = 0.2;
            cfg.StatorCurrentLimit = limit;
            cfg.StatorCurrentLimitEnable = true;
            intakeMotor.getConfigurator().apply(cfg);
            this.intakeCurrentLimit = limit;
        }
    }

    @Override
    public Boolean homeMechanism(boolean force) {
        if (force) {
            isStowed = false;
        }
        if (isStowed) {
            return true;
        }

        if (getWristDegrees() < kWrist.HOME_DEGREES - 10.0) {
            setWristDegrees(kWrist.HOME_DEGREES);
        } else {
            manualDriveWrist(0.2);
        }
        if (wristMotorAmps.getValue() > kWrist.CURRENT_PEAK_FOR_ZERO) {
            this.stopMechanism();
            this.wristMotor.setRotorPosition(mechDegreesToMotorRots(kWrist.HOME_DEGREES + kWrist.HARD_OFFSET));
            isStowed = true;
        }

        return isStowed;
    }

    @Override
    public Double getRecentCurrent() {
        return ampWindowVal;
    }

    @Override
    public void brake(Boolean toBrake) {
        var motorOutputCfg = new MotorOutputConfigs();
        motorOutputCfg.NeutralMode = toBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motorOutputCfg.Inverted = kWrist.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        wristMotor.getConfigurator().apply(motorOutputCfg);
        if (toBrake) {
            wristMotor.setControl(new StaticBrake());
        } else {
            wristMotor.setControl(new CoastOut());
        }
    }

    @Override
    public void setupShuffleboard(ShuffleLayout tab) {
        tab.addDouble("Wrist Amps", () -> wristMotorAmps.getValue());// refreshed in periodic
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

        ampWindowVal = ampWindow.calculate(wristMotorAmps.refresh().getValue());
    }
}
