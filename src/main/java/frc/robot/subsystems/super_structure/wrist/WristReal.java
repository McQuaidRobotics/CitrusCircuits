package frc.robot.subsystems.super_structure.wrist;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.kSuperStructure.*;
import frc.robot.subsystems.super_structure.Errors.*;
import frc.robot.util.ErrorHelper.*;

public class WristReal implements Wrist {

    private final TalonFX wristMotor, intakeMotor;

    private final StatusSignal<Double> wristMotorRots, wristMotorVelo, wristMotorAmps;

    public WristReal() {
        wristMotor = new TalonFX(kWrist.MOTOR_ID);
        wristMotor.getConfigurator().apply(getWristMotorConfig());

        wristMotorRots = wristMotor.getRotorPosition();
        wristMotorVelo = wristMotor.getRotorVelocity();
        wristMotorAmps = wristMotor.getStatorCurrent();

        intakeMotor = new TalonFX(kIntake.MOTOR_ID);
        intakeMotor.getConfigurator().apply(getIntakeMotorConfig());
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
        wristMotorCfg.Slot0.kS = kWrist.MOTOR_kS;
        wristMotorCfg.Slot0.kV = kWrist.MOTOR_kV;

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

        intakeMotorCfg.CurrentLimits.StatorCurrentLimitEnable = kIntake.CURRENT_LIMIT != 0;
        intakeMotorCfg.CurrentLimits.StatorCurrentLimit = kIntake.CURRENT_LIMIT;

        return intakeMotorCfg;
    }

    @Override
    public Result<Ok, GroupError<SuperStructureErrors>> setMechanismDegrees(Double degrees) {
        if (degrees > kWrist.MAX_DEGREES) {
            return Result.err(new SetpointTooHigh(kWrist.MAX_DEGREES, degrees));
        } else if (degrees < kWrist.MIN_DEGREES) {
            return Result.err(new SetpointTooLow(kWrist.MIN_DEGREES, degrees));
        }
        var posControlRequest = new MotionMagicTorqueCurrentFOC(mechDegreesToMotorRots(degrees));
        this.wristMotor.setControl(posControlRequest);
        return Result.ok(new Ok());
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        var percentControlRequest = new DutyCycleOut(percentOut, true, false);
        this.wristMotor.setControl(percentControlRequest);
    }

    @Override
    public void stopMechanism() {
        this.wristMotor.setVoltage(0.0);
    }

    @Override
    public Double getMechanismDegrees() {
        BaseStatusSignal.waitForAll(0, wristMotorRots, wristMotorVelo);
        return BaseStatusSignal.getLatencyCompensatedValue(wristMotorRots, wristMotorVelo)
                * 360.0
                * kWrist.MOTOR_TO_MECHANISM_RATIO;
    }

    // @Override
    // public void zeroMechanism() {
    //     // disable soft limits to not interfere with zeroing
    //     var softLimitCfg = new SoftwareLimitSwitchConfigs();
    //     wristMotor.getConfigurator().refresh(softLimitCfg);
    //     softLimitCfg.ForwardSoftLimitEnable = false;
    //     softLimitCfg.ReverseSoftLimitEnable = false;
    //     wristMotor.getConfigurator().apply(softLimitCfg);

    //     var currentSignal = wristMotor.getTorqueCurrent();
    //     currentSignal.setUpdateFrequency(250);

    //     var timer = new Timer();

    //     // start timer an motor, monitor current. this could also use moving average if
    //     // needed
    //     timer.start();
    //     wristMotor.set(0.1);
    //     while (currentSignal.getValue() < kWrist.CURRENT_PEAK_FOR_ZERO) {
    //         if (timer.hasElapsed(1.0)) {
    //             // took too long, stop motor and report error. NO ZERO
    //             wristMotor.set(0.0);
    //             DriverStation.reportError("Wrist Motor Zero Timeout", false);
    //             return;
    //         }
    //     }

    //     // stop motor, reset encoder
    //     wristMotor.set(mechDegreesToMotorRots(kWrist.MAX_DEGREES));
    //     wristMotor.setRotorPosition(0);

    //     // set soft limits
    //     softLimitCfg.ForwardSoftLimitEnable = true;
    //     softLimitCfg.ReverseSoftLimitEnable = true;
    //     wristMotor.getConfigurator().apply(softLimitCfg);
    // }

    @Override
    public Double getMechanismCurrent() {
        return wristMotorAmps.getValue();
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
    public void stopIntake() {
        this.intakeMotor.setVoltage(0.0);
    }

    @Override
    public void playErrorTone() {}

    @Override
    public void periodic() {
    }

    @Override
    public void setupShuffleboard(ShuffleboardTab tab) {
        tab.addDouble("Wrist Amps", () -> wristMotorAmps.getValue())
            .withSize(2, 1);
        tab.addDouble("Wrist Volts", () -> wristMotor.getSupplyVoltage().getValue())
            .withSize(2, 1);
    }
}
