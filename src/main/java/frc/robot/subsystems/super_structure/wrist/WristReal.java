package frc.robot.subsystems.super_structure.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.SuperStructure;;

public class WristReal implements Wrist {

    private final TalonFX wristMotor, intakeMotor;

    public WristReal() {
        wristMotor = new TalonFX(SuperStructure.Wrist.MOTOR_ID);
        wristMotor.getConfigurator().apply(getWristMotorConfig());

        intakeMotor = new TalonFX(SuperStructure.Intake.MOTOR_ID);
        intakeMotor.getConfigurator().apply(getIntakeMotorConfig());

        setupShuffleboard();
    }

    private Double mechDegreesToMotorRots(Double mechanismDegrees) {
        return (mechanismDegrees / 360.0) / SuperStructure.Wrist.MOTOR_TO_MECHANISM_RATIO;
    }

    /**
     * Constructs a TalonFXConfiguration object only from values
     * from {@link frc.robot.Constants.SuperStructure.Wrist}
     * 
     * @return the TalonFXConfiguration object
     */
    private TalonFXConfiguration getWristMotorConfig() {
        TalonFXConfiguration wristMotorCfg = new TalonFXConfiguration();
        wristMotorCfg.Slot0.kP = SuperStructure.Wrist.MOTOR_kP;
        wristMotorCfg.Slot0.kI = SuperStructure.Wrist.MOTOR_kI;
        wristMotorCfg.Slot0.kD = SuperStructure.Wrist.MOTOR_kD;
        wristMotorCfg.Slot0.kS = SuperStructure.Wrist.MOTOR_kS;
        wristMotorCfg.Slot0.kV = SuperStructure.Wrist.MOTOR_kV;

        wristMotorCfg.MotionMagic.MotionMagicCruiseVelocity = SuperStructure.Wrist.MAX_VELOCITY;
        wristMotorCfg.MotionMagic.MotionMagicAcceleration = SuperStructure.Wrist.MAX_ACCELERATION;
        wristMotorCfg.MotionMagic.MotionMagicJerk = SuperStructure.Wrist.MAX_JERK;

        wristMotorCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = SuperStructure.Wrist.ENABLE_SOFTLIMITS;
        wristMotorCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = SuperStructure.Wrist.ENABLE_SOFTLIMITS;
        wristMotorCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = mechDegreesToMotorRots(
                SuperStructure.Wrist.MAX_DEGREES);
        wristMotorCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = mechDegreesToMotorRots(
                -SuperStructure.Wrist.MIN_DEGREES);

        wristMotorCfg.MotorOutput.Inverted = SuperStructure.Wrist.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        return wristMotorCfg;
    }

    /**
     * Constructs a TalonFXConfiguration object only from values
     * from {@link frc.robot.Constants.SuperStructure.Intake}
     * 
     * @return the TalonFXConfiguration object
     */
    private TalonFXConfiguration getIntakeMotorConfig() {
        TalonFXConfiguration intakeMotorCfg = new TalonFXConfiguration();

        intakeMotorCfg.MotorOutput.Inverted = SuperStructure.Intake.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        intakeMotorCfg.CurrentLimits.StatorCurrentLimitEnable = SuperStructure.Intake.CURRENT_LIMIT != 0;
        intakeMotorCfg.CurrentLimits.StatorCurrentLimit = SuperStructure.Intake.CURRENT_LIMIT;

        return intakeMotorCfg;
    }

    @Override
    public void setMechanismDegrees(Double degrees) {
        if (degrees > SuperStructure.Wrist.MAX_DEGREES) {
            DriverStation.reportError("Wrist setpoint too high", false);
            return;
        } else if (degrees < -SuperStructure.Wrist.MIN_DEGREES) {
            DriverStation.reportError("Wrist setpoint too low", false);
            return;
        }
        var posControlRequest = new PositionTorqueCurrentFOC(mechDegreesToMotorRots(degrees));
        this.wristMotor.setControl(posControlRequest);
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
        var motorRots = wristMotor.getRotorPosition();
        var motorVelo = wristMotor.getRotorVelocity();
        return BaseStatusSignal.getLatencyCompensatedValue(motorRots, motorVelo)
                * 360.0
                * SuperStructure.Wrist.MOTOR_TO_MECHANISM_RATIO;
    }

    @Override
    public void zeroMechanism() {
        // disable soft limits to not interfere with zeroing
        var softLimitCfg = new SoftwareLimitSwitchConfigs();
        wristMotor.getConfigurator().refresh(softLimitCfg);
        softLimitCfg.ForwardSoftLimitEnable = false;
        softLimitCfg.ReverseSoftLimitEnable = false;
        wristMotor.getConfigurator().apply(softLimitCfg);

        var currentSignal = wristMotor.getTorqueCurrent();
        currentSignal.setUpdateFrequency(250);

        var timer = new Timer();

        // start timer an motor, monitor current. this could also use moving average if
        // needed
        timer.start();
        wristMotor.set(0.1);
        while (currentSignal.getValue() < SuperStructure.Wrist.CURRENT_PEAK_FOR_ZERO) {
            if (timer.hasElapsed(1.0)) {
                // took too long, stop motor and report error. NO ZERO
                wristMotor.set(0.0);
                DriverStation.reportError("Wrist Motor Zero Timeout", false);
                return;
            }
        }

        // stop motor, reset encoder
        wristMotor.set(mechDegreesToMotorRots(SuperStructure.Wrist.MAX_DEGREES));
        wristMotor.setRotorPosition(0);

        // set soft limits
        softLimitCfg.ForwardSoftLimitEnable = true;
        softLimitCfg.ReverseSoftLimitEnable = true;
        wristMotor.getConfigurator().apply(softLimitCfg);
    }


    @Override
    public void runIntake(Double percentOut) {
        var percentControlRequest = new DutyCycleOut(percentOut, true, false);
        this.intakeMotor.setControl(percentControlRequest);
    }

    @Override
    public void stopIntake() {
        this.intakeMotor.setVoltage(0.0);
    }


    /** once moved over to TEMPLATE this can be removed */
    private void setupShuffleboard() {
        var tab = Shuffleboard.getTab("Wrist");
        tab.addDouble("Wrist Degrees", this::getMechanismDegrees)
            .withPosition(0, 0)
            .withSize(3, 3)
            .withWidget(BuiltInWidgets.kGraph);
    }

    @Override
    public void periodic() {}
}
