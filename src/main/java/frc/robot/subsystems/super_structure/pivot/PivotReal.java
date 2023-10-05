package frc.robot.subsystems.super_structure.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix.sensors.PigeonIMU;
// import com.ctre.phoenix.sensors.PigeonIMUConfiguration;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
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
    // private final PigeonIMU gyro;

    private final StatusSignal<Double> motorRots, motorVelo, motorAmps, motorVolts;

    private Boolean isStowed = false;
    private Double cachedPivotDegrees;

    private Double mechDegreesToMotorRots(Double mechanismDegrees) {
        return (mechanismDegrees / 360.0) / kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    private Double motorRotsToMechDegrees(Double motorRots) {
        return motorRots * 360.0 * kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    public PivotReal(Double startingDegrees) {
        leaderMotor = new TalonFX(kPivot.LEFT_MOTOR_ID);
        followerMotor = new TalonFX(kPivot.RIGHT_MOTOR_ID);
        leaderMotor.getConfigurator().apply(getMotorConfig());
        followerMotor.getConfigurator().apply(getMotorConfig());

        followerMotor.setControl(
                new Follower(kPivot.LEFT_MOTOR_ID, true));

        leaderMotor.setRotorPosition(mechDegreesToMotorRots(startingDegrees));
        cachedPivotDegrees = startingDegrees;

        motorRots = leaderMotor.getRotorPosition();
        motorVelo = leaderMotor.getRotorVelocity();
        motorAmps = leaderMotor.getStatorCurrent();
        motorVolts = leaderMotor.getSupplyVoltage();

        // gyro = new PigeonIMU(kPivot.PIGEON_ID);
    }

    private TalonFXConfiguration getMotorConfig() {
        TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.Slot0.kP = kPivot.MOTOR_kP;
        motorCfg.Slot0.kI = kPivot.MOTOR_kI;
        motorCfg.Slot0.kD = kPivot.MOTOR_kD;

        motorCfg.MotionMagic.MotionMagicCruiseVelocity = kPivot.MAX_VELOCITY;
        motorCfg.MotionMagic.MotionMagicAcceleration = kPivot.MAX_ACCELERATION;
        motorCfg.MotionMagic.MotionMagicJerk = kPivot.MAX_JERK;

        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorCfg.MotorOutput.Inverted = kPivot.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        motorCfg.Voltage.PeakForwardVoltage = kPivot.VOLTAGE_COMP;
        motorCfg.Voltage.PeakReverseVoltage = -kPivot.VOLTAGE_COMP;

        motorCfg.MotorOutput.PeakForwardDutyCycle = 0.8;
        motorCfg.MotorOutput.PeakReverseDutyCycle = -0.8;

        return motorCfg;
    }

    @Override
    public Boolean setPivotDegrees(Double degrees) {
        if (degrees > kPivot.MAX_DEGREES) {
            new SetpointTooHigh(kPivot.MAX_DEGREES, degrees).log();
            return false;
        } else if (degrees < kPivot.MIN_DEGREES) {
            new SetpointTooLow(kPivot.MIN_DEGREES, degrees).log();
            return false;
        }

        // if the last state was stow re seed motor by gyro
        // if (isStowed) {
        // this.leaderMotor.setRotorPosition(
        // mechDegreesToMotorRots(gyro.getYaw())
        // );

        // }

        isStowed = false;
        var posControlRequest = new MotionMagicDutyCycle(mechDegreesToMotorRots(degrees));
        this.leaderMotor.setControl(posControlRequest);
        return Math.abs(degrees - getPivotDegrees()) < kPivot.TOLERANCE;
    }
    @Override
    public void manualDriveMechanism(Double percentOut) {
        isStowed = false;
        var percentControlRequest = new DutyCycleOut(percentOut);
        this.leaderMotor.setControl(percentControlRequest);
    }

    @Override
    public void stopMechanism() {
        this.leaderMotor.setVoltage(0.0);
    }

    @Override
    public Double getPivotDegrees() {
        return cachedPivotDegrees;
    }

    @Override
    public Boolean stowMechanism(Boolean toZero) {
        if (isStowed) {
            return true;
        }
        this.manualDriveMechanism(-0.15);
        if (motorAmps.refresh().getValue() > kPivot.CURRENT_PEAK_FOR_ZERO) {
            this.stopMechanism();
            if (toZero) {
                this.leaderMotor.setRotorPosition(mechDegreesToMotorRots(kPivot.HOME_DEGREES));
            }
            isStowed = true;
        }
        return isStowed;
    }

    @Override
    public void setupShuffleboard(ShuffleboardContainer tab) {
        tab.addNumber("Pivot Motor Rots", () -> motorRots.refresh().getValue());
        tab.addNumber("Pivot Motor Velo", () -> motorVelo.refresh().getValue());
        tab.addNumber("Pivot Motor Amps", () -> motorAmps.refresh().getValue());
        tab.addNumber("Pivot Motor Volts", () -> motorVolts.refresh().getValue());
        // tab.addNumber("Pivot Gyro Yaw", gyro::getYaw);
        // tab.addNumber("Pivot Gyro Roll", gyro::getRoll);
        // tab.addNumber("Pivot Gyro Pitch", gyro::getPitch);
        tab.addBoolean("Pivot Homed", () -> isStowed);
    }

    @Override
    public void periodic() {
        cachedPivotDegrees = motorRotsToMechDegrees(
            BaseStatusSignal.getLatencyCompensatedValue(motorRots, motorVelo)
        );
    }
}
