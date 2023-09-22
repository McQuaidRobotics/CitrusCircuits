package frc.robot.subsystems.super_structure.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.kSuperStructure.*;

public class ElevatorReal implements Elevator{
    private TalonFX extendMotorLeader, extendMotorFollower;

    public ElevatorReal() {
        extendMotorLeader = new TalonFX(kElevator.ELEVATOR_LEFT_MOTOR_ID);
        extendMotorLeader.getConfigurator().apply(getMotorConfiguration());

        extendMotorFollower = new TalonFX(kElevator.ELEVATOR_RIGHT_MOTOR_ID);
        extendMotorFollower.getConfigurator().apply(getMotorConfiguration());
        extendMotorFollower.setControl(new Follower(kElevator.ELEVATOR_RIGHT_MOTOR_ID, true));
    }

    private TalonFXConfiguration getMotorConfiguration() {
        var motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.kP = kElevator.MOTOR_kP;
        motorConfig.Slot0.kP = kElevator.MOTOR_kI;
        motorConfig.Slot0.kP = kElevator.MOTOR_kD;

        return motorConfig;
    }

    @Override
    public void setMechanismMeters(Double percent) {

    }

    @Override
    public Double getMechanismMeters() {
        return Specs.ELEVATOR_MIN_METERS;
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {

    }

    @Override
    public void periodic() {

    }
}
