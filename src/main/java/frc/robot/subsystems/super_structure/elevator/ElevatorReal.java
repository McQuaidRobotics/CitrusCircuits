package frc.robot.subsystems.super_structure.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class ElevatorReal {
    private TalonFX extendMotorLeader, extendMotorFollower;

    public ElevatorReal() {
        extendMotorLeader = new TalonFX(Constants.SuperStructure.Elevator.MOTOR_ID_LEADER);
        extendMotorLeader.getConfigurator().apply(getElevatorMotorConfig());

        extendMotorFollower = new TalonFX(Constants.SuperStructure.Elevator.MOTOR_ID_FOLLOWER);
        extendMotorFollower.getConfigurator().apply(getElevatorMotorConfig());
        extendMotorFollower.setControl(new Follower(Constants.SuperStructure.Elevator.MOTOR_ID_LEADER, true));
    }

    private TalonFXConfiguration getElevatorMotorConfig() {
        
    }
}
