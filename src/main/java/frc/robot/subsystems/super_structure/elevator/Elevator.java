package frc.robot.subsystems.super_structure.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public interface Elevator {
    public void setMechanismMeters(Double percent);

    public Double getMechanismMeters();

    /**
     * To be used for debugging, not guranteed to have all
     * safety features
     * 
     * @param percentOut of the mechanisms motors
     */
    public void manualDriveMechanism(Double percentOut);

    public void periodic();
}