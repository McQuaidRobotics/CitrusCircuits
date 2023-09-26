package frc.robot.subsystems.super_structure.elevator;

public class ElevatorSim implements Elevator{

    @Override
    public Boolean setMechanismMeters(Double percent) {
        return null;
    }

    @Override
    public Double getMechanismMeters() {
        return 0.0;
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        
    }

    @Override
    public void stopMechanism() {
        
    }

    @Override
    public Boolean isLimitSwitchHit() {
        return false;
    }

    @Override
    public Boolean homeMechanism() {
        return true;
    }

    @Override
    public void periodic() {
        
    }
    
}