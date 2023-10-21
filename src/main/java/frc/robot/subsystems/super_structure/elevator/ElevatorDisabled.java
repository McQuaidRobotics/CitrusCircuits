package frc.robot.subsystems.super_structure.elevator;

import frc.robot.Constants.kSuperStructure.kElevator;

public class ElevatorDisabled implements Elevator {

    public ElevatorDisabled(Double startingMeters) {
    }

    @Override
    public Boolean setElevatorMeters(Double meters) {
        return Math.abs(meters - kElevator.MIN_METERS) < kElevator.TOLERANCE;
    }

    @Override
    public Double getElevatorMeters() {
        return kElevator.MIN_METERS;
    }

    @Override
    public void manualDriveWrist(Double percentOut) {
    }

    @Override
    public void stopMechanism() {
    }

    @Override
    public Boolean isLimitSwitchHit() {
        return true;
    }

    @Override
    public Boolean homeMechanism(boolean force) {
        return true;
    }

    @Override
    public Double getRecentCurrent() {
        return 0.0;
    }
}