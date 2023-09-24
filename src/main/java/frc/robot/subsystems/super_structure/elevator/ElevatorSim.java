package frc.robot.subsystems.super_structure.elevator;

import frc.robot.subsystems.super_structure.Errors.SuperStructureErrors;
import frc.robot.util.ErrorHelper.GroupError;
import frc.robot.util.ErrorHelper.Ok;
import frc.robot.util.ErrorHelper.Result;

public class ElevatorSim implements Elevator{

    @Override
    public Result<Ok, GroupError<SuperStructureErrors>> setMechanismMeters(Double percent) {
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
    public Boolean isLimitSwitchHit() {
        return false;
    }

    @Override
    public void periodic() {
        
    }
    
}