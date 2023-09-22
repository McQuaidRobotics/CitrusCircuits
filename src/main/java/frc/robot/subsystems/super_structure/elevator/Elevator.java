package frc.robot.subsystems.super_structure.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.subsystems.super_structure.Errors.SuperStructureErrors;
import frc.robot.util.ErrorHelper.GroupError;
import frc.robot.util.ErrorHelper.Ok;
import frc.robot.util.ErrorHelper.Result;

public interface Elevator {
    public Result<Ok, GroupError<SuperStructureErrors>> setMechanismMeters(Double meters);

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