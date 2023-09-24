package frc.robot.subsystems.super_structure.elevator;

import frc.robot.subsystems.super_structure.Errors.SuperStructureErrors;
import frc.robot.util.ErrorHelper.GroupError;
import frc.robot.util.ErrorHelper.Ok;
import frc.robot.util.ErrorHelper.Result;
import frc.robot.Constants.kSuperStructure.kElevator;;

public interface Elevator {
    /**
     * Abstract from motors, will set the elevator meters.
     * @param meters from pivot
     * (min: {@link kElevator#ELEVATOR_MIN_METERS},
     * max: {@link kElevator#ELEVATOR_MAX_METERS})
     * @return if the operation was successful
     */
    public Result<Ok, GroupError<SuperStructureErrors>> setMechanismMeters(Double meters);

    public Double getMechanismMeters();

    /**
     * To be used for debugging, not guranteed to have all
     * safety features
     * 
     * @param percentOut of the mechanisms motors
     */
    public void manualDriveMechanism(Double percentOut);

    /**
     * @return if the reverse limit switch is activated
     */
    public Boolean isLimitSwitchHit();

    public void periodic();
}