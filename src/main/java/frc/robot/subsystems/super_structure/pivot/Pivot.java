package frc.robot.subsystems.super_structure.pivot;

import frc.robot.subsystems.super_structure.Errors.*;
import frc.robot.util.ErrorHelper.*;

public interface Pivot {

    /**
     * Abstract from motors, will set the wrist degrees.
     * Parallel to the elevator is 0 degrees
     */
    public Result<Ok, GroupError<SuperStructureErrors>> setMechanismDegrees(Double degrees);

    /**
     * To be used for debugging, not guranteed to have all
     * safety features
     * 
     * @param percentOut of the mechanisms motors
     */
    public void manualDriveMechanism(Double percentOut);

    /**
     * Stops the mechanism
     */
    public void stopMechanism();

    /**
     * @return the current angle of the mechanism
     */
    public Double getMechanismDegrees();

    /**
     * Moves the mechanism up until the current detection is triggered,
     * detection position resolves to 0 -
     * {@link frc.robot.Constants.kSuperStructure.kPivot#MIN_DEGREES}
     */
    public void zeroMechanism();

    /**
     * Plays a chirp on the motors to signify an error
     */
    public void playErrorTone();

    public void periodic();
}
