package frc.robot.subsystems.super_structure.wrist;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.super_structure.Errors.SuperStructureErrors;
import frc.robot.util.ErrorHelper.GroupError;
import frc.robot.util.ErrorHelper.Ok;
import frc.robot.util.ErrorHelper.Result;

public interface Wrist {
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
     * @return the current draw of the mechanism
     */
    public Double getMechanismCurrent();

    /**
     * Runs the intake at a given percent output
     * 
     * @param volts of the intake motor
     */
    public void runIntake(Double volts);

    /**
     * @return the voltage of the intake motor
     */
    public Double getIntakeVoltage();

    /**
     * Stops the intake
     */
    public void stopIntake();

    /**
     * Plays a chirp on the motors to signify an error
     */
    public void playErrorTone();

    public void periodic();

    public void setupShuffleboard(ShuffleboardTab tab);
}
