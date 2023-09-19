package frc.robot.subsystems.super_structure.wrist;

public interface Wrist {
    /**
     * Abstract from motors, will set the pivots degrees.
     * Parallel to the elevator is 0 degrees
     */
    public void setMechanismDegrees(Double degrees);

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
     * {@link frc.robot.Constants.SuperStructure.Wrist#MAX_DEGREES}
     */
    public void zeroMechanism();


    /**
     * Runs the intake at a given percent output
     * 
     * @param percentOut of the intake motor
     */
    public void runIntake(Double percentOut);

    /**
     * Stops the intake
     */
    public void stopIntake();

    public void periodic();
}
