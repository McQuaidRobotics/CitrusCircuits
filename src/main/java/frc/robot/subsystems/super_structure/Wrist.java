package frc.robot.subsystems.super_structure;

public class Wrist {
    public Wrist() {}

    private void configureMotors() {}

    /**
     * Abstract from motors, will set the pivots degrees
     * parallel to the elevator as 0 degrees
     */
    public void setMechanismDegrees(Double degrees) {}

    public Double getMechanismDegrees() {return null;}

    public void enableRollers() {}

    public void disableRollers() {}

    public void stopRollers() {}

    /**
     * Reverses the rollers for a breif second or to so that
     * that any game piece inside gets spit out
     */
    public void emptyRollers() {}

    public void periodic() {}
}
