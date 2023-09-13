package frc.robot.subsystems.super_structure;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pivot {
    public Pivot() {}

    private void configureMotors() {}

    /**
     * Abstract from motors, will set the pivots degrees
     * parallel to floor is 0 degrees
     */
    public void setMechanismDegrees(Double setpoint) {}

    public Double getMechanismDegrees() {return null;}


    public void periodic() {}
}
