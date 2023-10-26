package frc.robot.subsystems.super_structure;

import frc.robot.util.ShuffleboardApi.ShuffleLayout;

public interface Component {

    default public void periodic() {
    };

    default public void setupShuffleboard(ShuffleLayout tab) {
    };

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
     * Moves the mechanis towards the stow position,
     * if {@code toZero} is true the motor encoders will also be reseeded
     * 
     * @return true if the mcahnism has reached home
     */
    public Boolean homeMechanism(boolean force);

    /** Returns the average current over the past .5 seconds */
    public Double getRecentCurrent();

    public default void brake(Boolean toBrake) {
    }
}
