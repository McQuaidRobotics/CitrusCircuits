package frc.robot.subsystems.super_structure;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public interface Component {

    public void periodic();

    default public void setupShuffleboard(ShuffleboardContainer tab) {};

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
     * Moves the mechanis towards the home postion resetting the encoders
     * @return true if the mcahnism has reached home
     */
    public Boolean homeMechanism();
}
