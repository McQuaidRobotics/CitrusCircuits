package frc.robot.subsystems.super_structure.endEffector;

import frc.robot.subsystems.super_structure.Component;

public interface EndEffector extends Component {

    /**
     * Runs the end-effector at a given percent output
     * 
     * @param volts of the end-effector motor
     */
    public void runEndEffector(Double volts);

    /**
     * @return the voltage of the end-effector motor
     */
    public Double getEndEffectorVoltage();

    /**
     * @param amps true to enable current limits, false to disable
     */
    public void setEndEffectorCurrentLimits(Double amps);

    @Override
    default public void manualDriveMechanism(Double percentOut) {
        this.runEndEffector(percentOut * 12.0);
    }

    @Override
    default public Boolean homeMechanism(boolean force) {
        return true;
    }

    @Override
    default void stopMechanism() {
        this.runEndEffector(0.0);
    }
}
