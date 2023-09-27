package frc.robot.subsystems.super_structure;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public interface Component {

    default public void periodic() {};

    default public void setupShuffleboard(ShuffleboardContainer tab) {};

    default void massSoftLimits(Boolean toggle, TalonFX... motors) {
        for (var motor : motors) {
            var cfg = new SoftwareLimitSwitchConfigs();
            motor.getConfigurator().refresh(cfg);
            cfg.ReverseSoftLimitEnable = toggle;
            cfg.ForwardSoftLimitEnable = toggle;
            motor.getConfigurator().apply(cfg);
        }
    }

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
