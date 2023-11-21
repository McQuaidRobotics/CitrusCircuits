package frc.robot.subsystems.super_structure.endEffector;

public class EndEffectorSim implements EndEffector {
    private Double eeVolts = 0.0;

    public EndEffectorSim() {
    }

    @Override
    public Double getEndEffectorVoltage() {
        return eeVolts;
    }

    @Override
    public void runEndEffector(Double volts) {
        eeVolts = volts;
    }

    @Override
    public void setEndEffectorCurrentLimits(Double amps) {
    }
}
