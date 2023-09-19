package frc.robot.subsystems.super_structure.wrist;

import frc.robot.util.SimHelper.SetPoint;

public class WristSim implements Wrist {

    private SetPoint degrees = new SetPoint(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    private SetPoint velocity = new SetPoint(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    @Override
    public void stopMechanism() {
        //TODO
    }










    @Override
    public void stopIntake() {
        this.velocity
    }
}
