package frc.robot.subsystems.super_structure.pivot;

import frc.robot.util.SimHelper.SetPoint;

import frc.robot.Constants.kSuperStructure.*;
import frc.robot.subsystems.super_structure.Errors.*;

public class PivotSim implements Pivot {

    private final SetPoint pivotDegrees = new SetPoint(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    @Override
    public Boolean setMechanismDegrees(Double degrees) {
        if (degrees > kPivot.MAX_DEGREES) {
            new SetpointTooHigh(kPivot.MAX_DEGREES, degrees).log();
            return false;
        } else if (degrees < kPivot.MIN_DEGREES) {
            new SetpointTooLow(kPivot.MIN_DEGREES, degrees).log();
            return false;
        }
        pivotDegrees.setTargetPosition(degrees);
        return false;
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        pivotDegrees.setTargetVelocity(360.0 * percentOut);
    }

    @Override
    public void stopMechanism() {
        pivotDegrees.setTargetPosition(pivotDegrees.getPose());
    }

    @Override
    public Double getMechanismDegrees() {
        return pivotDegrees.getPose();
    }

    @Override
    public Boolean homeMechanism() {
        pivotDegrees.setTargetPosition(kPivot.HOME_DEGREES);
        return true;
    }

    @Override
    public void periodic() {}

}
