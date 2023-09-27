package frc.robot.subsystems.super_structure.pivot;

import frc.robot.util.SimHelper.SimplePoseSim;

import frc.robot.Constants.kSuperStructure.*;
import frc.robot.subsystems.super_structure.Errors.*;

public class PivotSim implements Pivot {

    private final Double maxVelo = 112.0 * 360.0 * kPivot.MOTOR_TO_MECHANISM_RATIO;
    private final SimplePoseSim pivotDegrees = new SimplePoseSim(maxVelo);

    public PivotSim(Double startingDegrees) {
        pivotDegrees.instantSetPose(startingDegrees);
    }

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
        return Math.abs(pivotDegrees.getPose() - degrees) < 0.1;
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        pivotDegrees.setTargetVelocity(percentOut * maxVelo);
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
        return Math.abs(pivotDegrees.getPose() - kPivot.HOME_DEGREES) < 0.1;
    }

}
