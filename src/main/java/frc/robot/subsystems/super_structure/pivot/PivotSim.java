package frc.robot.subsystems.super_structure.pivot;

import frc.robot.util.SimHelper.SetPoint;

import frc.robot.Constants.kSuperStructure.*;
import frc.robot.subsystems.super_structure.Errors.*;
import frc.robot.util.ErrorHelper.*;

public class PivotSim implements Pivot {

    private final SetPoint pivotDegrees = new SetPoint(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    @Override
    public Result<Ok, GroupError<SuperStructureErrors>> setMechanismDegrees(Double degrees) {
        if (degrees > kPivot.MAX_DEGREES) {
            return Result.err(new SetpointTooHigh(kPivot.MAX_DEGREES, degrees));
        } else if (degrees < kPivot.MIN_DEGREES) {
            return Result.err(new SetpointTooLow(kPivot.MIN_DEGREES, degrees));
        }
        pivotDegrees.setTargetPosition(degrees);
        return Result.ok(new Ok());
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
    public void zeroMechanism() {
        pivotDegrees.setTargetPosition(0.0);
    }

    @Override
    public void playErrorTone() {}

    @Override
    public void periodic() {}

}
