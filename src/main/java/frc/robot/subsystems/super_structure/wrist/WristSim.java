package frc.robot.subsystems.super_structure.wrist;

import frc.robot.util.SimHelper.SetPoint;
import frc.robot.Constants.kSuperStructure.*;
import frc.robot.subsystems.super_structure.Errors.*;

public class WristSim implements Wrist {

    private final SetPoint wristDegrees = new SetPoint(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    private final SetPoint intakeVelocity = new SetPoint(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    @Override
    public Boolean setMechanismDegrees(Double degrees) {
        if (degrees > kWrist.MAX_DEGREES) {
            new SetpointTooHigh(kWrist.MAX_DEGREES, degrees).log();
            return false;
        } else if (degrees < kWrist.MIN_DEGREES) {
            new SetpointTooLow(kWrist.MIN_DEGREES, degrees).log();
            return false;
        }
        wristDegrees.setTargetPosition(degrees);
        return false;
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        wristDegrees.setTargetVelocity(360.0 * percentOut);
    }

    @Override
    public void stopMechanism() {
        wristDegrees.setTargetPosition(wristDegrees.getPose());
    }

    @Override
    public Double getMechanismDegrees() {
        return wristDegrees.getPose();
    }

    @Override
    public void runIntake(Double percentOut) {
        this.intakeVelocity.setTargetVelocity(6480.0 * percentOut);
    }

    @Override
    public Double getIntakeVoltage() {
        return 0.0;
    }

    @Override
    public void periodic() {}

    @Override
    public Boolean homeMechanism() {
        wristDegrees.setTargetPosition(kWrist.HOME_DEGREES);
        return true;
    }

    @Override
    public void enableIntakeCurrentLimits(Boolean enable) {}
}
