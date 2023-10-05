package frc.robot.subsystems.super_structure.wrist;

import frc.robot.util.SimHelper.SimplePoseSim;
import frc.robot.Constants.kSuperStructure.*;
import frc.robot.subsystems.super_structure.Errors.*;

public class WristSim implements Wrist {

    private final Double maxVelo = 112.0 * 360.0 * kWrist.MOTOR_TO_MECHANISM_RATIO;
    private final SimplePoseSim wristDegrees = new SimplePoseSim(maxVelo);
    private Double intakeVolts = 0.0;

    public WristSim(Double startingDegrees) {
        wristDegrees.instantSetPose(startingDegrees);
    }

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
        return Math.abs(wristDegrees.getPose() - degrees) < 0.1;
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        wristDegrees.setTargetVelocity(maxVelo * percentOut);
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
        this.intakeVolts = percentOut * 12.0;
    }

    @Override
    public Double getIntakeVoltage() {
        return intakeVolts;
    }

    @Override
    public Boolean stowMechanism(Boolean toZero) {
        wristDegrees.setTargetPosition(kWrist.HOME_DEGREES);
        return Math.abs(wristDegrees.getPose() - kWrist.HOME_DEGREES) < 0.1;
    }

    @Override
    public void setIntakeCurrentLimits(Double limit) {
    }
}
