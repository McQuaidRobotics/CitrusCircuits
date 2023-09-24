package frc.robot.subsystems.super_structure.wrist;

import frc.robot.util.SimHelper.SetPoint;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.kSuperStructure.*;
import frc.robot.subsystems.super_structure.Errors.*;
import frc.robot.util.ErrorHelper.*;

public class WristSim implements Wrist {

    private final SetPoint wristDegrees = new SetPoint(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    private final SetPoint intakeVelocity = new SetPoint(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    @Override
    public Result<Ok, GroupError<SuperStructureErrors>> setMechanismDegrees(Double degrees) {
        if (degrees > kWrist.MAX_DEGREES) {
            return Result.err(new SetpointTooHigh(kWrist.MAX_DEGREES, degrees));
        } else if (degrees < kWrist.MIN_DEGREES) {
            return Result.err(new SetpointTooLow(kWrist.MIN_DEGREES, degrees));
        }
        wristDegrees.setTargetPosition(degrees);
        return Result.ok(new Ok());
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
    public Double getMechanismCurrent() {
        return 0.0;
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
    public void stopIntake() {
        this.intakeVelocity.setTargetVelocity(0.0);
    }

    @Override
    public void playErrorTone() {}

    @Override
    public void periodic() {}

    @Override
    public void setupShuffleboard(ShuffleboardTab tab) {
    }
}
