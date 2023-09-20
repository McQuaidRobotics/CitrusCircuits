package frc.robot.subsystems.super_structure;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.super_structure.elevator.Elevator;
import frc.robot.subsystems.super_structure.wrist.Wrist;
import frc.robot.subsystems.super_structure.wrist.WristReal;

public class SuperStructure extends SubsystemBase{
    private final Wrist wrist;
    private final Elevator elevator;
    private final Pivot pivot;

    private States currentState = States.START;

    public SuperStructure() {
        if (Robot.isReal()) {
            this.wrist = new WristReal();
            this.elevator = new Elevator();
            this.pivot = new Pivot();
        } else {
        }
        setupShuffleboard();
    }

    public void setState(States state) {
        this.wrist.setMechanismDegrees(state.wristDegrees);
        this.elevator.setMechanismMeters(state.elevatorMeters);
        this.pivot.setMechanismDegrees(state.pivotDegrees);
        this.currentState = state;
    }

    /**
     * To be used for debugging, not guranteed to have all
     * safety features
     * 
     * @param wristPercent of the wrist mechanisms motors
     * @param pivotPercent of the pivot mechanisms motors
     * @param elevatorPercent of the elevator mechanisms motors
     */
    public void manualControl(Double wristPercent, Double pivotPercent, Double elevatorPercent) {
        this.wrist.manualDriveMechanism(wristPercent);
        this.pivot.manualDriveMechanism(pivotPercent);
        this.elevator.manualDriveMechanism(elevatorPercent);
    }


    private Mechanism2d constructMechanism2d() {
        var mechanism2d = new Mechanism2d(2.0, 2.0);
        var root = mechanism2d.getRoot(
            "Pivot",
            Constants.SuperStructure.Specs.PIVOT_OFFSET_METERS.getY(),
            Constants.SuperStructure.Specs.PIVOT_OFFSET_METERS.getZ()
        );
        
        return mechanism2d;
    }


    /** once moved over to TEMPLATE this can be removed */
    private void setupShuffleboard() {
        var tab = Shuffleboard.getTab("SuperStructure");
        tab.addDouble("Wrist Setpoint Degrees", () -> {return this.currentState.wristDegrees;})
            .withSize(2, 1);
        tab.addDouble("Pivot Setpoint Degrees", () -> {return this.currentState.pivotDegrees;})
            .withSize(2, 1);
        tab.addDouble("Elevator Setpoint Meters", () -> {return this.currentState.elevatorMeters;})
            .withSize(2, 1);
        tab.addString("Current State", () -> {return this.currentState.name();})
            .withSize(2, 1);
    }


    @Override
    public void periodic() {
        this.wrist.periodic();
        this.elevator.periodic();
        this.pivot.periodic();
    }
}
