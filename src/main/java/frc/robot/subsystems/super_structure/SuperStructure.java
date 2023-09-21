package frc.robot.subsystems.super_structure;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.super_structure.States.IntakeDirection;
import frc.robot.subsystems.super_structure.States.SuperStructureForm;
import frc.robot.subsystems.super_structure.pivot.*;
import frc.robot.subsystems.super_structure.wrist.*;

public class SuperStructure extends SubsystemBase{
    private final Wrist wrist;
    private final Elevator elevator;
    private final Pivot pivot;

    private final Visualizer visualizer = new Visualizer();

    private States currentState = States.START;

    public SuperStructure() {
        this.elevator = new Elevator();
        if (Robot.isReal()) {
            this.wrist = new WristReal();
            this.pivot = new PivotReal();
        } else {
            this.wrist = new WristSim();
            this.pivot = new PivotSim();
        }
        setupShuffleboard();
    }

    public void setState(States state) {
        this.visualizer.updateSetpoint(state);
        this.currentState = state;
        this.wrist.setMechanismDegrees(state.wristDegrees);
        this.elevator.setMechanismMeters(state.elevatorMeters);
        this.pivot.setMechanismDegrees(state.pivotDegrees);
    }


    public SuperStructureForm getForm() {
        return new SuperStructureForm(
            this.wrist.getMechanismDegrees(),
            this.pivot.getMechanismDegrees(),
            this.elevator.getMechanismMeters(),
            IntakeDirection.STOP //TODO
        );
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
        visualizer.updateCurrent(getForm());
        this.wrist.periodic();
        this.elevator.periodic();
        this.pivot.periodic();
    }
}
