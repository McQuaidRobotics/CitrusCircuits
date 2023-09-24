package frc.robot.subsystems.super_structure;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.super_structure.elevator.Elevator;
import frc.robot.subsystems.super_structure.elevator.ElevatorReal;
import frc.robot.subsystems.super_structure.elevator.ElevatorSim;
import frc.robot.subsystems.super_structure.wrist.Wrist;
import frc.robot.subsystems.super_structure.wrist.WristReal;
import frc.robot.subsystems.super_structure.States.SuperStructurePosition;
import frc.robot.subsystems.super_structure.pivot.*;
import frc.robot.subsystems.super_structure.wrist.*;

public class SuperStructure extends SubsystemBase {
    private final Wrist wrist;
    private final Elevator elevator;
    private final Pivot pivot;

    private final Visualizer visualizer = new Visualizer();

    private SuperStructurePosition setpoint = SuperStructurePosition.fromState(States.START);

    public SuperStructure() {
        if (Robot.isReal()) {
            this.wrist = new WristReal();
            this.pivot = new PivotReal();
            this.elevator = new ElevatorReal();
        } else {
            this.wrist = new WristSim();
            this.pivot = new PivotSim();
            this.elevator = new ElevatorSim();
        }
        setupShuffleboard();
    }

    public void setSetpoint(SuperStructurePosition pose) {
        this.visualizer.updateSetpoint(pose);
        this.wrist.setMechanismDegrees(pose.wristDegrees);
        this.elevator.setMechanismMeters(pose.elevatorMeters);
        this.pivot.setMechanismDegrees(pose.pivotDegrees);
    }

    public void runEndEffector(Double volts) {
        this.wrist.runIntake(volts / 12.0);
    }

    public Boolean reachedSetpoint() {
        return this.setpoint.reachedState(this.getPose());
    }

    public SuperStructurePosition getPose() {
        return new SuperStructurePosition(
                this.wrist.getMechanismDegrees(),
                this.pivot.getMechanismDegrees(),
                this.elevator.getMechanismMeters(),
                this.wrist.getIntakeVoltage()
        );
    }

    /**
     * To be used for debugging, not guranteed to have all
     * safety features
     * 
     * @param wristPercent    of the wrist mechanisms motors
     * @param pivotPercent    of the pivot mechanisms motors
     * @param elevatorPercent of the elevator mechanisms motors
     */
    public void manualControl(Double wristPercent, Double pivotPercent, Double elevatorPercent, Double intakePercent) {
        this.wrist.manualDriveMechanism(wristPercent);
        this.pivot.manualDriveMechanism(pivotPercent);
        this.elevator.manualDriveMechanism(elevatorPercent);
        this.wrist.runIntake(intakePercent);
    }

    /** once moved over to TEMPLATE this can be removed */
    private void setupShuffleboard() {
        var tab = Shuffleboard.getTab("SuperStructure");
        tab.addDouble("Wrist Setpoint Degrees", () -> this.setpoint.wristDegrees)
                .withSize(2, 1);
        tab.addDouble("Pivot Setpoint Degrees", () -> this.setpoint.pivotDegrees)
                .withSize(2, 1);
        tab.addDouble("Elevator Setpoint Meters", () -> this.setpoint.elevatorMeters)
                .withSize(2, 1);

        var form = getPose();

        tab.addDouble("Wrist Current Degrees", () -> form.wristDegrees)
                .withSize(2, 1);
        tab.addDouble("Pivot Current Degrees", () -> form.pivotDegrees)
                .withSize(2, 1);
        tab.addDouble("Elevator Current Degrees", () -> form.elevatorMeters)
                .withSize(2, 1);

        tab.addString("Current Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "None")
                .withSize(2, 1);

        wrist.setupShuffleboard(tab);
    }

    @Override
    public void periodic() {
        visualizer.updateCurrent(getPose());
        this.wrist.periodic();
        this.elevator.periodic();
        this.pivot.periodic();
    }
}
