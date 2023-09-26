package frc.robot.subsystems.super_structure;

import java.util.function.BooleanSupplier;

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

    /**@returns true of the setpoint has been reached */
    public Boolean setSetpoint(SuperStructurePosition pose) {
        this.visualizer.updateSetpoint(pose);

        //only pivot or wrist+elevator should run at a time
        BooleanSupplier runWristElevator = () -> {
            return this.wrist.setMechanismDegrees(pose.wristDegrees) &&
            this.elevator.setMechanismMeters(pose.elevatorMeters);
        };
        BooleanSupplier pivot = () -> {
            return this.pivot.setMechanismDegrees(pose.pivotDegrees);
        };

        //We need to make sure the pivot and elevator do not move at the same time.
        //We need to also try and run the pivot when the elevator is in the least
        //extended between the two states(pose and current).

        //i.e. if the elevator is currently not extended but the pose wants it all the way out,
        //we will move the pivot first to reduce stress, but if the current elevator extension
        //is greater than set pose we move elevator first

        if (this.elevator.getMechanismMeters() - 0.1 < pose.elevatorMeters) {
            //check if wrist and elevator have reached their setpoints
            //if they have, run pivot
            if (runWristElevator.getAsBoolean()) {
                return pivot.getAsBoolean();
            }
        } else {
            if (pivot.getAsBoolean()) {
                return runWristElevator.getAsBoolean();
            }
        }
        return false;
    }

    public Boolean home() {
        //always home elevator/wrist then home pivot
        if (this.elevator.homeMechanism() && this.wrist.homeMechanism()) {
            return this.pivot.homeMechanism();
        }
        return false;
    }

    public void runEndEffector(Double volts) {
        //assume max voltage is 12.0
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

        tab.addDouble("Wrist Current Degrees", () -> this.wrist.getMechanismDegrees())
                .withSize(2, 1);
        tab.addDouble("Pivot Current Degrees", () -> this.pivot.getMechanismDegrees())
                .withSize(2, 1);
        tab.addDouble("Elevator Current Degrees", () -> this.elevator.getMechanismMeters())
                .withSize(2, 1);

        tab.addString("Current Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "None")
                .withSize(2, 1);

        wrist.setupShuffleboard(tab.getLayout("Wrist"));
        pivot.setupShuffleboard(tab.getLayout("Pivot"));
        elevator.setupShuffleboard(tab.getLayout("Elevator"));
    }

    @Override
    public void periodic() {
        visualizer.updateCurrent(getPose());
        this.wrist.periodic();
        this.elevator.periodic();
        this.pivot.periodic();
    }
}
