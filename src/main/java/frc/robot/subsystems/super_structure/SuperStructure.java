package frc.robot.subsystems.super_structure;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
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
            this.wrist = new WristReal(setpoint.wristDegrees);
            this.pivot = new PivotReal(setpoint.pivotDegrees);
            this.elevator = new ElevatorReal(setpoint.elevatorMeters);
        } else {
            this.wrist = new WristSim(setpoint.wristDegrees);
            this.pivot = new PivotSim(setpoint.pivotDegrees);
            this.elevator = new ElevatorSim(setpoint.elevatorMeters);
        }
        setupShuffleboard();
        visualizer.updateSetpoint(setpoint);
        visualizer.updateCurrent(setpoint);
    }

    /** @returns true of the setpoint has been reached */
    public Boolean setSetpoint(SuperStructurePosition to) {
        this.visualizer.updateSetpoint(to);
        this.setpoint = to;

        // only pivot or wrist+elevator should run at a time
        BooleanSupplier runWristElevatorParallel = () -> {
            var wrist = this.wrist.setMechanismDegrees(to.wristDegrees);
            var elev = this.elevator.setMechanismMeters(to.elevatorMeters);
            return elev && wrist;
        };
        // BooleanSupplier runWristElevatorSeq = () -> {
        // return this.wrist.setMechanismDegrees(to.wristDegrees)
        // && this.elevator.setMechanismMeters(to.elevatorMeters);
        // };
        BooleanSupplier runPivot = () -> {
            return this.pivot.setPivotDegrees(to.pivotDegrees);
        };

        // We need to make sure the pivot and elevator do not move at the same time.
        // We need to also try and run the pivot when the elevator is in the least
        // extended between the two states(pose and current).

        // i.e. if the elevator is currently not extended but the pose wants it all the
        // way out,
        // we will move the pivot first to reduce stress, but if the current elevator
        // extension
        // is greater than set pose we move elevator first

        if (this.elevator.getMechanismMeters() > to.elevatorMeters) {
            // check if wrist and elevator have reached their setpoints
            // if they have, run pivot
            if (runWristElevatorParallel.getAsBoolean()) {
                return runPivot.getAsBoolean();
            }
        } else {
            if (runPivot.getAsBoolean()) {
                return runWristElevatorParallel.getAsBoolean();
            }
        }
        return false;
    }

    public Boolean stow(Boolean toZero) {
        this.setpoint = SuperStructurePosition.fromState(States.HOME);
        this.visualizer.updateSetpoint(this.setpoint);
        // this will do wrist -> elevator -> pivot
        return this.wrist.stowMechanism(toZero)
                && this.elevator.stowMechanism(toZero)
                && this.pivot.stowMechanism(false);
    }

    public void runEndEffector(Double volts, Double currentLimit) {
        // when outtaking this should be false
        this.wrist.setIntakeCurrentLimits(currentLimit);
        // assume max voltage is 12.0
        this.wrist.runIntake(volts / 12.0);
    }

    public void stopAll() {
        this.wrist.stopMechanism();
        this.pivot.stopMechanism();
        this.elevator.stopMechanism();
        this.wrist.runIntake(0.0);
    }

    public Boolean reachedSetpoint() {
        return this.setpoint.reachedState(this.getPose());
    }

    public SuperStructurePosition getPose() {
        return new SuperStructurePosition(
                this.wrist.getMechanismDegrees(),
                this.pivot.getPivotDegrees(),
                this.elevator.getMechanismMeters(),
                this.wrist.getIntakeVoltage());
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
        // this.wrist.runIntake(intakePercent);
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
        tab.addDouble("Pivot Current Degrees", () -> this.pivot.getPivotDegrees())
                .withSize(2, 1);
        tab.addDouble("Elevator Current Meters", () -> this.elevator.getMechanismMeters())
                .withSize(2, 1);

        tab.addString("Current Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "None")
                .withSize(2, 1);

        wrist.setupShuffleboard(tab.getLayout("Wrist", BuiltInLayouts.kList));
        pivot.setupShuffleboard(tab.getLayout("Pivot", BuiltInLayouts.kList));
        elevator.setupShuffleboard(tab.getLayout("Elevator", BuiltInLayouts.kList));

        visualizer.setShuffleboardTab(tab);
    }

    @Override
    public void periodic() {
        this.wrist.periodic();
        this.elevator.periodic();
        this.pivot.periodic();

        visualizer.updateCurrent(getPose());

        if (DriverStation.isDisabled() && this.getCurrentCommand() != null) {
            this.getCurrentCommand().cancel();
        }
    }
}
