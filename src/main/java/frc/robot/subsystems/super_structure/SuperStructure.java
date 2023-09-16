package frc.robot.subsystems.super_structure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase{
    private Pivot pivot;
    private Wrist wrist;
    private Elevator elevator;

    public void setState(States state) {

    }

    @Override
    public void periodic() {
        this.wrist.periodic();
        this.elevator.periodic();
        this.pivot.periodic();
    }
}
