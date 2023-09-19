package frc.robot.subsystems.super_structure;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants;
import frc.robot.subsystems.super_structure.States.SuperStructureForm;

public class Visualizer {

    Mechanism2d mechanism;
    MechanismRoot2d root;
    MechanismLigament2d elevator;
    MechanismLigament2d wristLower;
    MechanismLigament2d wristUpper;

    public Visualizer() {
        mechanism = new Mechanism2d(2.0, 2.0);
        root = mechanism.getRoot(
                "Pivot",
                Constants.SuperStructure.Specs.PIVOT_OFFSET_METERS.getY(),
                Constants.SuperStructure.Specs.PIVOT_OFFSET_METERS.getZ());
        elevator = root.append(new MechanismLigament2d(
                "Elevator",
                Constants.SuperStructure.Specs.ELEVATOR_MIN_METERS,
                Constants.SuperStructure.Specs.PIVOT_MIN_ANGLE));
        wristLower = elevator.append(new MechanismLigament2d(
                "Wrist Lower",
                0.32,
                -15));
        wristUpper = wristLower.append(new MechanismLigament2d(
                "Wrist Upper",
                0.43,
                20));
    }

    public void update(SuperStructureForm state) {
        elevator.setAngle(state.pivotDegrees);
        elevator.setLength(state.elevatorMeters);
        wristLower.setAngle(state.wristDegrees - 15);
        wristUpper.setAngle(state.wristDegrees + 20);
    }
}
