package frc.robot.subsystems.super_structure;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.super_structure.States.SuperStructureForm;

public class Visualizer {

    private final Mechanism2d mechanism;
    private final MechanismRoot2d rootCurrent, rootSetpoint;
    private final MechanismLigament2d elevatorCurrent, wristLowerCurrent, wristUpperCurrent;
    private final MechanismLigament2d elevatorSetpoint, wristLowerSetpoint, wristUpperSetpoint;

    private final static Double ELEVATOR_RANGE = Constants.kSuperStructure.Specs.ELEVATOR_MAX_METERS
            - Constants.kSuperStructure.Specs.ELEVATOR_MIN_METERS;

    public Visualizer() {
        mechanism = new Mechanism2d(2.0, 2.0);

        rootCurrent = mechanism.getRoot(
                "Pivot Current",
                Constants.kSuperStructure.Specs.PIVOT_OFFSET_METERS.getY(),
                Constants.kSuperStructure.Specs.PIVOT_OFFSET_METERS.getZ());
        elevatorCurrent = rootCurrent.append(new MechanismLigament2d(
                "Elevator Current",
                Constants.kSuperStructure.Specs.ELEVATOR_MIN_METERS,
                Constants.kSuperStructure.Specs.PIVOT_MIN_ANGLE));
        wristLowerCurrent = elevatorCurrent.append(new MechanismLigament2d(
                "Wrist Lower Current",
                0.32,
                -15));
        wristUpperCurrent = elevatorCurrent.append(new MechanismLigament2d(
                "Wrist Upper Current",
                0.43,
                20));

        rootSetpoint = mechanism.getRoot(
                "Pivot Setpoint",
                Constants.kSuperStructure.Specs.PIVOT_OFFSET_METERS.getY(),
                Constants.kSuperStructure.Specs.PIVOT_OFFSET_METERS.getZ());
        elevatorSetpoint = rootSetpoint.append(new MechanismLigament2d(
                "Elevator Setpoint",
                Constants.kSuperStructure.Specs.ELEVATOR_MIN_METERS,
                Constants.kSuperStructure.Specs.PIVOT_MIN_ANGLE));
        wristLowerSetpoint = elevatorSetpoint.append(new MechanismLigament2d(
                "Wrist Lower Setpoint",
                0.32,
                -15));
        wristUpperSetpoint = elevatorSetpoint.append(new MechanismLigament2d(
                "Wrist Upper Setpoint",
                0.43,
                20));

        elevatorSetpoint.setColor(new Color8Bit(170, 180, 180));
        wristLowerSetpoint.setColor(new Color8Bit(170, 180, 180));
        wristUpperSetpoint.setColor(new Color8Bit(170, 180, 180));
        elevatorSetpoint.setLineWeight(7.5);
        wristLowerSetpoint.setLineWeight(7.5);
        wristUpperSetpoint.setLineWeight(7.5);
    }

    public void updateCurrent(SuperStructureForm currentForm) {
        elevatorCurrent.setAngle(currentForm.pivotDegrees);
        elevatorCurrent.setLength(currentForm.elevatorMeters);

        // lerp the elevator color based on % of range
        // 0% = green, 100% = red
        Double percent = (currentForm.elevatorMeters - Constants.kSuperStructure.Specs.ELEVATOR_MIN_METERS) / ELEVATOR_RANGE;
        int red = (int) (percent * 255);
        int green = (int) ((1 - percent) * 255);
        elevatorCurrent.setColor(new Color8Bit(red, green, 0));

        wristLowerCurrent.setAngle(currentForm.wristDegrees - 15);
        wristUpperCurrent.setAngle(currentForm.wristDegrees + 20);

        var intakeState = currentForm.intakeDirection;
        Color8Bit intakeColor;
        switch (intakeState) {
            case IN:
                intakeColor = new Color8Bit(0, 255, 100);
                break;
            case OUT:
                intakeColor = new Color8Bit(255, 0, 100);
                break;
            case STOP:
            default:
                intakeColor = new Color8Bit(127, 127, 100);
                break;
        }

        wristLowerCurrent.setColor(intakeColor);
        wristUpperCurrent.setColor(intakeColor);
    }

    public void updateSetpoint(States state) {
        elevatorSetpoint.setAngle(state.pivotDegrees);
        elevatorSetpoint.setLength(state.elevatorMeters);
        wristLowerSetpoint.setAngle(state.wristDegrees - 15);
        wristUpperSetpoint.setAngle(state.wristDegrees + 20);
    }
}
