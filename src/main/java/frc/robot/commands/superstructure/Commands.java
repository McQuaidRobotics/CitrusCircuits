package frc.robot.commands.superstructure;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Helpers;
import frc.robot.subsystems.super_structure.SuperStructure;

public class Commands {

    /**Inputs are halved */
    public static Command manualControl(
        SuperStructure superStructure,
        DoubleSupplier elevator,
        DoubleSupplier pivot,
        DoubleSupplier wrist,
        DoubleSupplier intake
    ) {
        var dbElevator = Helpers.deadbandSupplier(elevator, 0.1);
        var dbPivot = Helpers.deadbandSupplier(pivot, 0.1);
        var dbWrist = Helpers.deadbandSupplier(wrist, 0.1);

        return superStructure.run(() -> {
            superStructure.manualControl(
                dbWrist.getAsDouble() / 2.0,
                dbPivot.getAsDouble() / 2.0,
                dbElevator.getAsDouble() / 2.0,
                intake.getAsDouble()
            );
        });
    }

    
}