package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.super_structure.States.SuperStructurePosition;

/**Used to define State->State transition commands, to be used by {@link StateManager} */
public class Transitions {

    public static class TransitionData {
        public final States from, to;
        public final SuperStructure superStructure;

        public TransitionData(States from, States to, SuperStructure superStructure) {
            this.from = from;
            this.to = to;
            this.superStructure = superStructure;
        }
    }


    public static Command defaultTransition(TransitionData data) {
        return data.superStructure.run(() -> {
            data.superStructure.setSetpoint(
                SuperStructurePosition.fromState(data.to)
            );
        });
    }

    public static Command homeTransition(TransitionData data) {
        return data.superStructure.run(() -> data.superStructure.home());
    }
}
