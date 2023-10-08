package frc.robot.commands.superstructure;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.super_structure.States.SuperStructurePosition;

/**
 * Used to define State->State transition commands, to be used by
 * {@link StateManager}
 */
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
                    SuperStructurePosition.fromState(data.to));
        });
    }

    public static Command homeTransition(TransitionData data) {
        return data.superStructure.runEnd(
            () -> data.superStructure.home(),
            () -> OperatorPrefs.NEED_HOME = false
        );
    }

    public static Command stowTransition(TransitionData data) {
        return new Command() {
            private Integer cycles = 0;;

            @Override
            public void execute() {
                var b = data.superStructure.setSetpoint(
                        SuperStructurePosition.fromState(data.to));

                if (b) {
                    cycles++;
                }

                if (cycles > 50) {
                    var ampInfo = data.superStructure.getComponentAmps();
                    for (var compAmp : ampInfo) {
                        if (compAmp > 20) {
                            OperatorPrefs.NEED_HOME = true;
                            return;
                        }
                    }
                }
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(data.superStructure);
            }
        };
    }
}
