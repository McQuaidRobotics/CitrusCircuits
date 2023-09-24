package frc.robot.commands.superstructure;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.superstructure.Transitions.TransitionData;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.super_structure.States.IntakeBehavior;

public class StateManager {

    private static States lastState = States.START;

    @SuppressWarnings("unused")
    /**
     * Runs the given command when transitioning to the given state
     * @param state The state to transition to
     * @param cmd The command to run when transitioning
     * WARNING: be careful of the order you call the methods in
     */
    private static void toAllStates(States state, Function<TransitionData, Command> cmd) {
        HashMap<States, Function<TransitionData, Command>> map = new HashMap<>();
        for (var iState : States.values()) {
            map.put(iState, cmd);
        }
        transitions.put(state, map);
    }

    /**
     * Sets the transition from all states to the given state to the given command
     * @param state The state to transition to
     * @param cmd The command to run when transitioning
     * WARNING: be careful of the order you call the methods in
     */
    private static void fromAllStates(States state, Function<TransitionData, Command> cmd) {
        for (var iState : States.values()) {
            if (iState == state) continue;
            if (transitions.containsKey(iState)) {
                transitions.get(iState).put(state, cmd);
            } else {
                HashMap<States, Function<TransitionData, Command>> newMap = new HashMap<>();
                newMap.put(state, cmd);
                transitions.put(iState, newMap);
            }
        }
    }


    /**
     * A map of all possible transitions from one state to another,
     * default is simply setting the motors to the states' setpoints
     * WARNING: be careful of the order you edit the map in
     */
    private static Map<States, Map<States, Function<TransitionData, Command>>> transitions;

    static {
        fromAllStates(States.HOME, null);
    }


    private static Command getTransitionCmd(TransitionData data) {
        if (transitions.containsKey(data.from)) {
            if (transitions.get(data.from).containsKey(data.to)) {
                return transitions.get(data.from).get(data.to).apply(data);
            }
        }
        return Transitions.defaultTransition(data);
    }


    public static class CmdTransitionState extends CommandBase {
        private final SuperStructure superStructure;
        private final States to;
        private States from;
        private Command innerCmd;

        /**Can only be set in initialize, will skip x many cycles,
         * this also delays inner cmd initialize
         */
        private Integer deadCycles = 0;
        private Boolean innerInit = false;

        public CmdTransitionState(SuperStructure superStructure, States to) {
            this.superStructure = superStructure;
            this.to = to;
        }

        @Override
        public void initialize() {
            this.from = lastState;
            lastState = to;
            this.innerCmd = getTransitionCmd(new TransitionData(from, to, superStructure));
            if (from.intakeBehavior == IntakeBehavior.RUN_ON_TRANSITION) {
                superStructure.runEndEffector(from.intakeRequest.getVoltage());
                this.deadCycles = 10;
                this.innerInit = false;
                return;
            } else if (to.intakeBehavior == IntakeBehavior.RUN_ON_START || to.intakeBehavior == IntakeBehavior.RUN_WHOLE_TIME) {
                superStructure.runEndEffector(to.intakeRequest.getVoltage());
            }
            this.deadCycles = 0;
            this.innerCmd.initialize();
            this.innerInit = true;
        }

        @Override
        public void execute() {
            if (deadCycles > 0) {
                deadCycles--;
                return;
            }
            if (!innerInit) {
                this.innerCmd.initialize();
            }
            this.innerCmd.execute();
        }

        @Override
        public String getName() {
            if (from == null) {
                return "CmdTransitionState(? -> " + to + ")";
            } else {
                return "CmdTransitionState(" + from + " -> " + to + ")";
            }
        }
    }
}
