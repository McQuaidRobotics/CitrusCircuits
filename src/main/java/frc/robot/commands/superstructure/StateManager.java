package frc.robot.commands.superstructure;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GamepieceMode;
import frc.robot.commands.superstructure.Transitions.TransitionData;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
// import frc.robot.subsystems.super_structure.States.IntakeBehavior;
import frc.robot.subsystems.super_structure.States.IntakeBehavior;
import frc.robot.subsystems.super_structure.States.IntakeRequest;

/**
 * Acts as a stateful interface for the {@link SuperStructure}.
 * State transitions can be defined in {@link Transitions}.
 * The only public member is the {@link CmdTransitionState} command and should
 * be used as the main way of controlling the {@link SuperStructure}.
 */
public class StateManager {

    /**
     * Is essential for determining transitions,
     * can only be mutated by {@link CmdTransitionState}
     */
    private static States lastState = States.START;

    @SuppressWarnings("unused")
    /**
     * Runs the given command when transitioning to the given state
     * 
     * @param state The state to transition to
     * @param cmd   The command to run when transitioning
     *              WARNING: be careful of the order you call the methods in
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
     * 
     * @param state The state to transition to
     * @param cmd   The command to run when transitioning
     *              WARNING: be careful of the order you call the methods in
     */
    private static void fromAllStates(States state, Function<TransitionData, Command> cmd) {
        for (var iState : States.values()) {
            if (iState == state)
                continue;
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
    private static Map<States, Map<States, Function<TransitionData, Command>>> transitions = new HashMap<>();

    static {
        // without this the superstructure will never reseed
        fromAllStates(States.HOME, Transitions::homeTransition);
        // fromAllStates(States.STOW, Transitions::stowTransition);
    }

    /**
     * @param data
     * @return The command to run when transitioning from one state to another
     */
    private static Command getTransitionCmd(TransitionData data) {
        if (transitions.containsKey(data.from)) {
            if (transitions.get(data.from).containsKey(data.to)) {
                return transitions.get(data.from).get(data.to).apply(data);
            }
        }
        return Transitions.defaultTransition(data);
    }

    private static Double intakeVoltage(IntakeRequest req, Boolean useHeld) {
        if (useHeld) {
            var held = GamepieceMode.getHeldPiece();
            if (held == null) {
                return 0.0;
            }
            return req.getVoltage(held);
        } else {
            return req.getVoltage(
                GamepieceMode.getDesiredPiece()
            );
        }
    }

    private static Double intakeVoltage(States to) {
        return intakeVoltage(to.intakeRequest, to.useHeldGamepiece);
    }

    public static class CmdTransitionState extends CommandBase {
        private final SuperStructure superStructure;
        private final States to;
        private States from;
        private Command innerCmd;

        /**
         * Can only be set in initialize, will skip x many cycles,
         * this also delays inner cmd initialize
         */
        private Integer deadCycles = 0;

        // inner cmd tracking
        private Boolean innerInit = false;
        private Boolean innerFinish = false;

        private Boolean reachedSetpoint;

        public CmdTransitionState(final SuperStructure superStructure, final States to) {
            this.superStructure = superStructure;
            this.to = to;
            addRequirements(superStructure);
        }

        @Override
        public void initialize() {
            this.from = lastState;
            StateManager.lastState = to;
            this.innerCmd = getTransitionCmd(new TransitionData(from, to, superStructure));
            this.innerInit = false;
            this.innerFinish = false;
            this.reachedSetpoint = false;
            this.deadCycles = 0;
            if (from.intakeBehavior == IntakeBehavior.RUN_ON_TRANSITION
                    && to.intakeBehavior != IntakeBehavior.RUN_ON_TRANSITION) {
                superStructure.runEndEffector(intakeVoltage(from), from.intakeRequest.getCurrentLimit());
                this.deadCycles = 20;
            }
        }

        @Override
        public void execute() {
            // skipping dead cycles
            if (deadCycles > 0) {
                deadCycles--;
                return;
            }

            // inner command handling
            if (!innerInit) {
                this.innerCmd.initialize();
                this.innerInit = true;
            }
            if (!innerFinish) {
                this.innerCmd.execute();
            }
            if (innerCmd.isFinished()) {
                this.innerCmd.end(false);
                this.innerFinish = true;
            }

            if (superStructure.reachedSetpoint(to.toleranceMult) && !this.reachedSetpoint) {
                this.reachedSetpoint = true;
            }

            // solving intake behavior
            Double endEffectorVolts = 0.0;

            if (to.intakeBehavior == IntakeBehavior.RUN_ON_TRANSITION) {
                endEffectorVolts = intakeVoltage(IntakeRequest.HOLD, to.useHeldGamepiece);
            }

            if (to.intakeBehavior == IntakeBehavior.RUN_WHOLE_TIME
                    || to.intakeBehavior == IntakeBehavior.RUN_ON_START) {
                // SmartDashboard.putString("intake run type", "START/WHOLE");
                endEffectorVolts = intakeVoltage(to);
            }
            if (reachedSetpoint) {
                if (to.intakeBehavior == IntakeBehavior.RUN_ON_START) {
                    // SmartDashboard.putString("intake run type", "stop cuz not start");
                    endEffectorVolts = 0.0;
                } else if (to.intakeBehavior == IntakeBehavior.RUN_ON_REACH) {
                    // SmartDashboard.putString("intake run type", "Run on reach");
                    endEffectorVolts = intakeVoltage(to);
                }
            }

            superStructure.runEndEffector(endEffectorVolts, to.intakeRequest.getCurrentLimit());
        }

        @Override
        public void end(boolean interrupted) {
            superStructure.stopAll();
            from = null;
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
