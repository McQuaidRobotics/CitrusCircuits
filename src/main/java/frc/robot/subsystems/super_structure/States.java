package frc.robot.subsystems.super_structure;

import frc.robot.Constants.kSuperStructure.*;
import frc.robot.GamepieceMode;


public enum States {
    HOME(kPivot.HOME_DEGREES, kWrist.HOME_DEGREES, kElevator.HOME_METERS, IntakeRequest.HOLD, IntakeBehavior.RUN_ON_START, true),
    STOW(kPivot.HOME_DEGREES, kWrist.HOME_DEGREES-3.0, kElevator.HOME_METERS, IntakeRequest.HOLD, IntakeBehavior.RUN_ON_START, true),
    STANDBY(kPivot.SCORE_DEGREES, kWrist.HOME_DEGREES, kElevator.HOME_METERS, IntakeRequest.HOLD, IntakeBehavior.RUN_ON_START, true),
    PLACE_HIGH(kPivot.SCORE_DEGREES, -17.2, kElevator.MAX_METERS*0.95, IntakeRequest.OUTTAKING, IntakeBehavior.RUN_ON_TRANSITION, true),
    PLACE_MID(kPivot.SCORE_DEGREES, -33.0, elevRelative(0.556), IntakeRequest.OUTTAKING, IntakeBehavior.RUN_ON_TRANSITION, true),
    PLACE_LOW(13.0, 15.0, kElevator.HOME_METERS, IntakeRequest.SPIT, IntakeBehavior.RUN_ON_TRANSITION, true),
    PICKUP_GROUND(kPivot.HOME_DEGREES, 30.0, kElevator.HOME_METERS, IntakeRequest.INTAKING, IntakeBehavior.RUN_ON_REACH, false, 1.3),
    PICKUP_STATION(61.1, -40.5, 1.1, IntakeRequest.INTAKING, IntakeBehavior.RUN_ON_REACH, false);

    public final Double pivotDegrees;
    public final Double wristDegrees;
    public final Double elevatorMeters;
    public final IntakeRequest intakeRequest;
    public final IntakeBehavior intakeBehavior;
    public final Boolean useHeldGamepiece;
    public final Double toleranceMult;

    States(
            Double pivotDegrees, Double wristDegrees, Double elevatorMeters,
            IntakeRequest intakeRequest, IntakeBehavior intakeBehavior,
            Boolean useHeldGamepiece) {
        this.pivotDegrees = pivotDegrees;
        this.wristDegrees = wristDegrees;
        this.elevatorMeters = elevatorMeters;
        this.intakeRequest = intakeRequest;
        this.intakeBehavior = intakeBehavior;
        this.useHeldGamepiece = useHeldGamepiece;
        this.toleranceMult = 1.0;
    }

    States(
            Double pivotDegrees, Double wristDegrees, Double elevatorMeters,
            IntakeRequest intakeRequest, IntakeBehavior intakeBehavior, Boolean useHeldGamepiece,
            Double toleranceMult) {
        this.pivotDegrees = pivotDegrees;
        this.wristDegrees = wristDegrees;
        this.elevatorMeters = elevatorMeters;
        this.intakeRequest = intakeRequest;
        this.intakeBehavior = intakeBehavior;
        this.useHeldGamepiece = useHeldGamepiece;
        this.toleranceMult = toleranceMult;
    }

    private static Double elevRelative(Double setpoint) {
        return setpoint + kElevator.MIN_METERS;
    }

    public enum IntakeRequest {
        IDLE(0.0, 0.0, 0.0),
        INTAKING(12.0, -12.0, 100.0),
        OUTTAKING(-12.0, 12.0, 100.0),
        SPIT(-8.0, 8.0, 40.0),
        HOLD(1.5, -1.5, 10.0);

        public final double voltageCone, voltageCube, maxCurrent;

        IntakeRequest(double voltageCone, double voltageCube, double maxCurrent) {
            this.voltageCone = voltageCone;
            this.voltageCube = voltageCube;
            this.maxCurrent = maxCurrent;
        }

        public double getVoltage(GamepieceMode mode) {
            if (mode == GamepieceMode.CONE) {
                return voltageCone;
            } else {
                return voltageCube;
            }
        }

        public double getCurrentLimit() {
            return this.maxCurrent;
        }
    }

    public enum IntakeBehavior {
        /** The entire time the state is active the intake will run the request */
        RUN_WHOLE_TIME,
        /** Runs at the start of the state until its setpoint is reached */
        RUN_ON_START,
        /** Runs once the setpoint is reached until a new state transition */
        RUN_ON_REACH,
        /** Run when the state is transitioning for ~0.4 seconds */
        RUN_ON_TRANSITION;
    }

    /**
     * A compact way of passing around data about the super structures current poses
     */
    public static class SuperStructurePosition {
        public final Double wristDegrees;
        public final Double pivotDegrees;
        public final Double elevatorMeters;
        public final Double intakeVoltage;

        public SuperStructurePosition(
                Double wristDegrees, Double pivotDegrees,
                Double elevatorMeters, Double intakeVoltage) {
            this.wristDegrees = wristDegrees;
            this.pivotDegrees = pivotDegrees;
            this.elevatorMeters = elevatorMeters;
            this.intakeVoltage = intakeVoltage;
        }

        /** Does not check intake */
        public boolean reachedState(SuperStructurePosition pose, Double toleranceMult) {
            return Math.abs(pose.pivotDegrees - pivotDegrees) < (kPivot.TOLERANCE * toleranceMult) &&
                    Math.abs(pose.wristDegrees - wristDegrees) < (kWrist.TOLERANCE * toleranceMult) &&
                    Math.abs(pose.elevatorMeters - elevatorMeters) < (kElevator.TOLERANCE * toleranceMult);
        }

        /** Does not set intake */
        public static SuperStructurePosition fromState(States state) {
            return new SuperStructurePosition(
                    state.wristDegrees, state.pivotDegrees,
                    state.elevatorMeters, 0.0);
        }

        public static SuperStructurePosition fromState(States state, Double intake) {
            return new SuperStructurePosition(
                    state.wristDegrees, state.pivotDegrees,
                    state.elevatorMeters, intake);
        }
    }
}
