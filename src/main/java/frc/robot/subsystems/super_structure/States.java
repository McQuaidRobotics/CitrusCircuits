package frc.robot.subsystems.super_structure;

import frc.robot.GamepieceMode;

public enum States {
    START(10.0, 10.0, 10.0, IntakeRequest.IDLE, IntakeBehavior.RUN_WHOLE_TIME),
    HOME(10.0, 10.0, 10.0, IntakeRequest.IDLE, IntakeBehavior.RUN_WHOLE_TIME),
    STANDBY(10.0, 10.0, 10.0, IntakeRequest.IDLE, IntakeBehavior.RUN_WHOLE_TIME),
    PLACE_HIGH(10.0, 10.0, 10.0, IntakeRequest.OUTTAKING, IntakeBehavior.RUN_ON_TRANSITION),
    PLACE_MID(10.0, 10.0, 10.0, IntakeRequest.OUTTAKING, IntakeBehavior.RUN_ON_TRANSITION),
    PLACE_LOW(10.0, 10.0, 10.0, IntakeRequest.OUTTAKING, IntakeBehavior.RUN_ON_TRANSITION),
    PICKUP_GROUND(10.0, 10.0, 10.0, IntakeRequest.INTAKING, IntakeBehavior.RUN_ON_REACH),
    PICKUP_STATION(10.0, 10.0, 10.0, IntakeRequest.INTAKING, IntakeBehavior.RUN_ON_REACH);

    public final Double pivotDegrees;
    public final Double wristDegrees;
    public final Double elevatorMeters;
    public final IntakeRequest intakeRequest;
    public final IntakeBehavior intakeBehavior;

    States(
            Double pivotDegrees, Double wristDegrees, Double elevatorMeters,
            IntakeRequest intakeRequest, IntakeBehavior intakeBehavior) {
        this.pivotDegrees = pivotDegrees;
        this.wristDegrees = wristDegrees;
        this.elevatorMeters = elevatorMeters;
        this.intakeRequest = intakeRequest;
        this.intakeBehavior = intakeBehavior;
    }

    public enum IntakeRequest {
        IDLE(0.0, 0.0),
        INTAKING(12.0, -12.0),
        OUTTAKING(-12.0, 12.0),
        SPIT(8.0, -8.0);

        public final double voltageCone, voltageCube;

        IntakeRequest(double voltageCone, double voltageCube) {
            this.voltageCone = voltageCone;
            this.voltageCube = voltageCube;
        }

        public double getVoltage() {
            if (GamepieceMode.getCurrentMode() == GamepieceMode.CONE) {
                return voltageCone;
            } else {
                return voltageCube;
            }
        }
    }

    public enum IntakeBehavior {
        /** The entire time the state is active the intake will run the request */
        RUN_WHOLE_TIME,
        /** Runs at the start of the state until its setpoint is reached */
        RUN_ON_START,
        /** Runs once the setpoint is reached until a new state transition */
        RUN_ON_REACH,
        /** Run when the state is transitioning for 0.2 seconds */
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
        public boolean reachedState(SuperStructurePosition pose) {
            return Math.abs(pose.pivotDegrees - pivotDegrees) < 0.1 &&
                    Math.abs(pose.wristDegrees - wristDegrees) < 0.1 &&
                    Math.abs(pose.elevatorMeters - elevatorMeters) < 0.05;
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
