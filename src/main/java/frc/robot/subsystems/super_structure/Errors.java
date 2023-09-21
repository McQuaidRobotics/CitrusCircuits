package frc.robot.subsystems.super_structure;

import frc.robot.util.ErrorHelper.ErrorEnum;
import frc.robot.util.ErrorHelper.GroupError;

public class Errors {
    
    public enum SuperStructureErrors implements ErrorEnum {
        SetpointTooHigh,
        SetpointTooLow,
        CanTimeout;
    }

    public static class SetpointTooHigh implements GroupError<SuperStructureErrors> {
        private final String message;

        /**
         * Not unit specific.
         * @param limit of the setpoint
         * @param desired setpoint
         */
        public SetpointTooHigh(Double limit, Double desired) {
            this.message = "desired: " + desired + " > limit: " + limit;
        }

        @Override
        public String msg() {
            return message;
        }

        @Override
        public SuperStructureErrors getVariant() {
            return SuperStructureErrors.SetpointTooHigh;
        }
    }

    public static class SetpointTooLow implements GroupError<SuperStructureErrors> {
        private final String message;

        /**
         * Not unit specific.
         * @param limit of the setpoint
         * @param desired setpoint
         */
        public SetpointTooLow(Double limit, Double desired) {
            this.message = "desired: " + desired + " < limit: " + limit;
        }

        @Override
        public String msg() {
            return message;
        }

        @Override
        public SuperStructureErrors getVariant() {
            return SuperStructureErrors.SetpointTooLow;
        }
    }

    public static class CanTimeout implements GroupError<SuperStructureErrors> {
        private final String message;

        public CanTimeout(Integer id, String errorMessage) {
            this.message = "can id: " + id + ", message: " + errorMessage;
        }

        @Override
        public String msg() {
            return message;
        }

        @Override
        public SuperStructureErrors getVariant() {
            return SuperStructureErrors.CanTimeout;
        }
    }
}
