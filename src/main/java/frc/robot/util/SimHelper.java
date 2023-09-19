package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class SimHelper {

    //https://github.com/mahmoud-a-ali/scurve_traj_generation/tree/master
    //https://www.trajectorygenerator.com/ojet-online/

    private static class StateVector {
        public final Double position, velocity, acceleration;

        public StateVector(Double position, Double velocity, Double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }

        public static StateVector empty() {
            return new StateVector(0.0, 0.0, 0.0);
        }
    }

    private static class Boundries {
        public final Double maxVelocity, maxAcceleration, maxJerk;

        public Boundries(Double maxVelocity, Double maxAcceleration, Double maxJerk) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxJerk = maxJerk;
        }

        public Boundries symetric() {
            return new Boundries(-maxVelocity, -maxAcceleration, -maxJerk);
        }
    }

    public static class SetPoint {
        private final Boundries upper, lower;
        private StateVector start_state, end_state;
        private Double start_time;
        private Boolean has_setpoint;

        public SetPoint(Double maxVelocity, Double maxAcceleration, Double maxJerk) {
            this.upper = new Boundries(maxVelocity, maxAcceleration, maxJerk);
            this.lower = upper.symetric();
            this.start_state = StateVector.empty();
            this.end_state = StateVector.empty();
            this.start_time = 0.0;
            this.has_setpoint = false;
        }

        public void setTarget(StateVector target) {
            this.end_state = target;
            this.has_setpoint = true;
            this.start_time = Timer.getFPGATimestamp();
        }


        public Double getPose() {
            if (!has_setpoint) {
                return 0.0;
            }
            return this.end_state.position;
            // TODO: implement
        }

        public Double getVelocity() {
            if (!has_setpoint) {
                return 0.0;
            }
            return this.end_state.velocity;
            // TODO: implement
        }
    }
}
