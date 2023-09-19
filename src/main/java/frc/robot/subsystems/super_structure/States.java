package frc.robot.subsystems.super_structure;

public enum States {
    START(10.0, 10.0, 10.0),
    HOME(10.0, 10.0, 10.0),
    PLACE(10.0, 10.0, 10.0),
    PICKUP_FLOOR(10.0, 10.0, 10.0),
    PICKUP_STATION(10.0, 10.0, 10.0);

    public final Double pivotDegrees;
    public final Double wristDegrees;
    public final Double elevatorMeters;
    public final IntakeDirection intakeDirection = IntakeDirection.STOP;

    States(Double pivotDegrees, Double wristDegrees, Double elevatorMeters) {
        this.pivotDegrees = pivotDegrees;
        this.wristDegrees = wristDegrees;
        this.elevatorMeters = elevatorMeters;
    }

    public enum IntakeDirection {
        IN, OUT, STOP
    }

    /**A compact way of passing around data about the super structures current poses */
    public static class SuperStructureForm {
        public final double wristDegrees;
        public final double pivotDegrees;
        public final double elevatorMeters;
        public final IntakeDirection intakeDirection;

        public SuperStructureForm(double wristDegrees, double pivotDegrees, double elevatorMeters,
                IntakeDirection intakeDirection) {
            this.wristDegrees = wristDegrees;
            this.pivotDegrees = pivotDegrees;
            this.elevatorMeters = elevatorMeters;
            this.intakeDirection = intakeDirection;
        }
    }
}
