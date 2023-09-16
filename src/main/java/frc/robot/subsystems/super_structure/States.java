package frc.robot.subsystems.super_structure;

public enum States {
    /**
     * TODO {Maddox} These are examples of states that could
     * exist, not REAL states make sure to replace these values 
     * with correct ones when the time comes
    */
    HOME(10.0, 10.0, 9.0),
    PLACE(70.0, 20.0, 9.0);

    public final Double pivotDegrees;
    public final Double wristDegrees;
    public final Double elevatorPercent;

    States(Double pivotDegrees, Double wristDegrees, Double elevatorPercent) {
        this.pivotDegrees = pivotDegrees;
        this.wristDegrees = wristDegrees;
        this.elevatorPercent = elevatorPercent;
    }

    // public final class StateValues {
    //     public final Double pivotDegrees;
    //     public final Double wristDegrees;

    //     public StateValues(Double pivotDegrees, Double wristDegrees) {
    //         this.pivotDegrees = pivotDegrees;
    //         this.wristDegrees = wristDegrees;
    //     }
    // }
}
