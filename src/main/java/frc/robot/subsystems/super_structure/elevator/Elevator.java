package frc.robot.subsystems.super_structure.elevator;

public class Elevator {
    public Elevator() {}

    private void configureMotors() {};

    public void setMechanismMeters(Double percent) {}

    public Double getMechanismMeters() {return frc.robot.Constants.SuperStructure.Specs.ELEVATOR_MIN_METERS;}

    /**
     * To be used for debugging, not guranteed to have all
     * safety features
     * 
     * @param percentOut of the mechanisms motors
     */
    public void manualDriveMechanism(Double percentOut) {
        // var percentControlRequest = new DutyCycleOut(percentOut, true, false);
        // this.wristMotor.setControl(percentControlRequest);
    }

    public void periodic() {}
}