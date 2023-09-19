package frc.robot.subsystems.super_structure;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SuperStructure;;


public class Pivot {

    /**Left */
    private final TalonFX pivotLeaderMotor;
    /**Right */
    private final TalonFX pivotFollowerMotor;


    public Pivot() {
        pivotLeaderMotor = new TalonFX(SuperStructure.Pivot.LEFT_MOTOR_ID);
        pivotFollowerMotor = new TalonFX(SuperStructure.Pivot.RIGHT_MOTOR_ID);

    }

    private void getMotorConfig(boolean inverted) {

    }

    /**
     * Abstract from motors, will set the pivots degrees
     * parallel to floor is 0 degrees
     */
    public void setMechanismDegrees(Double setpoint) {}

    /**
     * @return the current angle of the mechanism
     */
    public Double getMechanismDegrees() {return null;}

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
