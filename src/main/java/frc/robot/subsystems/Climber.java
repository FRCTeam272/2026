package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.PIDSettings;
import frc.lib.utils.SparkMAXContainer;
import frc.lib.utils.TrimPot;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    public final int climberMotorID = 6;
    SparkMAXContainer climberMotor = new SparkMAXContainer(climberMotorID);

    public double raisedTarget = 0.0;
    public double loweredTarget = 0.0;

    PIDSettings pid = Constants.CLIMBER_PID_SETTINGS;

    public TrimPot trim = new TrimPot("Climber");

    public Climber() {
        climberMotor.assignPIDValues(pid.kP, pid.kI, pid.kD);
    }

    public boolean Raise(){
        return climberMotor.goToPostion(raisedTarget + trim.adjusterValue);
    }

    public boolean Lower(){
        return climberMotor.goToPostion(loweredTarget + trim.adjusterValue);
    }

    public boolean Zero(){
        return climberMotor.goToPostion(0);
    }

    public void Stop(){
        climberMotor.motor.set(0);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
