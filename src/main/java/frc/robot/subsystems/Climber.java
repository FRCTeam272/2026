package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.PIDSettings;
import frc.lib.utils.SparkMAXContainer;
import frc.lib.utils.TrimPot;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    public final int climberMotorID = 9;
    SparkMAXContainer climberMotor = new SparkMAXContainer(climberMotorID);

    public double raisedTarget = 0.0;
    public double loweredTarget = 0.0;

    PIDSettings pid = Constants.CLIMBER_PID_SETTINGS;

    public TrimPot trim = new TrimPot("Climber");

    double lastP, lastI, lastD, lastG;

    public Climber() {
        climberMotor.assignPIDValues(pid.kP, pid.kI, pid.kD);
        SmartDashboard.putNumber("Climber/Stage1", 10.7 * 2.78);
        SmartDashboard.putNumber("Climber/Stage2", 26* 2.78);
        SmartDashboard.putNumber("Climber/Stage3", -7* 2.78);
        SmartDashboard.putNumber("Climber/Stage4", -7* 2.78);
        SmartDashboard.putNumber("Climber/Dismount", 26* 2.78);
        SmartDashboard.putNumber("Climber/Executing", 0* 2.78);
        SmartDashboard.putNumber("Climber/PID/P", pid.kP);
        SmartDashboard.putNumber("Climber/PID/I", pid.kI);
        SmartDashboard.putNumber("Climber/PID/D", pid.kD);
        SmartDashboard.putNumber("Climber/PID/G", pid.kG);
        
        climberMotor.setCurrentLimit(40);
    }

    public void setValue(double value){
        climberMotor.motor.set(value);
    }

    public boolean RaiseToPoint(){
        return climberMotor.goToPostion(raisedTarget + trim.adjusterValue);
    }

    public boolean LowerToPoint(){
        return climberMotor.goToPostion(loweredTarget + trim.adjusterValue);
    }

    public boolean SetHeight(double value){
        return climberMotor.goToPostion(value, 0.5);
    }

    public boolean Zero(){
        return climberMotor.goToPostion(0);
    }

    public void Stop(){
        climberMotor.motor.set(0);
    }

    public void assignPID(double P, double I, double D){
        climberMotor.assignPIDValues(P, I, D);
    }

    public void assignFF(double kG){
        climberMotor.assignFF(0, 0, 0, kG);
    }
    
    @Override
    public void periodic() {
        

        SmartDashboard.putBoolean("Ready/Can Trench(Climber)", this.climberMotor.getPosition() < 2);
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Climber/Current", climberMotor.encoder.getPosition());
        climberMotor.reportMotor("ClimberMotor");
        climberMotor.getPID("ClimberMotor/PID/");
        var p = SmartDashboard.getNumber("Climber/PID/P", lastP);
        var i = SmartDashboard.getNumber("Climber/PID/I", lastI);
        var d = SmartDashboard.getNumber("Climber/PID/D", lastD);
        var g = SmartDashboard.getNumber("Climber/PID/G", lastG);

        if(p != lastP || i != lastI || d != lastD){
            assignPID(p, i, d);
            lastP = p;
            lastI = i;
            lastD = d;
        }
        if (g != lastG){
            assignFF(g);
            lastG = g;
        }
    }
}
