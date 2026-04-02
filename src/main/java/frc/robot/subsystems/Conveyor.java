package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.PIDSettings;
import frc.lib.utils.SparkMAXContainer;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
    public boolean ConveryorUsePID = false;
    public SparkMAXContainer conveyorMotor;
    public final int CONVEYOR_LOCATION = 7;
    private static PIDSettings conveyorPID = Constants.CONVEYOR_PID_SETTINGS;
    private double converyorVoltage = -.99;
    private double converyorVelocity = -1500;

    public Conveyor() {
        conveyorMotor = new SparkMAXContainer(CONVEYOR_LOCATION);
        conveyorMotor.assignPIDValues(5, 0, 0);
        conveyorMotor.setBreakMode(false);
        conveyorMotor.setMaxSpeed(1);
        SmartDashboard.putNumber("Conveyor/Velocity", converyorVelocity);
        SmartDashboard.putNumber("Conveyor/P", conveyorPID.kP);
        SmartDashboard.putNumber("Conveyor/I", conveyorPID.kI);
        SmartDashboard.putNumber("Conveyor/D", conveyorPID.kD);
        SmartDashboard.putBoolean("Conveyor/UsePID", ConveryorUsePID);
    }

    public void Load() {
        
        conveyorMotor.motor.set(-.99);
        // conveyorMotor.goToVoltage(-120);
    }

    public void Load(double amount) {
        this.Load();
        // conveyorMotor.goToVoltage(amount);
    }

    public void Unload() {
        conveyorMotor.goToVoltage(12);
    }

    public void Stop() {
        conveyorMotor.motor.set(0);
    }

    private void dynamicPID(double kP, double kI, double kD) {
        conveyorMotor.assignPIDValues(kP, kI, kD);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // if we don't see FMS, allow for dynamic PID tuning
        if (!DriverStation.isFMSAttached()) {
            // final double p = SmartDashboard.getNumber("Conveyor/P", conveyorPID.kP);
            // final double i = SmartDashboard.getNumber("Conveyor/I", conveyorPID.kI);
            // final double d = SmartDashboard.getNumber("Conveyor/D", conveyorPID.kD);
            // if(p != conveyorPID.kP || i != conveyorPID.kI || d != conveyorPID.kD){
            //     dynamicPID(p, i, d);
            // }
            
            // this.converyorVelocity = SmartDashboard.getNumber("Conveyor/Velocity", converyorVelocity);
            // this.converyorVoltage = SmartDashboard.getNumber("Conveyor/Voltage", converyorVoltage);
            // this.ConveryorUsePID = SmartDashboard.getBoolean("Conveyor/UsePID", ConveryorUsePID);
            // this.conveyorMotor.getPID("Converyor/PID_Actual/");
            conveyorMotor.reportMotor("Converyor");
        }

    }

}
