package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.PIDSettings;
import frc.lib.utils.SparkMAXContainer;
import frc.robot.Constants;

public class Regulator extends SubsystemBase {
    public boolean RegulatorUsePID = false;
    public SparkMAXContainer regulatorMotor;
    public final int REGULATOR_LOCATION = 8;
    private static PIDSettings regulatorPID = Constants.REGULATOR_PID_SETTINGS;
    private double regulatorSpeed = -.99;
    private double regulatorVelocity = 1500;

    public Regulator() {
        regulatorMotor = new SparkMAXContainer(REGULATOR_LOCATION);
        regulatorMotor.assignPIDValues(regulatorPID.kP, regulatorPID.kI, regulatorPID.kD);

        SmartDashboard.putNumber("Regulator/Speed", regulatorVelocity);

        SmartDashboard.putNumber("Regulator/P", regulatorPID.kP);
        SmartDashboard.putNumber("Regulator/I", regulatorPID.kI);
        SmartDashboard.putNumber("Regulator/D", regulatorPID.kD);

        SmartDashboard.putBoolean("Regulator/UsePID", RegulatorUsePID);
    }

    public void Load() {
        if (RegulatorUsePID) {
            regulatorMotor.setVelocity(regulatorVelocity);
        } else {
            regulatorMotor.motor.set(regulatorSpeed);
        }
    }

    public void Unload() {
        if (RegulatorUsePID) {
            regulatorMotor.setVelocity(-regulatorVelocity);
        } else {
            regulatorMotor.motor.set(-regulatorSpeed);
        }
    }

    public void Stop() {
        regulatorMotor.motor.set(0);
    }

    private void dynamicRegulatorPID(double kP, double kI, double kD) {
        regulatorMotor.assignPIDValues(kP, kI, kD);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // if we don't see FMS, allow for dynamic PID tuning
        if (!DriverStation.isFMSAttached()) {
            final double rp = SmartDashboard.getNumber("Regulator/P", regulatorPID.kP);
            final double ri = SmartDashboard.getNumber("Regulator/I", regulatorPID.kI);
            final double rd = SmartDashboard.getNumber("Regulator/D", regulatorPID.kD);
            
            if(rp != regulatorPID.kP || ri != regulatorPID.kI || rd != regulatorPID.kD){
                dynamicRegulatorPID(rp, ri, rd);
            }

            this.regulatorVelocity = SmartDashboard.getNumber("Regulator/Velocity", regulatorVelocity);
            this.regulatorSpeed = SmartDashboard.getNumber("Regulator/Speed", regulatorSpeed);
            this.RegulatorUsePID = SmartDashboard.getBoolean("Regulator/UsePID", RegulatorUsePID);

            this.regulatorMotor.getPID("Regulator/PID_Actual/");
            regulatorMotor.reportMotor("Regulator");
        }

    }
}
