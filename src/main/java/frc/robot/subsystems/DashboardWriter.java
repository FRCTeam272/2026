// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.sub_containers.DriveBaseContainer;

public class DashboardWriter extends SubsystemBase {
  /** Creates a new Dashboard Writer. */
  private Alliance autoWinner;
  private boolean isOurTurnShooting = true;

  private final int[][] WINNER_SHOOT_TIMES = new int[][] {
    // start sec, end sec
    {timeToInt("1:20"), timeToInt("1:45")},  // TWO SHIFT
    {timeToInt("0:30"), timeToInt("0:55")}   // FOUR SHIFT
  };

  private final int[][] EVERYONE_SHOOT_TIMES = new int[][] {
    // start sec, end sec
    {timeToInt("2:10"), timeToInt("2:20")},  // TRANSITION SHIFT
    {timeToInt("0:30"), timeToInt("0:00")},  // END GAME SHIFT
  };
  
  public DashboardWriter() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Is Production", TunerConstants.isTestBot ? 0 : 1);
  }

  private void updateAutoWinner() {
    var gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() > 0) {
      autoWinner = (gameData.charAt(0) == 'B') ? Alliance.Red : Alliance.Blue;
    } 
  }

  private int timeToInt(String timeStr){
    String[] parts = timeStr.split(":");
    if (parts.length != 2) return 0;

    try {
      int minutes = Integer.parseInt(parts[0]);
      int seconds = Integer.parseInt(parts[1]);
      return minutes * 60 + seconds;
    } catch (NumberFormatException e) {
      return 0;
    }
  }

  private boolean isBetween(int time, int start, int end){
    return time >= start && time <= end;
  }

  private boolean updateOurTurnShooting(){
    var ourAlliance = DriverStation.getAlliance().get();
    var time = (int)DriverStation.getMatchTime();
    
    for (int[] values : EVERYONE_SHOOT_TIMES) {
      if (isBetween(time, values[0], values[1])) {
        return true;
      }
    }

    for (int[] values : WINNER_SHOOT_TIMES) {
      if (isBetween(time, values[0], values[1])) {
        return ourAlliance == autoWinner;
      }
    }

    return false;
  }

  @Override
  public void periodic() {
    // only allow for these to be managed when not connected to FMS
    if(!DriverStation.isFMSAttached()){
      double newSpeed = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Speed Factor", DriveBaseContainer.speedFactor);
      DriveBaseContainer.speedFactor = Math.max(-1.0, Math.min(1.0, newSpeed));
      
      double newRotation = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Rotation Factor", DriveBaseContainer.rotationFactor);
      DriveBaseContainer.rotationFactor = Math.max(-1.0, Math.min(1.0, newRotation));
    }

    // only run these checks if we are in teleop and connected to FMS
    if(DriverStation.isFMSAttached() && DriverStation.isTeleop()){
      if (autoWinner == null) updateAutoWinner();

      isOurTurnShooting = updateOurTurnShooting();
    }

    SmartDashboard.putBoolean("Is Our Turn Shooting", isOurTurnShooting);
  }
}
