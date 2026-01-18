// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.sub_containers.DriveBaseContainer;

public class DashboardWriter extends SubsystemBase {
  /** Creates a new Dashboard Writer. */
  private Alliance autoWinner;
  private boolean isOurTurnShooting;

  private final int[][] WINNER_SHOOT_TIMES = new int[][] {
    // start sec, end sec
    {timeToInt("1:45"), timeToInt("1:20")},  // Period Two
    {timeToInt("0:55"), timeToInt("0:00")}   // Period Four
  };

  private final int[][] EVERYONE_SHOOT_TIMES = new int[][] {
    // start sec, end sec
    {timeToInt("2:20"), timeToInt("2:10")},  // Period One
    {timeToInt("0:55"), timeToInt("0:30")},  // Period Three
  };
  
  public DashboardWriter() {}

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

  private boolean updateOurTurnShootingProd(){
    var ourAlliance = DriverStation.getAlliance().get();
    var time = DriverStation.getMatchTime();
    
    for (int[] values : EVERYONE_SHOOT_TIMES) {
      if (isBetween((int)time, values[0], values[1])) {
        return true;
      }
    }

    for (int[] values : WINNER_SHOOT_TIMES) {
      if (isBetween((int)time, values[0], values[1])) {
        return ourAlliance == autoWinner;
      }
    }

    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Is Production", TunerConstants.isTestBot ? 0 : 1);

    double newSpeed = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Speed Factor", DriveBaseContainer.speedFactor);
    DriveBaseContainer.speedFactor = Math.max(-1.0, Math.min(1.0, newSpeed));
    
    double newRotation = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber("Rotation Factor", DriveBaseContainer.rotationFactor);
    DriveBaseContainer.rotationFactor = Math.max(-1.0, Math.min(1.0, newRotation));

    if (autoWinner == null) updateAutoWinner();

    if(DriverStation.isTeleop()){
      // match time counts up in home mode, down in production mode
      // if (DriverStation.isFMSAttached()) 
        isOurTurnShooting = updateOurTurnShootingProd();
    }

    SmartDashboard.putBoolean("Is Our Turn Shooting", isOurTurnShooting);
  }
}
