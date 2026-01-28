
package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/*you should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbaced/orgaizing-command-baced.html#defineing-commands */
public class AdjustHood extends Command{

  /** Creates a new IntakeIntake. */
  Shooter shooter;
  boolean isDone = false;
   public AdjustHood(Shooter shooter){ 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.AdjustHood(50);
    System.out.println("Hey Sean");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isDone = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
