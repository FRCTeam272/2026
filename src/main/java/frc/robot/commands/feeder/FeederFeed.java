package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class FeederFeed extends Command {
    
    Feeder feeder;
    boolean isDone=false;
    public FeederFeed(Feeder feeder) {
        addRequirements(feeder);
        this.feeder = feeder;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        feeder.PushForward();
        System.out.println("Hello Sarah!");
    }

    @Override
    public void end(boolean interrupted) {
        isDone = true; 
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
