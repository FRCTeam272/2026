package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class FeederStop extends Command {
    
    Feeder feeder;
    boolean isDone=false;
    public FeederStop(Feeder feeder) {
        addRequirements(feeder);
        this.feeder = feeder;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        feeder.stop();
        System.out.println("I really hope this code doesn't explode on me.");
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