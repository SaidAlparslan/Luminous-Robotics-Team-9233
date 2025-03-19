package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hang;

public class HangMode extends Command {
    private final Hang hang;

    public HangMode(Hang hang) {
        this.hang = hang;
        addRequirements(hang); 
    }

    @Override
    public void initialize() {
        hang.toggleHanging();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
