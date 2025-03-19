package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class KomutIptal extends Command {

    public KomutIptal() {
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
