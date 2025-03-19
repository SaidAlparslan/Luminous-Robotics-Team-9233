package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Hang;

public class HangCommand extends Command {
    private final Hang hang;
    private final boolean yonduzmu;
    private double speed;
    private final CommandXboxController joystick = new CommandXboxController(0);

    public HangCommand(Hang hang,boolean tersmi,double speed) {
        this.hang = hang;
        this.yonduzmu = tersmi;
        this.speed=speed;
        addRequirements(hang);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute(){
        if (yonduzmu){
            hang.setSpeed1(speed);
        }
        else{
            hang.setSpeed1(-(joystick.getRawAxis(3)+1)*2/5 );
        }  
    }
    @Override
    public void end(boolean interrupted){
        hang.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}