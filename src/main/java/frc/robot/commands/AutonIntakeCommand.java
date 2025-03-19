package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class AutonIntakeCommand extends Command {
    private final Intake intake;

    public AutonIntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.startCoralIntake();
        System.out.println("Coral Bekleniyor");
    }

    @Override
    public void end(boolean interrupted){
        intake.stopIntake();
        System.out.println("Coral Algilandi");
    }

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()){
            return true;}
        return intake.isCoralDetected();
    }
}