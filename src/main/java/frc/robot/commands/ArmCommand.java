package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;

public class ArmCommand extends Command {
    private final Arm arm;
    private final ArmState m_state;

    public ArmCommand(Arm arm, ArmState state) {
        this.arm = arm;
        this.m_state = state;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        System.out.println("Arm: " + m_state);
        arm.setTargetState(m_state);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Arm Current Pos", arm.getCurrentPosition());
        SmartDashboard.putNumber("Arm Target Pos", m_state.getPosition());
        
    }

     @Override
    public boolean isFinished() {
        if (Robot.isSimulation()) {
        return true;
    }
    return Math.abs(arm.getCurrentPosition() - m_state.getPosition()) < 3;
    } 


    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
