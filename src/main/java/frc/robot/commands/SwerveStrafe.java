package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveStrafe extends Command {
private final CommandSwerveDrivetrain drivetrain;
public boolean sagmi;
public double maxspeed;
    public SwerveStrafe(CommandSwerveDrivetrain drivetrain, boolean sagmi,double maxspeed) {
        this.drivetrain = drivetrain;
        this.sagmi=sagmi;
        this.maxspeed = maxspeed;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        if (sagmi){
            drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(-maxspeed/15));
        }
        else{
            drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(maxspeed/15));
        }
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
