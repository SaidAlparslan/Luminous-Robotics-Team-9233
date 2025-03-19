package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveDrive extends Command {
private final CommandSwerveDrivetrain drivetrain;
public boolean duzmu;
public double maxspeed;
    public SwerveDrive(CommandSwerveDrivetrain drivetrain, boolean duzmu,double maxspeed) {
        this.drivetrain = drivetrain;
        this.duzmu=duzmu;
        this.maxspeed = maxspeed;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        if (duzmu){
            drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(maxspeed/12));
        }
        else{
            drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-maxspeed/12));
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
