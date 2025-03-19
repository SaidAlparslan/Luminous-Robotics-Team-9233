// Copyright Luminous Robotics Team 9233

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmAndElevatorCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutonIntakeCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.ElevatorTrapezoidCommand;
import frc.robot.commands.HangCommand;
import frc.robot.commands.HangMode;
import frc.robot.commands.IntakeManuelCommand;
import frc.robot.commands.IntakePositionCommand;
import frc.robot.commands.KomutIptal;
import frc.robot.commands.ReefCoralOuttakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorTrapezoidProfile;
import frc.robot.subsystems.ElevatorTrapezoidProfile.ElevatorTrapezoidState;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.commands.StartIntake;
import frc.robot.commands.StopIntake;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.SwerveStrafe;
import frc.robot.subsystems.Arm;

public class RobotContainer {
    private final ElevatorTrapezoidProfile elevatortrapezoid;
    private final Intake intake;
    private final Arm arm;
    private final Hang hang;
    
    private final LED m_led = new LED(2, 3, 4);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
 
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0); 
    private final CommandXboxController controlpanel = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        elevatortrapezoid = new ElevatorTrapezoidProfile();
        arm = new Arm();
        intake = new Intake(m_led,arm);
        hang = new Hang();
        
        NamedCommands.registerCommand("Intake", new AutonIntakeCommand(intake));
        NamedCommands.registerCommand("StartIntake", new StartIntake(intake));
        NamedCommands.registerCommand("StopIntake", new StopIntake(intake));
        NamedCommands.registerCommand("L4Shoot",new ReefCoralOuttakeCommand(arm, ArmState.L4ARM, elevatortrapezoid, ElevatorTrapezoidState.L4,intake,m_led) );
        NamedCommands.registerCommand("L2Shoot",new ReefCoralOuttakeCommand(arm, ArmState.L1_L2_L3ARM, elevatortrapezoid, ElevatorTrapezoidState.L2,intake,m_led) );
        NamedCommands.registerCommand("L4POS",new ArmAndElevatorCommand(arm, ArmState.L4ARM, elevatortrapezoid, ElevatorTrapezoidState.L4));
        NamedCommands.registerCommand("intakepos",new IntakePositionCommand(arm, elevatortrapezoid , ElevatorTrapezoidState.NOTRL1));

        new EventTrigger("L1").onTrue(new ArmAndElevatorCommand(arm, ArmState.L4ARM , elevatortrapezoid , ElevatorTrapezoidState.NOTRL1));
        new EventTrigger("L2").onTrue(new ArmAndElevatorCommand(arm, ArmState.L4ARM , elevatortrapezoid , ElevatorTrapezoidState.L2));
        new EventTrigger("L3").onTrue(new ArmAndElevatorCommand(arm, ArmState.L4ARM , elevatortrapezoid , ElevatorTrapezoidState.L3));
        new EventTrigger("L4").onTrue(new ArmAndElevatorCommand(arm, ArmState.L4ARM, elevatortrapezoid, ElevatorTrapezoidState.L4));
        new EventTrigger("intake").onTrue(new IntakePositionCommand(arm, elevatortrapezoid, ElevatorTrapezoidState.NOTRL1));
        new EventTrigger("intakearm").onTrue(new ArmCommand(arm, ArmState.NOTRINTAKEARM));
        new EventTrigger("l1l2l3arm").onTrue(new ArmCommand(arm, ArmState.L4ARM));
        new EventTrigger("l4arm").onTrue(new ArmCommand(arm, ArmState.L4ARM));
        
        configureBindings();
        
        autoChooser = AutoBuilder.buildAutoChooser("orta+robotsagi-ist1");
        SmartDashboard.putData("Auto Mode", autoChooser);
        }
    
        private void configureBindings() {
        drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getRawAxis(1)* MaxSpeed*2/3*elevatortrapezoid.swervehizikatsayisi()*hang.hanghizkatsayisi()) 
                    .withVelocityY(-joystick.getRawAxis(0) * MaxSpeed*2/3*elevatortrapezoid.swervehizikatsayisi()*hang.hanghizkatsayisi()) 
                    .withRotationalRate(-joystick.getRawAxis(2) * MaxAngularRate*4/5*elevatortrapezoid.swervehizikatsayisi()*hang.hanghizkatsayisi())
            )
        );

        //INTAKE
        joystick.button(1).toggleOnTrue(new CoralIntakeCommand(intake));
        joystick.button(6).whileTrue(new IntakeManuelCommand(intake, 0.7,true));
        joystick.button(8).whileTrue(new IntakeManuelCommand(intake, 0.7,false));

        //HANG
        joystick.button(5).whileTrue(new HangCommand(hang,true,0.2));
        joystick.button(7).whileTrue(new HangCommand(hang, false, 0.2));

        joystick.button(9).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.button(10).toggleOnTrue(new HangMode(hang));
        joystick.button(14).onTrue(new KomutIptal());

        joystick.povLeft().whileTrue(new SwerveStrafe(drivetrain, false, MaxSpeed));
        joystick.povRight().whileTrue(new SwerveStrafe(drivetrain, true, MaxSpeed));
        joystick.povDown().whileTrue(new SwerveDrive(drivetrain, false, MaxSpeed));
        joystick.povUp().whileTrue(new SwerveDrive(drivetrain, true, MaxSpeed));

        controlpanel.button(4).onTrue(new ElevatorTrapezoidCommand(elevatortrapezoid, ElevatorTrapezoidState.L2,arm));
        controlpanel.button(2).onTrue(new ElevatorTrapezoidCommand(elevatortrapezoid, ElevatorTrapezoidState.L3,arm));
        controlpanel.button(1).onTrue(new ElevatorTrapezoidCommand(elevatortrapezoid, ElevatorTrapezoidState.L4,arm));
        controlpanel.button(3).onTrue(new ElevatorTrapezoidCommand(elevatortrapezoid, ElevatorTrapezoidState.NOTRL1,arm));

        controlpanel.button(6).onTrue(new IntakePositionCommand(arm, elevatortrapezoid, ElevatorTrapezoidState.NOTRL1));
        controlpanel.button(5).onTrue(new ArmCommand(arm, ArmState.PROCESSORARM));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
}
