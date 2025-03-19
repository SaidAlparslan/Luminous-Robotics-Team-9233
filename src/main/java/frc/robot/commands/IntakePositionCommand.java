package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ElevatorTrapezoidProfile;
import frc.robot.subsystems.ElevatorTrapezoidProfile.ElevatorTrapezoidState;
import frc.robot.subsystems.Arm.ArmState;
public class IntakePositionCommand extends SequentialCommandGroup {
    public IntakePositionCommand(Arm arm,ElevatorTrapezoidProfile elevator, ElevatorTrapezoidState elevator_state) {
        addCommands(  
        new ElevatorTrapezoidCommand(elevator, elevator_state,arm),
        new ArmCommand(arm, ArmState.NOTRINTAKEARM)
        );
    }
}
