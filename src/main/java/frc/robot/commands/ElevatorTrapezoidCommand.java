package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ElevatorTrapezoidProfile;
import frc.robot.subsystems.ElevatorTrapezoidProfile.ElevatorTrapezoidState;
import frc.robot.subsystems.Arm.ArmState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class ElevatorTrapezoidCommand extends Command {
    private final ElevatorTrapezoidProfile m_elevator;
    private final ElevatorTrapezoidState m_state;
    private final Arm arm;
    private boolean isCoral =true;



    public ElevatorTrapezoidCommand(ElevatorTrapezoidProfile elevator, ElevatorTrapezoidState state,Arm arm) {
        this.m_elevator = elevator;
        this.m_state = state;
        this.arm =arm;
        
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        isCoral =true;
        if (arm.getTargetState()==Arm.ArmState.PROCESSORARM){
            isCoral=false;
        }

        if (arm.getTargetState() == Arm.ArmState.NOTRINTAKEARM && 
        m_elevator.getCurrentState() == ElevatorTrapezoidState.NOTRL1&&m_state!=ElevatorTrapezoidState.L4) {
        new ArmCommand(arm, ArmState.L1_L2_L3ARM).schedule();
        cancel();
        return;
        }
        if (arm.getTargetState() == Arm.ArmState.NOTRINTAKEARM && 
        m_elevator.getCurrentState() == ElevatorTrapezoidState.NOTRL1&&m_state==ElevatorTrapezoidState.L4) {
        new ArmCommand(arm, ArmState.L4ARM).schedule();
        cancel();
        return;
        }
        if (arm.getTargetState() == Arm.ArmState.NOTRINTAKEARM && 
        m_state == ElevatorTrapezoidState.NOTRL1) {
        System.out.println("Elevator hareket edemez! Arm NOTR pozisyonunda.");
        cancel();
        return;
        }

        if(arm.getTargetState()==Arm.ArmState.PROCESSORARM&&m_state==ElevatorTrapezoidState.L4){
            m_elevator.setState(ElevatorTrapezoidState.NET);
        }
        else{
        m_elevator.setState(m_state);
        System.out.println("ElevatorTrapezoid: " + m_state);}
    }
    @Override
    public void end(boolean interrupted){  
        System.out.println("Elevator Duruyor");
        if(m_state==ElevatorTrapezoidState.NOTRL1&&!SmartDashboard.getBoolean("Coral LS", false)&&!DriverStation.isAutonomous()&&isCoral){
            new ArmCommand(arm, ArmState.NOTRINTAKEARM).schedule();
        }
        if(m_state==ElevatorTrapezoidState.L4&&arm.getTargetState()!=Arm.ArmState.L4ARM&&SmartDashboard.getBoolean("Coral LS", false)&&isCoral){
            new ArmCommand(arm, ArmState.L4ARM).schedule();
            System.out.println("Pozisyon: L4");
        }
        if(m_state==ElevatorTrapezoidState.L4&&arm.getTargetState()==Arm.ArmState.PROCESSORARM){
            System.out.println("Pozisyon: Net");
            new ArmCommand(arm,ArmState.NETARM).schedule();
        }
    }
    @Override
public void execute() {
    SmartDashboard.putNumber("Elevator Current Pos", m_elevator.getCurrentPosition());
    SmartDashboard.putNumber("Elevator Target Pos", m_state.getTargetHeight());
}

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()){
            return 
            Math.abs(m_elevator.getCurrentPosition() - m_state.getTargetHeight()) < 4;}
            if(m_state==ElevatorTrapezoidState.L4&&!DriverStation.isAutonomous()){
                return Math.abs(m_elevator.getCurrentPosition() - m_state.getTargetHeight()) < 4;
            }
        return Math.abs(m_elevator.getCurrentPosition() - m_state.getTargetHeight()) < 1;
}}
