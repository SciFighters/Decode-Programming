package org.firstinspires.ftc.teamcode.needle.stateMachine;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.TelescopicArmSubsystem;

public class StateMachine {
    GamepadEx driver, system;
    SubsystemBase telescopicArmSubsystem, armAxisSubsystem, clawSubsystem, mecanumDrive;
    States currentState = States.NONE;
    RobotState state1, state2, state3;

    public StateMachine(GamepadEx driver, GamepadEx system, MecanumDrive mecanumDrive, TelescopicArmSubsystem telescopicArmSubsystem, ArmAxisSubsystem armAxisSubsystem, ClawSubsystem clawSubsystem) {
        this.driver = driver;
        this.system = system;
        this.mecanumDrive = mecanumDrive;
        this.telescopicArmSubsystem = telescopicArmSubsystem;
        this.armAxisSubsystem = armAxisSubsystem;
        this.clawSubsystem = clawSubsystem;
    }
    public void updateStates(States newState){
        if(currentState != newState){
            CommandScheduler.getInstance().clearButtons();

            CommandScheduler.getInstance().getDefaultCommand(mecanumDrive).cancel();
            CommandScheduler.getInstance().getDefaultCommand(clawSubsystem).cancel();
            CommandScheduler.getInstance().getDefaultCommand(armAxisSubsystem).cancel();
            CommandScheduler.getInstance().getDefaultCommand(telescopicArmSubsystem).cancel();
            //not using actual states here
            switch (newState){
                case NONE:
                    state1.setState();
                    break;
                case INTAKE:
                    state2.setState();
                    break;
                case DISCHARGE:
                    state3.setState();
                    break;
            }
            currentState = newState;
        }
    }

}
