package org.firstinspires.ftc.teamcode.decodeCommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;

public class IntakeCommands {

    public static class IntakeState extends CommandBase {
        //        variables(eg subsystems, targetPos etc)
        IntakeSubsystem intakeSubsystem;



        //            addRequirements(subsystem)
        public IntakeState(IntakeSubsystem intakeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
        intakeSubsystem.setPower(1);
        intakeSubsystem.setPosition(0.7);
        }

        @Override
        public void execute() {
            //every frame
        }

        @Override
        public boolean isFinished() {
            //returns if it is finished, false by default(runs forever), return true means instant finish
            return true;
        }
    }
     public static class TransferState extends CommandBase {
        //        variables(eg subsystems, targetPos etc)
        IntakeSubsystem intakeSubsystem;



        //            addRequirements(subsystem)
        public TransferState(IntakeSubsystem intakeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setPower(1);
            intakeSubsystem.setPosition(0.3);
        }

        @Override
        public void execute() {
            //every frame
        }

        @Override
        public boolean isFinished() {
            //returns if it is finished, false by default(runs forever), return true means instant finish
            return true;
        }
    }
    public static class ClosedState extends CommandBase {
        //        variables(eg subsystems, targetPos etc)
        IntakeSubsystem intakeSubsystem;



        //            addRequirements(subsystem)
        public ClosedState(IntakeSubsystem intakeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setPower(0);
            intakeSubsystem.setPosition(0.7);
        }

        @Override
        public void execute() {
            //every frame
        }

        @Override
        public boolean isFinished() {
            //returns if it is finished, false by default(runs forever), return true means instant finish
            return true;
        }
    }

}
