package org.firstinspires.ftc.teamcode.decodeCommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;

public class IntakeCommands {

    public static class IntakeState extends CommandBase {
        IntakeSubsystem intakeSubsystem;

        public IntakeState(IntakeSubsystem intakeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setPower(1);
            intakeSubsystem.setPosition(1);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class TransferState extends CommandBase {
        IntakeSubsystem intakeSubsystem;

        public TransferState(IntakeSubsystem intakeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setPower(1);
            intakeSubsystem.setPosition(0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class ClosedState extends CommandBase {
        IntakeSubsystem intakeSubsystem;

        public ClosedState(IntakeSubsystem intakeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setPower(0);
            intakeSubsystem.setPosition(1);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

}
