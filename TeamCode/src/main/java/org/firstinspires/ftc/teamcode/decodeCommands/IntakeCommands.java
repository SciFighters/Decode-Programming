package org.firstinspires.ftc.teamcode.decodeCommands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;


public class IntakeCommands {


    public static class IntakeState extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        ElapsedTime time;
        double currentTime;


        public static boolean resetCount;

        public IntakeState(IntakeSubsystem intakeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            time = new ElapsedTime();
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            time.reset();
            IntakeSubsystem.count = 0;
            resetCount = false;
            currentTime = time.seconds();
            intakeSubsystem.setPower(1);
            intakeSubsystem.setPosition(1);
        }


        @Override
        public boolean isFinished() {
            return IntakeSubsystem.count >= 60;
        }

        @Override
        public void end(boolean interrupted) {
            intakeSubsystem.setPower(0);
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

    public static class OutTakeState extends CommandBase {
        IntakeSubsystem intakeSubsystem;


        public OutTakeState(IntakeSubsystem intakeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setPower(-0.7);
            intakeSubsystem.setPosition(1);
        }

        @Override
        public boolean isFinished() {
            return true;
        }

    }


    public static class SortingState extends CommandBase {
        IntakeSubsystem intakeSubsystem;

        public SortingState(IntakeSubsystem intakeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setPower(0);
            intakeSubsystem.setPosition(0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }

    }

}
