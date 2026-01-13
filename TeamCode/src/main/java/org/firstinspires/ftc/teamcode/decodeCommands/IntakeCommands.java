package org.firstinspires.ftc.teamcode.decodeCommands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;

public class IntakeCommands {


    public static class IntakeState extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        ElapsedTime time;
        double lastTime, currentTime, lastIn;
        int i = 0;
        int count;
        double leftDistance = 0, rightDistance = 0;
        double fourthStart = 0, fullStart = 0;
        public static boolean resetCount;

        public IntakeState(IntakeSubsystem intakeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            time = new ElapsedTime();
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            time.reset();
            count = 0;
            resetCount = false;
            currentTime = time.seconds();
            intakeSubsystem.setPower(1);
            intakeSubsystem.setPosition(1);
        }

        @Override
        public void execute() {
            i++;
            if(count < 3){
                intakeSubsystem.setPower(1);
                intakeSubsystem.setPosition(1);
                fourthStart = time.seconds();
                fullStart = fourthStart;
            } else if (count == 3) {
                fourthStart = time.seconds();
                if(fourthStart - fullStart > -1){
                    intakeSubsystem.setPower(0.5);
                    intakeSubsystem.setPosition(0.5);
                }
            }else {
                intakeSubsystem.setPower(-0.5);
                intakeSubsystem.setPosition(1);
                if(time.seconds() - fourthStart > 0.55){
                    count = 3;
                    intakeSubsystem.setPower(0.5);
                }
            }
            lastTime = currentTime;
            currentTime = time.seconds();
            if (true) {
                double lastLeft = leftDistance;
                double lastRight = rightDistance;
                leftDistance = intakeSubsystem.getLeftDistance();
                rightDistance = intakeSubsystem.getRightDistance();
                if (leftDistance < 5 && lastLeft > 5) {
                    count += 1;
                }
                if (rightDistance < 5 && lastRight > 5) {
                    count += 1;
                }

            }
            if(resetCount){
                count = 0;
            }
        }
//
//        @Override
//        public boolean isFinished() {
//            return true;
//        }
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

    public static class SemiTransferState extends CommandBase {
        IntakeSubsystem intakeSubsystem;

        public SemiTransferState(IntakeSubsystem intakeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setPower(1);
            intakeSubsystem.setPosition(0.21);
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

    public static class PreSortingState extends CommandBase {
        IntakeSubsystem intakeSubsystem;

        public PreSortingState(IntakeSubsystem intakeSubsystem) {
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

    public static class SortingState extends CommandBase {
        IntakeSubsystem intakeSubsystem;

        public SortingState(IntakeSubsystem intakeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.setPower(1);
            intakeSubsystem.setPosition(0.6);
        }

        @Override
        public void execute() {
            super.execute();
            if (intakeSubsystem.getCurrent() > 4.5) {
                intakeSubsystem.setPower(-1);
            } else {
                intakeSubsystem.setPower(1);
            }
        }

    }

}
