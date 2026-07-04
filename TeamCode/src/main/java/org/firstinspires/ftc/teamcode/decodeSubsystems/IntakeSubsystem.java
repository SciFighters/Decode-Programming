package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSubsystem extends SubsystemBase {
    private static final double OPEN_POSITION = 1;
    private static final double CLOSED_POSITION = 0;

    private final DcMotorEx motor;
    private final Servo intakeServo1, intakeServo2;
    public final DigitalChannel leftSwitch, rightSwitch;
    private Thread sensorThread;
    public static boolean reversed = false;
    public static int count = 0;
    public static int switchLoopCount = 0;
    volatile boolean running = true;
    public volatile boolean intaking = false;

    public IntakeSubsystem(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "intakeMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeServo1 = hm.get(Servo.class, "rightIntake");
        intakeServo2 = hm.get(Servo.class, "leftIntake");
        leftSwitch = hm.get(DigitalChannel.class, "leftSwitch");
        rightSwitch = hm.get(DigitalChannel.class, "rightSwitch");
        leftSwitch.setMode(DigitalChannel.Mode.INPUT);
        rightSwitch.setMode(DigitalChannel.Mode.INPUT);
        reversed = false;
        running = true;
//        startSensorThread();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void runIntakeFull() {
        setPower(1);
    }

    public void runReverse() {
        setPower(-0.7);
    }

    public void stopIntake() {
        setPower(0);
    }



    public Command transfer() {
        return new InstantCommand(() -> {
            runIntakeFull();
            close();
        }, this);
    }

    public Command outtake() {
        return new InstantCommand(this::runReverse, this).alongWith(open());
    }

    public Command sortState() {
        return new InstantCommand(() -> {
            stopIntake();
            close();
        }, this);
    }

    public Command closeState() {
        return new InstantCommand(this::stopIntake, this).alongWith(open());
    }

    public Command intake() {
        // Functionally the same as IntakeState,
        // unsure why there were a lot of timers and stuff there
        return new FunctionalCommand(() -> {
            count = 0;
            runIntakeFull();
        }, () -> {
        }, (interrupted) -> stopIntake(), () -> count >= 3)
                .alongWith(open());
    }


    private void startSensorThread() {
        sensorThread = new Thread(() -> {

            boolean currentRight, currentLeft;
            boolean lastRight = false, lastLeft = false;
            while (running) {
                if (intaking) {

                    // check switch state
                    currentRight = rightSwitchState();
                    currentLeft = leftSwitchState();

                    // test to see how many loops run while switch is press
                    if (currentRight || currentLeft)
                        switchLoopCount++;
                    else
                        switchLoopCount = 0;

                    // check if a switch was press
                    if ((currentRight && !lastRight)) {
                        count += 1;
                    }
                    if ((currentLeft && !lastLeft)) {
                        count += 1;
                    }

                    // update last state
                    lastLeft = currentLeft;
                    lastRight = currentRight;

                }

                try {
                    Thread.sleep(8);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        });

        sensorThread.setDaemon(true);
        sensorThread.start();
    }

    public boolean rightSwitchState() {
        return rightSwitch.getState();
    }

    public boolean leftSwitchState() {
        return !leftSwitch.getState();
    }

    public void stopSensorThread() {
        running = false;
    }

    private void setPosition(double position) {
        intakeServo1.setPosition(position);
        intakeServo2.setPosition(1 - position);
    }

    public Command open() {
        return new InstantCommand(() -> setPosition(OPEN_POSITION), this);
    }

    public void close() {
        setPosition(CLOSED_POSITION);
    }

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

}