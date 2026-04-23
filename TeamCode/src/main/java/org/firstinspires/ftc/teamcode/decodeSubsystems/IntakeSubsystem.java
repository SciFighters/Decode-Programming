package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSubsystem extends SubsystemBase {
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

    private void startSensorThread() {
        sensorThread = new Thread(() -> {

            boolean currentRight, currentLeft;
            boolean lastRight = false, lastLeft = false;
            while (running) {
                if(intaking){

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

    public boolean rightSwitchState(){
        return rightSwitch.getState();
    }

    public boolean leftSwitchState(){
        return !leftSwitch.getState();
    }

    public void stopSensorThread() {
        running = false;
    }

    public void setPosition(double position) {
        intakeServo1.setPosition(position);
        intakeServo2.setPosition(1 - position);
    }

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }


}