package org.firstinspires.ftc.teamcode.decodeOpModes.testers;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeCommands.CarouselCommands;
import org.firstinspires.ftc.teamcode.decodeCommands.CommandGroups;
import org.firstinspires.ftc.teamcode.decodeCommands.DischargeCommands;
import org.firstinspires.ftc.teamcode.decodeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.SavedValues;
import org.firstinspires.ftc.teamcode.needle.commands.MecanumCommands;

@TeleOp(group = "tests")
public class AutomaticShootingTuner extends ActionOpMode {
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    CarouselSubsystem carouselSubsystem;
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;
    GamepadEx driver, system;
    Button driverA, systemA;
    Button driverB, systemB;
    Button driverY, systemY;
    Button driverX, systemX;
    Button driverDPadDown, systemDPadDown;
    Button driverDPadUp, systemDPadUp;
    Button driverDPadRight, systemDPadRight;
    Button driverDPadLeft, systemDPadLeft;
    Button driverRightBumper, systemRightBumper;
    Button driverLeftBumper, systemLeftBumper;
    Button driverStart, systemStart;
    Button driverBack, systemBack;
    Button driverLeftStick, systemLeftStick, driverRightStick, systemRightStick;
    double wantedRPM = 1000, wantedDegree = 50;
    public static boolean atSpeed;

    @Override
    public void initialize() {

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        carouselSubsystem = new CarouselSubsystem(hardwareMap);
        carouselSubsystem.resetEncoders();

        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(63, 0), Math.PI));

        limelightSubsystem = new LimelightSubsystem(hardwareMap, AutoShooter.TeamColor.RED, mecanumDrive);

        driver = new GamepadEx(gamepad1);
        system = new GamepadEx(gamepad2);
        initButtons();
        mecanumDrive.setDefaultCommand(new MecanumCommands.Drive(mecanumDrive, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX()));
        driverA.whenPressed(new CommandGroups.StartIntake(intakeSubsystem,carouselSubsystem));
        driverB.whenPressed(new IntakeCommands.ClosedState(intakeSubsystem));
        driverY.whenPressed(new IntakeCommands.OutTakeState(intakeSubsystem));
        driverX.whenPressed(new SequentialCommandGroup(new CommandGroups.Shoot(intakeSubsystem,carouselSubsystem),
                new CommandGroups.StartIntake(intakeSubsystem,carouselSubsystem).withTimeout(1000)));
        systemDPadUp.whenPressed(() -> wantedRPM += 100);
        systemDPadDown.whenPressed(() -> wantedRPM -= 100);
        systemA.whenPressed(() -> wantedDegree += 2);
        systemY.whenPressed(() -> wantedDegree -= 2);
        systemX.whenPressed(() -> wantedDegree += 0.2);
        systemB.whenPressed(() -> wantedDegree -= 0.2);

        driverY.whenPressed(new IntakeCommands.OutTakeState(intakeSubsystem));
        driverDPadDown.whenPressed(new DischargeCommands.setState(dischargeSubsystem, 0, 54).withTimeout(10000));
        driverDPadUp.whenPressed(new IntakeCommands.ClosedState(intakeSubsystem));

//        limelightSubsystem.startLimelight();
    }

    @Override
    public void run() {
        aimTurret();
        atSpeed = Math.abs(dischargeSubsystem.getRPM() - wantedRPM) < 200;

        mecanumDrive.localizer.update();
        double launchAngle =
                (AutoShooter.getLaunchAngle(new Pose2d(mecanumDrive.localizer.getPose().position,mecanumDrive.localizer.getPose().heading.toDouble() - Math.PI), AutoShooter.TeamColor.RED) + 360) % 360;
//        double power = -((launchAngle + 360) % 360 - dischargeSubsystem.getTurretAngle());
//        power += Math.signum(power) * 0.03;
//        power = Range.clip(power, -0.4, 0.4);
//        dischargeSubsystem.setTurretPower(power);


        dischargeSubsystem.setFlyWheelRPM(wantedRPM);
        dischargeSubsystem.setRampDegree(wantedDegree);
        super.run();
        multipleTelemetry.addData("turretAngle", dischargeSubsystem.getTurretAngle());

        multipleTelemetry.addData("rampDegree", wantedDegree);
        multipleTelemetry.addData("wantedRPM", wantedRPM );
        multipleTelemetry.addData("rpm", dischargeSubsystem.getRPM());
        multipleTelemetry.addData("power", dischargeSubsystem.getFlyWheelPower());
        multipleTelemetry.addData("x", mecanumDrive.localizer.getPose().position.x);
        multipleTelemetry.addData("y", mecanumDrive.localizer.getPose().position.y);
        multipleTelemetry.addData("distance", AutoShooter.getGoalDistance(mecanumDrive.localizer.getPose(), AutoShooter.TeamColor.RED));
        multipleTelemetry.addData("anglee", launchAngle);
        multipleTelemetry.addData("heading", mecanumDrive.localizer.getPose().heading.toDouble() * 180 / Math.PI);
        multipleTelemetry.update();
    }
    private void aimTurret(){
        com.seattlesolvers.solverslib.geometry.Vector2d mecanumToTurret = new com.seattlesolvers.solverslib.geometry.Vector2d(1.5748, 0).rotateBy(mecanumDrive.localizer.getPose().heading.toDouble() / Math.PI * 180);

        double launchAngle =
                (AutoShooter.getLaunchAngle(new Pose2d(mecanumDrive.localizer.getPose().position.x + mecanumToTurret.getX(),
                        mecanumDrive.localizer.getPose().position.y + mecanumToTurret.getY(),
                        mecanumDrive.localizer.getPose().heading.toDouble() - Math.PI), SavedValues.teamColor) + 360) % 360;
        launchAngle = Range.clip(launchAngle,24,332);

        double power;
//            if (limelightSubsystem.getTx() != 0 && dischargeSubsystem.getTurretAngle() < 355 && dischargeSubsystem.getTurretAngle() > 5){
//                power = limelightSubsystem.getTx() * kp;
//            } else{
        power = -(launchAngle - dischargeSubsystem.getTurretAngle()) * 0.018;
//            }
//        power += mecanumSpeed * 0.12;
        power += Math.signum(power) * 0.04;
        dischargeSubsystem.setTurretPower(power);
    }

    @Override
    public void end() {
        limelightSubsystem.stopLimelight();
    }

    public void initButtons() {
        driverA = new GamepadButton(driver, GamepadKeys.Button.A);
        driverB = new GamepadButton(driver, GamepadKeys.Button.B);
        driverY = new GamepadButton(driver, GamepadKeys.Button.Y);
        driverX = new GamepadButton(driver, GamepadKeys.Button.X);
        driverDPadDown = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);
        driverDPadUp = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        driverDPadRight = new GamepadButton(driver, GamepadKeys.Button.DPAD_RIGHT);
        driverDPadLeft = new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT);
        driverRightBumper = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        driverLeftBumper = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
        driverStart = new GamepadButton(driver, GamepadKeys.Button.START);
        driverLeftStick = new GamepadButton(driver, GamepadKeys.Button.LEFT_STICK_BUTTON);
        driverRightStick = new GamepadButton(driver, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        driverBack = new GamepadButton(driver, GamepadKeys.Button.BACK);
        systemA = new GamepadButton(system, GamepadKeys.Button.A);
        systemB = new GamepadButton(system, GamepadKeys.Button.B);
        systemY = new GamepadButton(system, GamepadKeys.Button.Y);
        systemX = new GamepadButton(system, GamepadKeys.Button.X);
        systemDPadDown = new GamepadButton(system, GamepadKeys.Button.DPAD_DOWN);
        systemDPadUp = new GamepadButton(system, GamepadKeys.Button.DPAD_UP);
        systemDPadRight = new GamepadButton(system, GamepadKeys.Button.DPAD_RIGHT);
        systemDPadLeft = new GamepadButton(system, GamepadKeys.Button.DPAD_LEFT);
        systemRightBumper = new GamepadButton(system, GamepadKeys.Button.RIGHT_BUMPER);
        systemLeftBumper = new GamepadButton(system, GamepadKeys.Button.LEFT_BUMPER);
        systemStart = new GamepadButton(system, GamepadKeys.Button.START);
        systemLeftStick = new GamepadButton(system, GamepadKeys.Button.LEFT_STICK_BUTTON);
        systemRightStick = new GamepadButton(system, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        systemBack = new GamepadButton(system, GamepadKeys.Button.BACK);
    }
}

