package org.firstinspires.ftc.teamcode.decodeOpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
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

@TeleOp
public class WheatleyOpMode extends ActionOpMode {
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
    Trigger driverLeftTrigger;
    AutoShooter.TeamColor teamColor;
    ElapsedTime time;
    boolean endGame = false;


    @Override
    public void initialize() {

        SavedValues.currentCount = 0;
        time = new ElapsedTime();
        teamColor = SavedValues.teamColor;

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        carouselSubsystem = new CarouselSubsystem(hardwareMap);

        mecanumDrive = new MecanumDrive(hardwareMap, SavedValues.position);
        mecanumDrive.driveMode();

        limelightSubsystem = new LimelightSubsystem(hardwareMap, teamColor, mecanumDrive);
        register(limelightSubsystem);
        driver = new GamepadEx(gamepad1);
        system = new GamepadEx(gamepad2);
        initButtons();

        mecanumDrive.setDefaultCommand(new MecanumCommands.Drive(mecanumDrive, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX() * 1.25, () -> 1 - 0.5 * gamepad1.right_trigger, teamColor));
        dischargeSubsystem.setDefaultCommand(new DischargeCommands.AutomaticAiming(dischargeSubsystem, limelightSubsystem, mecanumDrive, carouselSubsystem, teamColor));
        driverA.whenPressed(new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem));
        driverB.whenPressed(
                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
                        .whenFinished(() -> CommandScheduler.getInstance().schedule(
                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem)))
        );
        driverX.whenPressed(
                new CommandGroups.Shoot(intakeSubsystem, carouselSubsystem)
                        .whenFinished(() -> CommandScheduler.getInstance().schedule(
                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem)))
        );
        driverDPadLeft.whenPressed(() -> DischargeCommands.AutomaticAiming.aim = !DischargeCommands.AutomaticAiming.aim);
        driverLeftBumper.whenPressed(() -> IntakeCommands.IntakeState.resetCount = !IntakeCommands.IntakeState.resetCount);

        driverY.whenPressed(new CommandGroups.StartOuttake(intakeSubsystem, carouselSubsystem));
        driverLeftBumper.whenPressed(new CommandGroups.SortedShooting(intakeSubsystem, carouselSubsystem)
                .whenFinished(() -> CommandScheduler.getInstance().schedule(
                        new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem)))
        );
        driverRightBumper.whenPressed(new IntakeCommands.ClosedState(intakeSubsystem));
        mecanumDrive.lazyImu.get().resetYaw();
        systemDPadLeft.whenPressed(() -> DischargeCommands.AutomaticAiming.turretCorrection += 2);
        systemDPadRight.whenPressed(() -> DischargeCommands.AutomaticAiming.turretCorrection -= 2);
        systemDPadUp.whenPressed(() -> DischargeCommands.AutomaticAiming.rpmCorrection += 50);
        systemDPadDown.whenPressed(() -> DischargeCommands.AutomaticAiming.rpmCorrection -= 50);
        systemA.whenPressed(() -> SavedValues.currentCount = (SavedValues.currentCount + 1) % 9);
        systemY.whenPressed(() -> SavedValues.currentCount = Math.max((SavedValues.currentCount - 1) % 9, 0));
        systemB.whenPressed(() -> SavedValues.currentCount = 0);
        systemA.whenReleased(() -> gamepad2.rumble(100));
        systemY.whenReleased(() -> gamepad2.rumble(100));
        systemB.whenReleased(() -> gamepad2.rumble(100));
//        systemB.whenPressed(() -> SavedValues.zone = AutoShooter.Zone.BOTH);
        systemLeftBumper.whenPressed(() ->  SavedValues.zone = SavedValues.teamColor == AutoShooter.TeamColor.RED ? AutoShooter.Zone.CLOSE : AutoShooter.Zone.FAR);
        systemRightBumper.whenPressed(() ->  SavedValues.zone = SavedValues.teamColor == AutoShooter.TeamColor.RED ? AutoShooter.Zone.FAR : AutoShooter.Zone.CLOSE);
        systemLeftStick.whenPressed(() -> DischargeCommands.AutomaticAiming.limelight = !DischargeCommands.AutomaticAiming.limelight);
        systemRightStick.whenPressed(new CommandGroups.PowerTakeOff(mecanumDrive, () -> -system.getRightY())
                .beforeStarting(() -> {dischargeSubsystem.getCurrentCommand().cancel();
                    schedule(new  DischargeCommands.setState(dischargeSubsystem,0,60, 180));}));
        driverLeftTrigger.whileActiveOnce(new MecanumCommands.Aim(mecanumDrive, () -> driver.getLeftY(), () -> driver.getLeftX(),() -> 1 - 0.5 * gamepad1.right_trigger,-120 * Math.PI/180, teamColor));
        driverRightStick.whileActiveOnce(new MecanumCommands.Aim(mecanumDrive, () -> driver.getLeftY(), () -> driver.getLeftX(),() -> 1 - 0.5 * gamepad1.right_trigger,Math.PI/2, teamColor));

    }

    @Override
    public void initialize_loop() {
        time.reset();
    }

    @Override
    public void run() {
        mecanumDrive.updatePoseEstimate();

        if (gamepad1.start && gamepad1.x) {
            mecanumDrive.localizer.setPose(new Pose2d(mecanumDrive.localizer.getPose().position, 0));
        }
        if (!DischargeCommands.AutomaticAiming.inRange && AutoShooter.canLaunch(mecanumDrive.localizer.getPose())) {
            gamepad1.rumble(100);
        }
        if (!endGame && time.seconds() > 90) {
            gamepad2.rumble(1000);
            endGame = true;
        }
        if (110.5 > time.seconds() && time.seconds() > 109.5) {
            gamepad1.rumble(100);
        }
        super.run();
        multipleTelemetry.addData("COUNT", SavedValues.currentCount);
        multipleTelemetry.addLine("----------------------------");
        multipleTelemetry.addLine("Position");
        multipleTelemetry.addData("X", mecanumDrive.localizer.getPose().position.x);
        multipleTelemetry.addData("Y", mecanumDrive.localizer.getPose().position.y);
        multipleTelemetry.addData("heading", mecanumDrive.localizer.getPose().heading.toDouble() * 180 / Math.PI);
        multipleTelemetry.addLine("Carousel");
        multipleTelemetry.addData("carouselCount", IntakeSubsystem.count);
        multipleTelemetry.addData("carouselAngle", carouselSubsystem.getAngle());
        multipleTelemetry.addData("carouselPosition", carouselSubsystem.getPosition());
        multipleTelemetry.addLine("Discharge");
        multipleTelemetry.addData("turretAngle", dischargeSubsystem.getTurretAngle());
        multipleTelemetry.addData("turret Ticks", dischargeSubsystem.getTurretPosition());
        multipleTelemetry.addData("rpm", dischargeSubsystem.getRPM());
        multipleTelemetry.addData("flyWheelPower", dischargeSubsystem.flyWheelMotor.motorEx.getPower());
        multipleTelemetry.addData("correction", DischargeCommands.AutomaticAiming.turretCorrection);

        multipleTelemetry.update();
        SavedValues.position = mecanumDrive.localizer.getPose();
    }

    @Override
    public void end() {
        limelightSubsystem.stopLimelight();
        intakeSubsystem.stopSensorThread();
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
        driverLeftTrigger = new Trigger(()-> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
    }
}