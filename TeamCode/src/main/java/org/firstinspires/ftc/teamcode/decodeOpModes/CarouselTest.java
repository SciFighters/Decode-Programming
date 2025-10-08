package org.firstinspires.ftc.teamcode.decodeOpModes;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.CarouselCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;

@Autonomous(name = "CarouselTest")
public class CarouselTest extends ActionOpMode {

    @Override
    public void initialize() {
        CarouselSubsystem carouselSubsystem = new CarouselSubsystem(hardwareMap);

        // Create commands
        CarouselCommands.MoveToPos moveToPos = new CarouselCommands.MoveToPos(carouselSubsystem, 1000);
        CarouselCommands.ThirdOfSpin thirdOfSpin = new CarouselCommands.ThirdOfSpin(carouselSubsystem);
        CarouselCommands.SortByMotif sortByMotif = new CarouselCommands.SortByMotif(Motif.GPP, 1, carouselSubsystem);
        CarouselCommands.Discharge discharge = new CarouselCommands.Discharge(carouselSubsystem);

        // Combine them into a sequential command group
        SequentialCommandGroup fullSequence = new SequentialCommandGroup(
                moveToPos,
                thirdOfSpin,
                sortByMotif,
                discharge
        );

        // Schedule once; ActionOpMode automatically runs the scheduler
        CommandScheduler.getInstance().schedule(fullSequence);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        // Keep the scheduler running while opmode is active
        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
        }

        CommandScheduler.getInstance().cancelAll();
    }
}
