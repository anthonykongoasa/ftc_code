package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "testpathRR")
public class testpath extends LinearOpMode {

    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- Setup MecanumDrive ----------------
        Pose2d startPose = new Pose2d(-52, -48, Math.toRadians(55));
        drive = new MecanumDrive(hardwareMap, startPose);
        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;

        // ---------------- Convert MeepMeep path to RoadRunner ----------------
        Vector2d shootPos = new Vector2d(-34, 34);

        Actions.runBlocking(
            drive.actionBuilder(startPose)

                // Shoot #1
                strafeToLinearHeading(shootPos, Math.toRadians(45), new TranslationalVelConstraint(60))
                        //.waitSeconds(2.75) // we'll take this out + replace w/ shooting stuff
                        //shoot();
                        // ----------------------------Intake 1st pile --------
                        //intake.setPower(-1);
                        .strafeToLinearHeading(new Vector2d(-12, -30), Math.toRadians(-90))
                        .strafeToLinearHeading(new Vector2d(-12, -48), Math.toRadians(-90), new TranslationalVelConstraint(60))

                       //----------------gate-------
                       
                        .strafeToLinearHeading(new Vector2d(-6, -50), Math.toRadians(180))
                        .strafeToConstantHeading(new Vector2d(-6, -52), new TranslationalVelConstraint(65))
                        .waitSeconds(0.5)
                        //--------------shoot------------
                        .strafeToLinearHeading(shootPos, Math.toRadians(45), new TranslationalVelConstraint(65))

                        .waitSeconds(2.75)
                        // ----------------------------Intake 2nd pile --------
                        .strafeToLinearHeading(new Vector2d(11.5, -30), Math.toRadians(-90), new TranslationalVelConstraint(60))
                        .strafeToLinearHeading(new Vector2d(11.5, -48), Math.toRadians(-90))



                        .strafeToLinearHeading(shootPos, Math.toRadians(45), new TranslationalVelConstraint(65))
                        .waitSeconds(2.75) // we'll take this out + replace w/ shooting stuff
                        //--------------pile #3

                        .strafeToLinearHeading(new Vector2d(35, -30), Math.toRadians(-90), new TranslationalVelConstraint(60))
                        .strafeToLinearHeading(new Vector2d(35, -48), Math.toRadians(-90))
                        //--------------final shoot position, off line-----------
                        .strafeToLinearHeading(new Vector2d(-50, -20), Math.toRadians(75), new TranslationalVelConstraint(60))
                        .waitSeconds(2.75) // we'll take this out + replace w/ shooting stuff


                .build()
        );
    }
}
