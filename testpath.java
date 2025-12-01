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
        Pose2d startPose = new Pose2d(-40, 61, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);
        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;

        // ---------------- Convert MeepMeep path to RoadRunner ----------------
        Vector2d shootPos = new Vector2d(-34, 34);

        Actions.runBlocking(
            drive.actionBuilder(startPose)

                // Shoot #1
                .strafeToLinearHeading(shootPos, Math.toRadians(45), new TranslationalVelConstraint(60))
                .waitSeconds(2.75)

                // Intake #1
                .splineToLinearHeading(new Pose2d(-30, 12, Math.toRadians(180)), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-48, 12), Math.toRadians(180), new TranslationalVelConstraint(60))

                // Gate
                .strafeToLinearHeading(new Vector2d(-50, 6), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(-52, 6), new TranslationalVelConstraint(65))
                .waitSeconds(0.5)

                // Shoot #2
                .strafeToLinearHeading(shootPos, Math.toRadians(45), new TranslationalVelConstraint(65))
                .waitSeconds(2.75)

                // Intake #2
                .strafeToLinearHeading(new Vector2d(-30, -11.5), Math.toRadians(180), new TranslationalVelConstraint(60))
                .strafeToLinearHeading(new Vector2d(-48, -11.5), Math.toRadians(180))

                // Shoot #3
                .strafeToLinearHeading(shootPos, Math.toRadians(45), new TranslationalVelConstraint(65))
                .waitSeconds(2.75)

                // Intake #3
                .strafeToLinearHeading(new Vector2d(-30, -35), Math.toRadians(180), new TranslationalVelConstraint(60))
                .strafeToLinearHeading(new Vector2d(-48, -35), Math.toRadians(180))

                // Final shoot position
                .strafeToLinearHeading(new Vector2d(-20, 50), Math.toRadians(15), new TranslationalVelConstraint(60))
                .waitSeconds(2.75)

                // Rotate heading for TeleOp setup
                .turn(Math.toRadians(-90))

                .build()
        );
    }
}
