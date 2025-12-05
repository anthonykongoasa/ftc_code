package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Blue9RR")
public class Blue9RR extends LinearOpMode {

    double ON = -1;
    double OFF = 0;
    double IN = 0.15;
    double NEARVEL = 1110; //(tune)

    //PID
    private double NEW_P = 120;
    private double NEW_I = 0;
    private double NEW_D = .8;

    DcMotorEx leftOut;
    DcMotorEx rightOut;
    DcMotor intake;
    DcMotor belt;
    Servo leftIndex;
    Servo rightIndex;
    private MecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        leftOut = (DcMotorEx)hardwareMap.get(DcMotor.class, "leftarm");
        rightOut = (DcMotorEx)hardwareMap.get(DcMotor.class, "rightarm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        belt = hardwareMap.get(DcMotor.class, "uptake");
        leftIndex = hardwareMap.get(Servo.class, "lefthand");
        rightIndex = hardwareMap.get(Servo.class, "righthand");

        //PID
        PIDFCoefficients pidSettings = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, 11.5);
        leftOut.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings);
        rightOut.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings);
        // ---------------- Setup MecanumDrive ----------------
        Pose2d startPose = new Pose2d(-52, -48, Math.toRadians(55));
        drive = new MecanumDrive(hardwareMap, startPose);


        waitForStart();

        if (isStopRequested()) return;
        (leftOut).setVelocity(-NEARVEL);
        rightOut.setVelocity(NEARVEL);

        // ---------------- Convert MeepMeep path to RoadRunner ----------------

        Vector2d shootPos = new Vector2d(-20, -16);

        Actions.runBlocking(
                drive.actionBuilder(startPose)

                        // Shoot #1
                        .strafeToLinearHeading(shootPos, Math.toRadians(55), new TranslationalVelConstraint(60))
                        .waitSeconds(0.5) // we'll take this out + replace w/ shooting stuff


                        //--------------shoot------------
                        //  .strafeToLinearHeading(shootPos, Math.toRadians(45), new TranslationalVelConstraint(65))

                        //.waitSeconds(2.75)
                        // ----------------------------Intake 2nd pile --------
                        //.strafeToLinearHeading(new Vector2d(10.5, -25), Math.toRadians(-90), new TranslationalVelConstraint(60))
                        //.strafeToLinearHeading(new Vector2d(10.5, -48), Math.toRadians(-90))



                        //.strafeToLinearHeading(shootPos, Math.toRadians(45), new TranslationalVelConstraint(65))
                        //.waitSeconds(2.75) // we'll take this out + replace w/ shooting stuff
                        //--------------pile #3

                        //.strafeToLinearHeading(new Vector2d(33, -25), Math.toRadians(-90), new TranslationalVelConstraint(60))
                        //.strafeToLinearHeading(new Vector2d(33, -48), Math.toRadians(-90))
                        //--------------final shoot position, off line-----------
                        //.strafeToLinearHeading(new Vector2d(-50, -20), Math.toRadians(75), new TranslationalVelConstraint(60))
                        //.waitSeconds(2.75) // we'll take this out + replace w/ shooting stuff


                        .build()
        );
        shoot();

        //------------------------------intake POSITION #1----------------
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-20, -16, Math.toRadians(47)))
                        .strafeToLinearHeading(new Vector2d(4, -26), Math.toRadians(-88))
                        .strafeToLinearHeading(new Vector2d(4, -52), Math.toRadians(-88), new TranslationalVelConstraint(60))
                        .waitSeconds(0.2)
                        //-------------GATE----------
                        //.strafeToLinearHeading(new Vector2d(-4, -50), Math.toRadians(180))
                        //.strafeToConstantHeading(new Vector2d(-4, -52), new TranslationalVelConstraint(105))

                        .build()
        );
        intake.setPower(-0.1);
        // ------------shoot #2-------------
         Actions.runBlocking(
            drive.actionBuilder(new Pose2d(4, -48, Math.toRadians(-92)))
                   .strafeToLinearHeading(shootPos, Math.toRadians(52), new TranslationalVelConstraint(60))
                    .build()
          );

             shoot();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-20, -16, Math.toRadians(52)))
                        .strafeToLinearHeading(new Vector2d(33, -28), Math.toRadians(-91))
                        .strafeToLinearHeading(new Vector2d(33, -52), Math.toRadians(-91), new TranslationalVelConstraint(60))
                        .waitSeconds(0.2)


                        .build()
        );
        intake.setPower(-0.1);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(31, -48, Math.toRadians(-91)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(50), new TranslationalVelConstraint(60))
                        .build()
        );

        shoot();






    }
    private void shoot() {

        intake.setPower(ON);

        leftIndex.setPosition(0);
        rightIndex.setPosition(1);
        belt.setPower(-0.8);
        delay(2.25); // wait 3 seconds for shooting

        // Stop all


        leftIndex.setPosition(0.8);
        rightIndex.setPosition(0.2);
        belt.setPower(-0.3);
        delay(0.5);
    }

    // -------- Delay helper ----------
    private void delay(double t) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            // Optionally update telemetry

        }
    }
}
