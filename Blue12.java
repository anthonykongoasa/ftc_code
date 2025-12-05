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

@Autonomous(name = "Blue12RR")
public class Blue12RR extends LinearOpMode {

    double ON = -1;
    double OFF = 0;
    double IN = 0.15;
    double NEARVEL = 1125; //(tune)
    double FARVEL = 1500;

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

        // Shoot #1 (approach)
        .strafeToLinearHeading(shootPos, Math.toRadians(55), new TranslationalVelConstraint(75)) // slow for accurate shot
        .build()
);
shoot();
        Actions.runBlocking(
        drive.actionBuilder(new Pose2d(-20, -16, Math.toRadians(47)))
            
        .strafeToLinearHeading(new Vector2d(4, -26), Math.toRadians(-88), new TranslationalVelConstraint(60)) // sharp
        .strafeToLinearHeading(new Vector2d(4, -52), Math.toRadians(-88), new TranslationalVelConstraint(70)) // fast straight
        // Sharp gate turn
        .strafeToLinearHeading(new Vector2d(8, -48), Math.toRadians(0), new TranslationalVelConstraint(55)) // slow for precision
        .strafeToConstantHeading(new Vector2d(8, -52), new TranslationalVelConstraint(80)) // final adjustment (open) ram
        .waitSeconds(0.1)
        .build()
);
intake.setPower(0.3);

// ------------shoot #2-------------
Actions.runBlocking(
    drive.actionBuilder(new Pose2d(8, -52, Math.toRadians(180)))
           .strafeToLinearHeading(shootPos, Math.toRadians(52), new TranslationalVelConstraint(65)) // approach slow
           .build()
);
shoot();

// Intake #2
Actions.runBlocking(
    drive.actionBuilder(new Pose2d(-20, -16, Math.toRadians(52)))
        .strafeToLinearHeading(new Vector2d(33, -25), Math.toRadians(-91), new TranslationalVelConstraint(65)) // strafe slightly slower
        .strafeToLinearHeading(new Vector2d(33, -48), Math.toRadians(-91), new TranslationalVelConstraint(75)) // straight back, faster
        .build()
);
intake.setPower(0.3);

// Shoot #3
Actions.runBlocking(
    drive.actionBuilder(new Pose2d(33, -48, Math.toRadians(-91)))
        .strafeToLinearHeading(shootPos, Math.toRadians(50), new TranslationalVelConstraint(65)) // precise shot
        .build()
);
shoot();

// Intake #3
Actions.runBlocking(
    drive.actionBuilder(new Pose2d(20, -16, Math.toRadians(50)))
        .strafeToLinearHeading(new Vector2d(62, -25), Math.toRadians(-90), new TranslationalVelConstraint(60)) // strafe to intake
        .strafeToLinearHeading(new Vector2d(62, -48), Math.toRadians(-90), new TranslationalVelConstraint(70)) // straight back, faster
        .build()
);

// Far shot zone
leftOut.setVelocity(-FARVEL);
rightOut.setVelocity(FARVEL);
        
intake.setPower(0.3);
Actions.runBlocking(
    drive.actionBuilder(new Pose2d(62, -48, Math.toRadians(-90)))
        .strafeToLinearHeading(new Vector2d(80, -10), Math.toRadians(35), new TranslationalVelConstraint(65)) // sharp turn 
        .build()
);
shoot();

// Off-line final // next to gate hoepfully
Actions.runBlocking(
    drive.actionBuilder(new Pose2d(80, -10, Math.toRadians(35)))
        .strafeToLinearHeading(new Vector2d(17, -25), Math.toRadians(180), new TranslationalVelConstraint(70)) // straight, fast
);
        
        
        
        // off line (not important)






    }
    private void shoot() {

        intake.setPower(ON);

        leftIndex.setPosition(0);
        rightIndex.setPosition(1);
        belt.setPower(-0.8);
        delay(1); // wait 1 seconds for shooting

        // Stop all

        //spinning against
        leftIndex.setPosition(0.8);
        rightIndex.setPosition(0.2);
       // belt.setPower(-0.3);
       
    }

    // -------- Delay helper ----------
    private void delay(double t) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            // Optionally update telemetry

        }
    }
}
