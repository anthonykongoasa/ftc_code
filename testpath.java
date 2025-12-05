package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "testpathRR")
public class testpath extends LinearOpMode {
    
    double ON = -1;
    double OFF = 0;
    double IN = 0.15;
    double NEARVEL = 1000; //(tune)

    //PID
    private double NEW_P = 120;
    private double NEW_I = 0;
    private double NEW_D = .8;

    DcMotorEx leftOut;
    DcMotorEx rightOut;
    DcMotor intake;
    DcMotor belt;
    CRServo leftIndex;
    CRServo rightIndex;
    private MecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        leftOut = (DcMotorEx)hardwareMap.get(DcMotor.class, "leftarm");
        rightOut = (DcMotorEx)hardwareMap.get(DcMotor.class, "rightarm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        belt = hardwareMap.get(DcMotor.class, "uptake");
        leftIndex = (CRServo)hardwareMap.get(Servo.class, "leftUptake");
        rightIndex = (CRServo)hardwareMap.get(Servo.class, "rightUptake");

        //PID
        PIDFCoefficients pidSettings = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, 5.46116667);
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
                        .strafeToLinearHeading(shootPos, Math.toRadians(47), new TranslationalVelConstraint(60))
                        .waitSeconds(1) // we'll take this out + replace w/ shooting stuff

                       
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
             drive.actionBuilder(drive.getPoseEstimate())
                     .strafeToLinearHeading(new Vector2d(-9, -20), Math.toRadians(-90))
                     .strafeToLinearHeading(new Vector2d(-9, -48), Math.toRadians(-90), new TranslationalVelConstraint(60))
                     .waitSeconds(0.2)
                     //-------------GATE----------
                     //.strafeToLinearHeading(new Vector2d(-4, -50), Math.toRadians(180))
                     //.strafeToConstantHeading(new Vector2d(-4, -52), new TranslationalVelConstraint(105))
                     .waitSeconds(0.5)
                    .build()
             );
        // ------------shoot #2-------------
        // Actions.runBlocking(
         //    drive.actionBuilder(drive.getPoseEstimate())
          //           .strafeToLinearHeading(shootPos, Math.toRadians(47), new TranslationalVelConstraint(60))
           //  );
        
   //     shoot();
        
            

        
        
        
    }
    private void shoot() {

        intake.setPower(-ON);

        leftIndex.setPower(ON);
        rightIndex.setPower(-ON);
        belt.setPower(-0.5);
        delay(2.0); // wait 3 seconds for shooting

        // Stop all

        
        leftIndex.setPower(OFF);
        rightIndex.setPower(OFF);
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
