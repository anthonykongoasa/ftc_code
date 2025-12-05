package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="FastPIDFTuner")
public class FastPIDFTuner extends LinearOpMode {

    private DcMotorEx leftMotor, rightMotor;

    // Starting PIDF values
    private double P = 120;
    private double I = 0;
    private double D = 0.8;
    private double F = 11.5; // adjust this live

    // Set your desired shooting speed here (RPM)
    private double targetVelocityRPM = 800; // example: 4000 RPM

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftarm");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightarm");

        // Run using encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply initial PIDF
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        telemetry.addLine("D-pad up/down: increase/decrease F by 0.1");
        telemetry.addLine("Press START to run motors at target velocity");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            // Update F coefficient live
            if(gamepad1.dpad_up) F += 0.1;
            if(gamepad1.dpad_down) F -= 0.1;

          //vel
            if(gamepad1.dpad_right) targetVelocityRPM += 10;   
            if(gamepad1.dpad_left) targetVelocityRPM -= 10; 

            // Clamp to reasonable range
            targetVelocityRPM = Math.max(0, Math.min(targetVelocityRPM, 6000));

            // Set motors to current target velocity
            

            PIDFCoefficients newPIDF = new PIDFCoefficients(P, I, D, F);
            leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
            rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);

            // Set both motors to fixed target velocity
            leftMotor.setVelocity(-targetVelocityRPM);
            rightMotor.setVelocity(targetVelocityRPM);

            // Telemetry
            telemetry.addData("Target Vel (RPM)", targetVelocityRPM);
            telemetry.addData("Left Motor Vel", leftMotor.getVelocity());
            telemetry.addData("Right Motor Vel", rightMotor.getVelocity());
            telemetry.addData("F Coefficient", F);
            telemetry.update();

            sleep(50);
        }
    }
}
