package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MultitaskingWithThreads extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor;

    private volatile boolean isRunning = true; // Shared flag for stopping threads

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        // Wait for the game to start
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Create threads for multitasking
        Thread driveThread = new Thread(new DriveControl());
        Thread armThread = new Thread(new ArmControl());

        // Start threads
        driveThread.start();
        armThread.start();

        // Wait for the game to stop
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

        // Stop threads when OpMode ends
        isRunning = false;

        // Wait for threads to finish
        driveThread.join();
        armThread.join();
    }

    // Runnable class for drivetrain control
    private class DriveControl implements Runnable {

        @Override

        public void run() {

            while (isRunning) {

                double drivePower = -gamepad1.left_stick_y;
                double turnPower = gamepad1.right_stick_x;

                // Set drivetrain power
                leftMotor.setPower(drivePower + turnPower);
                rightMotor.setPower(drivePower - turnPower);

                try {
                    Thread.sleep(20); // Sleep to reduce CPU usage
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }

            // Stop motors when thread stops
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

    }

    // Runnable class for arm control
    private class ArmControl implements Runnable {

        @Override

        public void run() {

            while (isRunning) {
                double armPower = gamepad2.left_stick_y;
                // Set arm motor power
                armMotor.setPower(armPower);

                try {
                    Thread.sleep(20); // Sleep to reduce CPU usage
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }

            // Stop arm motor when thread stops
            armMotor.setPower(0);
        }
    }
}
