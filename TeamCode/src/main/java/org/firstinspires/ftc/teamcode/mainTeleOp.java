/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.VoltageSensor;





/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="mainTeleOp", group="Linear OpMode")

public class mainTeleOp extends LinearOpMode {

    // Declare OpMode members.

    private FtcDashboard dashboard;
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo intakeCRSLeft = null;
    private CRServo intakeCRSRight = null;
    private Servo intakeServoLeft = null;
    private Servo lockServo = null;
    private DcMotor intakeDrive = null;

    private Servo clawServo = null;
    private Servo wristServo = null;
    private Servo armServo = null;


    private DcMotor outmoto1 = null;
    private DcMotor outmoto2 = null;
    private static final int MIN_POSITION = 0;
    private static final int MAX_POSITION = 880;
    int intakeTargetPosition = 0;

    public double ServoPosition = 0.5;

    //wrist positions
    public double wristPositionDown = 0;
    public double wristPositionStraight = 0.62;
    public double wristPositionOut = 1;

    //arm positions
    public double armPositionDeposit = 0.425;
    public double armPositionHover = 0.815;
    public double armPositionGrab = 0.95;

    //claw positions
    public double clawPositionOpen = 0.26;
    public double clawPositionClosed = 0.48;

    private boolean previousDpadDownState = false;
    private boolean previousDpadUpState = false;
    private boolean PreviousDpadLeftState = false;
    private boolean previousAState = false;

    private boolean previousIntakeState = false;
    private Servo intakeServoRight = null;

    boolean sampleDistanceTriggered = false;

    int state = 0; // Persistent state variable
    long startTime = 0; // Persistent timer variable

    NormalizedColorSensor colorSensor;
    NormalizedColorSensor sampleDistance;

    View relativeLayout;

    final float[] hsvValues = new float[3];




    public double intakeServoPosition = 0;



    @Override
    public void runOpMode() throws InterruptedException{
        dashboard = FtcDashboard.getInstance();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        intakeCRSLeft  = hardwareMap.get(CRServo.class, "intakeCRSLeft");
        intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");
        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        lockServo = hardwareMap.get(Servo.class, "lockServo");

        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        armServo = hardwareMap.get(Servo.class, "armServo");

        outmoto1 = hardwareMap.get(DcMotor.class, "outmoto1");
        outmoto2 = hardwareMap.get(DcMotor.class, "outmoto2");

        outmoto1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outmoto1.setTargetPosition(0);
        outmoto1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outmoto1.setPower(0.1);
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        sampleDistance = hardwareMap.get(NormalizedColorSensor.class, "sampleDistance");



        armServo.setPosition(0.475);
        wristServo.setPosition(wristPositionDown);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Wait for the game to start (driver presses START)
        waitForStart();
        //Thread intakeControlThread = new Thread(new intakeController());
        //intakeControlThread.start();

        IntakeController intakeController = new IntakeController(gamepad1, intakeDrive);
        runtime.reset();

        armServo.setPosition(0.475);
        wristServo.setPosition(wristPositionDown);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            boolean sampleDistanceTriggered = false;
            long startTime = 0;
            int state = 0;

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(
                                        -gamepad2.left_stick_y*Math.abs(gamepad2.left_stick_y)*1,
                                        -gamepad2.left_stick_x*Math.abs(gamepad2.left_stick_x) *1
                                ),
                        (-gamepad2.right_stick_x* Math.abs(gamepad2.right_stick_x) * 0.85)
                        ));
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            NormalizedRGBA colors1 = sampleDistance.getNormalizedColors();

                intakeController.run();
            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color. */

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors.toColor(), hsvValues);
             double color = hsvValues[0];

            //intakeDrive.setPower(-gamepad1.left_stick_y * 0.5);
            //new intakeController();

            // Setup a variable for each drive wheel to save power level for telemetry
          if (gamepad1.x){
              intakeCRSLeft.setPower(1);
              intakeCRSRight.setPower(-1);
            }
          if (gamepad1.b){
                intakeCRSLeft.setPower(0);
                intakeCRSRight.setPower(0);
            }




          if ( (gamepad1.right_bumper) || (color > 15 && color < 60 ) ){
             // intakeServoPosition += 0.02;
              intakeServoLeft.setPosition(0.32);
              intakeServoRight.setPosition(0.695);
              intakeCRSLeft.setPower(-0.15);
              intakeCRSRight.setPower(0.15);


              //intake goes up


            }

          if(intakeDrive.getCurrentPosition() < 150 && previousIntakeState){
              lockServo.setPosition(0.3);
              intakeCRSLeft.setPower(-1);
              intakeCRSRight.setPower(1);
              previousIntakeState = false;

          }







          if ((gamepad1.right_trigger > 0.25) ){
              //intakeServoPosition -= 0.02;

              intakeServoLeft.setPosition(0.54);
              intakeServoRight.setPosition(0.45);
              lockServo.setPosition(0);
              intakeCRSLeft.setPower(-1);
              intakeCRSRight.setPower(1);



          }

            if (gamepad1.dpad_up && previousDpadUpState){
                outmoto1.setTargetPosition(2300);
                outmoto1.setPower(1);
                outmoto2.setPower(-outmoto1.getPower());
            }

            if (gamepad1.dpad_left && PreviousDpadLeftState){
                outmoto1.setTargetPosition(1160);
                outmoto1.setPower(1);
                outmoto2.setPower(-outmoto1.getPower());
            }

//            if (gamepad1.dpad_down && !previousDpadDownState){
//                clawServo.setPosition(clawPositionClosed);
//                sleep(200);
//                armServo.setPosition(0.15);
//                wristServo.setPosition(wristPositionDown);
//                sleep(200);
//                clawServo.setPosition(clawPositionOpen);
//
//                outmoto1.setTargetPosition(0);
//                outmoto1.setPower(1);
//                outmoto2.setPower(0);
//                telemetry.addData("yippee", gamepad1.a);
//
//            }
            updateArmTransfer();
            updateArmRetracty();

            if (!outmoto1.isBusy()) {
                    outmoto2.setPower(0);


                }


            if (gamepad1.a){
                wristServo.setPosition(wristPositionOut);
                clawServo.setPosition(clawPositionOpen);
            }

            if (gamepad2.y) {
                intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //intakeServoRight.setPosition(intakeServoPosition);
            previousDpadDownState = gamepad1.dpad_down;
            previousDpadUpState = gamepad1.dpad_up;
            PreviousDpadLeftState = gamepad1.dpad_left;



            if ((intakeDrive.getCurrentPosition() > 145) && outmoto1.getCurrentPosition()<100) {
                previousIntakeState = true;
                if(outmoto1.getCurrentPosition() <15) {
                    armServo.setPosition(armPositionHover);
                    clawServo.setPosition(clawPositionOpen);
                }
            }

//            if (!sampleDistanceTriggered && (((DistanceSensor) sampleDistance).getDistance(DistanceUnit.MM) < 15)) {
//                sampleDistanceTriggered = true;
//                armServo.setPosition(armPositionGrab);
//                clawServo.setPosition(clawPositionClosed);
//                sleep(500);
//                armServo.setPosition(0.15);
//                sleep(750);
//                wristServo.setPosition(wristPositionStraight);
//            }
                TelemetryPacket packet = new TelemetryPacket();

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels

            // Show the elapsed game time and wheel power.
            packet.put("Status", "Run Time: " + runtime.toString());
            packet.put("IntakeServoPosition", intakeServoPosition);
            packet.put("IntakePosition", intakeDrive.getCurrentPosition());
            packet.put("HSV Value", color);
            packet.put("Distance", ((DistanceSensor) sampleDistance).getDistance(DistanceUnit.MM));
            packet.put("Deposit Slides",  outmoto1.getCurrentPosition());
            double batteryVoltage = voltageSensor.getVoltage();
            packet.put("Battery Voltage", batteryVoltage);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Battery Voltage", batteryVoltage);
            telemetry.update();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("IntakeServoPosition", intakeServoPosition);
            telemetry.addData("IntakePosition", intakeDrive.getCurrentPosition());
            telemetry.addData("hsv Value:", color);
            telemetry.addData("Distance", ((DistanceSensor) sampleDistance).getDistance(DistanceUnit.MM));
            telemetry.addData("Deposit Slides", outmoto1.getCurrentPosition());
            telemetry.update();
        }
    }

//    private class intakeController implements Runnable {
//        @Override
//        public void run() {
//            while (opModeIsActive()) {
//                double joystickInput = -gamepad1.left_stick_y;
//                int intakePosition = intakeDrive.getCurrentPosition();
//
//                intakeTargetPosition = intakeTargetPosition + (int)(joystickInput * 40);
//
//                if (intakeTargetPosition < MIN_POSITION) {
//                    intakeTargetPosition = MIN_POSITION;
//                } else if (intakeTargetPosition > MAX_POSITION) {
//                    intakeTargetPosition = MAX_POSITION;
//
//                }
//
//                adb a
//
//                try {
//                    Thread.sleep(20);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//            }
//        }
//    }
}

// to make sure that the thread doesn't hinder other operations




    public void updateArmTransfer() {
        if (!sampleDistanceTriggered && ((DistanceSensor) sampleDistance).getDistance(DistanceUnit.MM) < 15) {
            sampleDistanceTriggered = true;
            startTime = System.currentTimeMillis();
            state = 1; // Start the state machine
        }

        if (sampleDistanceTriggered) {
            long elapsedTime = System.currentTimeMillis() - startTime;

            switch (state) {
                case 1:
                    armServo.setPosition(armPositionGrab);
                    clawServo.setPosition(clawPositionOpen);
                    wristServo.setPosition(wristPositionDown);
                    state = 2;
                    startTime = System.currentTimeMillis(); // Reset timer
                    break;

                case 2:
                    if (elapsedTime >= 150) {

                        clawServo.setPosition(clawPositionClosed);
                        state = 3;
                        startTime = System.currentTimeMillis(); // Reset timer
                    }
                    break;

                case 3:
                    if (elapsedTime >= 200) {

                        armServo.setPosition(0.475);
                        state = 4;
                        startTime = System.currentTimeMillis(); // Reset timer
                    }
                    break;

                case 4:
                    if (elapsedTime >= 750) {
                        wristServo.setPosition(wristPositionStraight);
                        state = 0;
                        sampleDistanceTriggered = false;

                        // End the state machine
                    }
                    break;
            }
        }
    }

    enum RobotState {
        IDLE,
        CLOSE_CLAW,
        MOVE_ARM,
        MOVE_WRIST,
        OPEN_CLAW,
        COMPLETE
    }

    private RobotState currentState = RobotState.IDLE;
    private long stateStartTime = 0;


    public void updateArmRetracty() {
        // Get the current time in milliseconds
        long currentTime = System.currentTimeMillis();

        switch (currentState) {
            case IDLE:
                if (gamepad1.dpad_down && !previousAState) {
                    // Transition to CLOSE_CLAW state
                    clawServo.setPosition(clawPositionClosed);
                    stateStartTime = currentTime; // Record the time
                    currentState = RobotState.CLOSE_CLAW;
                }
                break;

            case CLOSE_CLAW:
                if (currentTime - stateStartTime >= 200) { // Wait 200ms
                    armServo.setPosition(0.475);
                    stateStartTime = currentTime;
                    currentState = RobotState.MOVE_ARM;
                }
                break;

            case MOVE_ARM:
                if (currentTime - stateStartTime >= 200) { // Wait 200ms
                    wristServo.setPosition(wristPositionDown);
                    stateStartTime = currentTime;
                    currentState = RobotState.MOVE_WRIST;
                }
                break;

            case MOVE_WRIST:
                if (currentTime - stateStartTime >= 200) { // Wait 200ms
                    clawServo.setPosition(clawPositionOpen);
                    stateStartTime = currentTime;
                    currentState = RobotState.OPEN_CLAW;
                }
                break;

            case OPEN_CLAW:
                if (currentTime - stateStartTime >= 200) { // Wait 200ms
                    outmoto1.setTargetPosition(0);
                    outmoto1.setPower(1);
                    outmoto2.setPower(0);
                    telemetry.addData("yippee", gamepad1.a);
                    stateStartTime = currentTime;
                    currentState = RobotState.COMPLETE;
                }
                break;

            case COMPLETE:
                if (currentTime-stateStartTime > 200) {
                    currentState = RobotState.IDLE;
                }
                // All actions complete; stay idle or transition as needed
                break;
        }
    }

}
