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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;




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

@TeleOp(name="intake testing", group="Linear OpMode")

public class IntakeTest extends LinearOpMode {

    // Declare OpMode members.

    
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo intakeCRSLeft = null;
    private CRServo intakeCRSRight = null;
    private Servo intakeServoLeft = null;
    private Servo lockServo = null;
    private DcMotor intakeDrive = null;
    private static final int MIN_POSITION = 0;
    private static final int MAX_POSITION = 950;
    int intakeTargetPosition = 0;

    private boolean previousAState = false;
    private boolean previousYState = false;
    private Servo intakeServoRight = null;

    NormalizedColorSensor colorSensor;
    View relativeLayout;

    final float[] hsvValues = new float[3];




    public double intakeServoPosition = 0;



    @Override
    public void runOpMode() throws InterruptedException{
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

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");





        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        // Wait for the game to start (driver presses START)
        waitForStart();
        Thread intakeControlThread = new Thread(new intakeController());
        intakeControlThread.start();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color. */

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors.toColor(), hsvValues);
             double color = hsvValues[0];

            //intakeDrive.setPower(-gamepad1.left_stick_y * 0.5);
            new intakeController();

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
              intakeServoRight.setPosition(0.64);
              intakeCRSLeft.setPower(-0.06);
              intakeCRSRight.setPower(0.06);


              //intake goes up


            }

          if(intakeDrive.getCurrentPosition() < 150){
              lockServo.setPosition(0.5);
              intakeCRSLeft.setPower(-1);
              intakeCRSRight.setPower(1);
          }



          if ((gamepad1.right_trigger > 0.25) ){
              //intakeServoPosition -= 0.02;

              intakeServoLeft.setPosition(0.48);
              intakeServoRight.setPosition(0.48);
              lockServo.setPosition(0.75);
              intakeCRSLeft.setPower(-1);
              intakeCRSRight.setPower(1);



          }

            //intakeServoRight.setPosition(intakeServoPosition);
            previousAState = gamepad1.a;
            previousYState = gamepad1.y;


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
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("IntakeServoPosition", intakeServoPosition);
            telemetry.addData("IntakePosition", intakeDrive.getCurrentPosition());
            telemetry.addData("hsv Value:", color);
            telemetry.update();
        }
    }
    
    private class intakeController implements Runnable {
        @Override
        public void run() {
            while (opModeIsActive()) {
                double joystickInput = -gamepad1.left_stick_y;
                int intakePosition = intakeDrive.getCurrentPosition();

                intakeTargetPosition = intakeTargetPosition + (int)(joystickInput * 40);

                if (intakeTargetPosition < MIN_POSITION) {
                    intakeTargetPosition = MIN_POSITION;
                } else if (intakeTargetPosition > MAX_POSITION) {
                    intakeTargetPosition = MAX_POSITION;
                    
                }
                
                intakeDrive.setTargetPosition(intakeTargetPosition);
                intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeDrive.setPower(1);
                
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
