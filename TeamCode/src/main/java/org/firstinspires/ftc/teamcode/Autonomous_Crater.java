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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.teamcode.HardwarePrimeTech.COUNTS_PER_MM;
import static org.firstinspires.ftc.teamcode.HardwarePrimeTech.MARKER_RELEASED;
import static org.firstinspires.ftc.teamcode.HardwarePrimeTech.TURN_SPEED;
import static org.firstinspires.ftc.teamcode.HardwarePrimeTech.driveValue;
import static org.firstinspires.ftc.teamcode.HardwarePrimeTech.liftValue;
import static org.firstinspires.ftc.teamcode.HardwarePrimeTech.strafeValue;
import static org.firstinspires.ftc.teamcode.HardwarePrimeTech.turnValue;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous_Crater", group="Pushbot")
//@Disabled
public class Autonomous_Crater extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePrimeTech         robot   = new HardwarePrimeTech();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    int gold_position = 0;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AZdEGAb/////AAABmZ4AF6SZ9UY4tTSsTIZJ44oEW7p8rxW7ag25tT3UZ/fuIsUS/lDsxQlWoG+guqvqmGpsx6Uut/kOGSXDvbaZ92ttTmKtyFS9V1V6LIscYnPO73m6a5FK9WOq3/4xNyyEU1fLmRjmPlO1A2Yg9GtgWPmt8xqcblGsOgjg3rO6/BfXbPXtPLmHgePVwp/FCwEpGb7czfi5AIYcaazuOLKl6DTFzXdq1vlhr/1SZQlfhYOR74ECNclPfQXuLk+AOEoUyWvVemG/gdh1StRGIdUy0upbeM+bKqhOUqUvKC4MasSoQ4REok9uLmsZXxbpEnWiJ1ZbGACRgTb7SdYOY9mFLlkOoV5mPJ6DcGssF1Pkbwzj";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        /***VUFORIA + TENSORFLOW***/
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        /***ENCODERS***/
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.backLeftMotor.getCurrentPosition(),
                robot.backRightMotor.getCurrentPosition());
        telemetry.update();


        if (tfod != null) {
            tfod.activate();
        }

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Status:", "Wait for start");
            telemetry.update();
        }

        runtime.reset();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        waitForStart();

        /***    AUTONOMUS STARTS HERE                           ***/
        /***                AUTONOMUS STARTS HERE               ***/
        /***                            AUTONOMUS STARTS HERE   ***/



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public int checkTensorFlow ( int T){
        int goldie = 0;
        if (opModeIsActive()) {
            while (T > 0 && opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    goldie = -1;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    goldie = 1;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    goldie = 0;
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
                sleep(1);
                T--;
            }
            if (tfod != null) {
                tfod.shutdown();
            }
        }
        return goldie;
    }



    private void initVuforia () {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }



    private void initTfod () {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void encoderDrive ( double speed, double distance, double timeoutS){
        if (opModeIsActive()) {
            int newBackLeftTarget;
            int newBackRightTarget;
            int newFrontLeftTarget;
            int newFrontRightTarget;
            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newBackLeftTarget = robot.backLeftMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * driveValue);
                newBackRightTarget = robot.backRightMotor.getCurrentPosition() + (int) (-distance * COUNTS_PER_MM * driveValue);
                newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * driveValue);
                newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int) (-distance * COUNTS_PER_MM * driveValue);

                robot.backLeftMotor.setTargetPosition(newBackLeftTarget);
                robot.backRightMotor.setTargetPosition(newBackRightTarget);
                robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
                robot.frontRightMotor.setTargetPosition(newFrontRightTarget);

                // Turn On RUN_TO_POSITION
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.backLeftMotor.setPower(speed);
                robot.backRightMotor.setPower(-speed);
                robot.frontLeftMotor.setPower(speed);
                robot.frontRightMotor.setPower(-speed);

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (
                                robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() &&
                                        robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()
                        )) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d"
                            , newFrontLeftTarget, newFrontRightTarget
                            , newBackLeftTarget, newBackRightTarget
                    );
                    telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            robot.frontLeftMotor.getCurrentPosition(),
                            robot.frontRightMotor.getCurrentPosition()
                            ,
                            robot.backLeftMotor.getCurrentPosition(),
                            robot.backRightMotor.getCurrentPosition()
                    );
                    telemetry.update();
                }

                // Stop all motion;
                /**
                 robot.backLeftMotor.setPower(0);
                 robot.backRightMotor.setPower(0);
                 robot.frontLeftMotor.setPower(0);
                 robot.frontRightMotor.setPower(0);
                 **/
                /** COMMENT THESE FOR SPEED **/

                // Turn off RUN_TO_POSITION
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //sleep(250);   // optional pause after each move
            }
        }

    }

    public void encoderStrafe(double speed, double distance, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = robot.backLeftMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * strafeValue);
            newBackRightTarget = robot.backRightMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * strafeValue);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * strafeValue);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * strafeValue);

            robot.backLeftMotor.setTargetPosition(newBackLeftTarget);
            robot.backRightMotor.setTargetPosition(newBackRightTarget);
            robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.frontRightMotor.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.backLeftMotor.setPower(-speed);
            robot.backRightMotor.setPower(-speed);
            robot.frontLeftMotor.setPower(speed);
            robot.frontRightMotor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (
                            robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() &&
                                    robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()
                    ))
            {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d"
                        , newFrontLeftTarget, newFrontRightTarget
                        , newBackLeftTarget, newBackRightTarget
                );
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition()
                        ,
                        robot.backLeftMotor.getCurrentPosition(),
                        robot.backRightMotor.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            /**
             robot.backLeftMotor.setPower(0);
             robot.backRightMotor.setPower(0);
             robot.frontLeftMotor.setPower(0);
             robot.frontRightMotor.setPower(0);
             **/
            // Turn off RUN_TO_POSITION
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move
        }
    }

    public void driveForward (double distance, double speed)
    {
        encoderDrive(speed, -distance,15);
    }

    public void driveBackward (double distance, double speed) {
        encoderDrive(-speed, distance,15);
    }

    public void strafeRight(double distance, double speed) {
        encoderStrafe(speed, -distance, 15);
    }

    public void strafeLeft (double distance, double speed) {
        encoderStrafe(-speed, distance, 15);
    }


    public void encoderTurn(double speed, double distance, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = robot.backLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * turnValue);
            newBackRightTarget = robot.backRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * turnValue);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * turnValue);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * turnValue);

            robot.backLeftMotor.setTargetPosition(newBackLeftTarget);
            robot.backRightMotor.setTargetPosition(newBackRightTarget);
            robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.frontRightMotor.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.backLeftMotor.setPower(speed);
            robot.backRightMotor.setPower(speed);
            robot.frontLeftMotor.setPower(speed);
            robot.frontRightMotor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (
                            robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy() &&
                                    robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()
                    ))
            {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d"
                        , newFrontLeftTarget, newFrontRightTarget
                        , newBackLeftTarget, newBackRightTarget
                );
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition()
                        ,
                        robot.backLeftMotor.getCurrentPosition(),
                        robot.backRightMotor.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move
        }
    }

    public void rotateRight(double angle)
    {
        encoderTurn(TURN_SPEED, angle, 15);
    }

    public void rotateLeft(double angle)
    {
        encoderTurn(-TURN_SPEED, -angle, 15);
    }

    public void encoderLift(double speed, double distance, double direction, double timeoutS) {
        int liftTarget;

        if (direction == -1)
            robot.liftMotor.setDirection(DcMotor.Direction.FORWARD);
        else if (direction == 1)
            robot.liftMotor.setDirection(DcMotor.Direction.REVERSE);
        // Ensure that the opmode is still active
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (opModeIsActive() && !isStopRequested() && !(gamepad1.dpad_up && gamepad1.b)) {

            // Determine new target position, and pass to motor controller
            liftTarget = robot.liftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * liftValue);

            robot.liftMotor.setTargetPosition(liftTarget);

            // Turn On RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.liftMotor.isBusy())
                    && !isStopRequested())
            {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to :%7d", liftTarget);
                telemetry.addData("Path2",  "Running at :%7d", robot.liftMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.liftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor.setDirection(DcMotor.Direction.FORWARD);

            //sleep(250);   // optional pause after each move
        }
    }

    public void lowerRobot()
    {
        encoderLift(1, 15, 1, 15);
    }

    public void dropMarker() {
        stopAllMotion();
        if (opModeIsActive())
        {
            robot.markerServo.setPosition(MARKER_RELEASED);
            idle();
        }
    }


    public void stopAllMotion() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        robot.stringMotor.setPower(0);
        robot.liftMotor.setPower(0);
        robot.armMotor.setPower(0);
    }

}

