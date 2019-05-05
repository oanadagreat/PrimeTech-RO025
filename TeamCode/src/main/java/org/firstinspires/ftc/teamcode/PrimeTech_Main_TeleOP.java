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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.HardwarePrimeTech.COUNTS_PER_MM;
import static org.firstinspires.ftc.teamcode.HardwarePrimeTech.MARKER_RELEASED;
import static org.firstinspires.ftc.teamcode.HardwarePrimeTech.MARKER_START;
import static org.firstinspires.ftc.teamcode.HardwarePrimeTech.stringValue;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="PrimeTech_Main_TeleOP", group="Linear Opmode")
//@Disabled
public class PrimeTech_Main_TeleOP extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    double drive = 0;
    double turn = 0;
    double strafe = 0;

    double frontLeftPower = 0;
    double frontRightPower = 0;
    double backLeftPower = 0;
    double backRightPower = 0;

    double armsPower = 0;
    double stringPower = 0;
    double liftPower = 0;

    HardwarePrimeTech robot = new HardwarePrimeTech();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Status", "Waiting for start");
            telemetry.update();
        }
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

            /**GAMEPAD 1**/

            ///Drive
            if (gamepad1.left_stick_y != 0)
                drive = -gamepad1.left_stick_y;
            else if (gamepad2.left_stick_y != 0)
                drive = -gamepad2.left_stick_y;
            else drive = 0;

            if (gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0)
            {
                if (gamepad1.left_stick_x != 0)
                    strafe = -gamepad1.left_stick_x;
                else strafe = -gamepad1.right_stick_x;
            }
            else if (gamepad2.left_stick_x != 0 || gamepad2.right_stick_x != 0)
            {
                if (gamepad2.left_stick_x != 0)
                    strafe = -gamepad2.left_stick_x;
                else strafe = -gamepad2.right_stick_x;
            }
            else strafe = 0;

            if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0)
            {
                if (gamepad1.left_trigger != 0)
                    turn = -gamepad1.left_trigger;
                else if (gamepad1.right_trigger != 0)
                    turn = gamepad1.right_trigger;
            }
            else if (gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0)
            {
                if (gamepad2.left_trigger != 0)
                    turn = -gamepad2.left_trigger;
                else if (gamepad2.right_trigger != 0)
                    turn = gamepad2.right_trigger;
            }
            else turn = 0;
            //turn = gamepad1.right_stick_x;

            frontLeftPower  = Range.clip(-drive - turn + strafe, -1.0, 1.0);
            frontRightPower = Range.clip(+drive - turn + strafe, -1.0, 1.0);
            backLeftPower   = Range.clip(-drive - turn - strafe, -1.0, 1.0);
            backRightPower  = Range.clip(+drive - turn - strafe, -1.0, 1.0);

            ///Lift

            if (gamepad1.a)
                robot.collectionServo.setPower(1);
            else if (gamepad1.b)
                robot.collectionServo.setPower(-1);
            else if (gamepad1.y)
                robot.collectionServo.setPower(0);

            if (gamepad1.dpad_left)
                stringPower = 1;
            else if (gamepad1.dpad_right || gamepad2.dpad_right)
                stringPower = -1;
            else stringPower = 0;

            if (gamepad1.dpad_up)
                armsPower = 1;
            else if (gamepad1.dpad_down)
                armsPower = -1;
            else armsPower = 0;

            /**GAMEPAD 2**/

            if (gamepad2.dpad_up)
                liftPower = 1;
            else if (gamepad2.dpad_down)
                liftPower = -1;
            else liftPower = 0;

            if (gamepad2.a)
                robot.markerServo.setPosition(MARKER_START);
            else if (gamepad2.b)
                robot.markerServo.setPosition(MARKER_RELEASED);

            /**PUTERI**/

            // PUTERE LA MOTOARE DE DEPLASARE
            robot.backLeftMotor.setPower(backLeftPower);
            robot.frontLeftMotor.setPower(frontLeftPower);
            robot.backRightMotor.setPower(backRightPower);
            robot.frontRightMotor.setPower(frontRightPower);

            robot.armMotor.setPower(armsPower);
            //robot.rightArmMotor.setPower(armsPower);
            robot.stringMotor.setPower(stringPower);
            robot.liftMotor.setPower(liftPower);
        }

        stopItDude();

    }

    public void stopItDude(){
        //OPRIT MOTOARE DE DEPLASARE
        robot.backLeftMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.frontRightMotor.setPower(0);

        robot.armMotor.setPower(0);
        //robot.rightArmMotor.setPower(0);
        robot.stringMotor.setPower(0);
        robot.liftMotor.setPower(0);

    }

    public void extendString(double distance, double speed)
    {
        if (opModeIsActive())
        {
            int stringTarget;
            robot.stringMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.stringMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stringTarget = robot.stringMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * stringValue);
            robot.stringMotor.setTargetPosition(stringTarget);
            robot.stringMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.stringMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && robot.stringMotor.isBusy() && !isStopRequested())
            {
                telemetry.addData("Path1",  "Running to :%7d", stringTarget);
                telemetry.addData("Path2",  "Running at :%7d", robot.stringMotor.getCurrentPosition());
                telemetry.update();
            }

            robot.stringMotor.setPower(0);

        }
    }

    public void encoderArm(double speed, double distance, double direction, double timeoutS) {

        if (opModeIsActive() && !isStopRequested() && !(gamepad1.dpad_up && gamepad1.b)) {

            int armTarget;

            if (direction == -1)
                robot.liftMotor.setDirection(DcMotor.Direction.REVERSE);
            else if (direction == 1)
                robot.liftMotor.setDirection(DcMotor.Direction.FORWARD);
            // Ensure that the opmode is still active
            robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            armTarget = robot.liftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM);

            robot.liftMotor.setTargetPosition(armTarget);

            // Turn On RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.liftMotor.isBusy()) && !isStopRequested())
            {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to :%7d", armTarget);
                telemetry.addData("Path2",  "Running at :%7d", robot.liftMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.liftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor.setDirection(DcMotor.Direction.FORWARD);
        }
    }
}
