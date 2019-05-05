package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwarePrimeTech
{
    /* Public OpMode members. */
    public DcMotor  frontLeftMotor;
    public DcMotor  backLeftMotor;
    public DcMotor  frontRightMotor;
    public DcMotor  backRightMotor;

    public DcMotor  stringMotor;
    public DcMotor  armMotor;
    public DcMotor  liftMotor;

    public Servo markerServo;
    public CRServo collectionServo;


    ///Variables

    public static final double      driveValue = 2.43;
    public static final double      strafeValue = 2;
    public static final double      diagonalValue = 4;
    public static final double      liftValue = 305;
    public static final double      stringValue = 1;
    public static final double      turnValue = 2;

    public static final double      DRIVE_SPEED = 1;
    public static final double      TURN_SPEED = 0.5;
    public static final double      MARKER_START = 0;
    public static final double      MARKER_RELEASED = 1;
    ///ENCODER VARIABLES
    public static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder 1440 tetrix
    public static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    public static final double     WHEEL_DIAMETER_MM   = 4.0 * 25.4;     // For figuring circumference
    public static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    public static final double     HEADING_THRESHOLD       = 10;
    public static final double     P_TURN_COEFF            = 0.1;
    public final static double MAX_PUTERE = 1.0;
    public final static double MIN_PUTERE = -1.0;



    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePrimeTech(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftMotor = hwMap.get(DcMotor.class, "FrontLeftMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "BackLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "FrontRightMotor");
        backRightMotor = hwMap.get(DcMotor.class, "BackRightMotor");
        armMotor = hwMap.get(DcMotor.class, "ArmMotor");
        stringMotor = hwMap.get(DcMotor.class, "StringMotor");
        liftMotor = hwMap.get(DcMotor.class, "LiftMotor");

        markerServo = hwMap.get(Servo.class, "MarkerServo");
        collectionServo = hwMap.get(CRServo.class, "CollectionServo");

        //Setting the direction of the motors
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setDirection(DcMotor.Direction.REVERSE);
        stringMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        armMotor.setPower(0);
        stringMotor.setPower(0);
        liftMotor.setPower(0);

        //Setting the positions of the servos
        collectionServo.setPower(0);
        markerServo.setPosition(MARKER_START);


    }
 }

