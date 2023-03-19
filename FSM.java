// This code is an example of FSM in an actual program.

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="FSM Example")
public class FSM extends OpMode {
    //Setting steps of the FSM (or states)
    public enum LiftState{
        GRIPING,
        LIFTING,
        RELEASE,
        LOWER
    }

    //Setting the first step
    LiftState liftState = LiftState.GRIPING;

    ElapsedTime gripingTimer = new ElapsedTime();
    ElapsedTime releaseTimer = new ElapsedTime();

    double slideVelocity = 300;

    //Declaring Servos and Motors
    Servo gripper;
    DcMotorEx slide;

    public void init() {
        //Initalize your motors, servos, or sensors here
    }

    public void loop() {
        //The actual part of the code that loops during teleop
        switch (liftState) {
            case GRIPING:
                //First step (or state) of the FSM
                break;
            case LIFTING:
                //Second step (or state) of the FSM
                break;
            case RELEASE:
                //Third step (or state) of the FSM
                break;
            case LOWER:
                //Third step (or state) of the FSM
                break;
            default:
                //This state is reached if the FSM cycle gets out of order
                liftState = LiftState.GRIPING;
        }
    }
}

