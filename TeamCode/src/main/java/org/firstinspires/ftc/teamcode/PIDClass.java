package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Administrator on 2016/3/15.
 */
public class PIDClass {

    static public boolean runCertainDistance(DcMotor LFMotor, DcMotor LBMotor, DcMotor RFMotor, DcMotor RBMotor, double power, double distance/*cm*/) {
        double position = distance / (2 * Math.PI * RobotMap.RADIUS);
        double LPosition = LFMotor.getCurrentPosition();
        double RPosition = RBMotor.getCurrentPosition();
        if(distance > 0) {
            if (LPosition <= position && RPosition <= position) {

                if (!inRangeOf(LPosition, RPosition - 5, RPosition + 5)) {
                    double shift = LPosition - RPosition;
                    double deltaPower = gain(shift);
                    LFMotor.setPower(-(power - deltaPower));
                    LBMotor.setPower(-(power - deltaPower));
                    RFMotor.setPower(power + deltaPower);
                    RBMotor.setPower(power + deltaPower);
                } else {
                    LFMotor.setPower(-power);
                    LBMotor.setPower(-power);
                    RFMotor.setPower(power);
                    RBMotor.setPower(power);
                }
                return false;
            }
            else {
                return true;
            }

        } else{
            if(LPosition >= position && RPosition >= position){
                if (!inRangeOf(LPosition, RPosition - 5, RPosition + 5)) {
                    double shift = LPosition - RPosition;
                    double deltaPower = gain(shift);
                    LFMotor.setPower(-(power + deltaPower));
                    LBMotor.setPower(-(power + deltaPower));
                    RFMotor.setPower(power - deltaPower);
                    RBMotor.setPower(power - deltaPower);
                } else {
                    LFMotor.setPower(-power);
                    LBMotor.setPower(-power);
                    RFMotor.setPower(power);
                    RBMotor.setPower(power);
                }
                return false;
            }
            else {
                return true;
            }
            }
    }


    static public boolean inRangeOf(double value, double min, double max){
        if(value >= min && value <= max)
            return true;
        else
            return false;

    }

    static public double gain(double shift){
        double gain;
        gain = 3.5E-5 * Math.pow(shift,3)/2;
        return gain;
    }

    static public boolean turn(DcMotor LFMotor, DcMotor LBMotor, DcMotor RFMotor, DcMotor RBMotor, double startingDegree, double currentDegree,double targetDegree){
        double leftVaule;
        double rightVaule;
        if(ifToDegree(startingDegree,currentDegree,targetDegree)) {
            if (startingDegree <= targetDegree) {
                leftVaule = 0;
                rightVaule = RobotMap.MOTOR_TURN_RIGHTPOWER;
            } else {
                leftVaule = RobotMap.MOTOR_TURN_LEFTPOWER;
                rightVaule = 0;
            }

            LFMotor.setPower(-leftVaule);
            LBMotor.setPower(-leftVaule);
            RFMotor.setPower(rightVaule);
            RBMotor.setPower(rightVaule);

            return false;
        }
        else{
            return true;
        }
    }

    static public boolean ifToDegree(double startingDegree,double currentDegree,double targetDegree){
        if (startingDegree * 6>= targetDegree){
            if(currentDegree * 6 <= targetDegree)
                return true;
            else
                return false;
        }
        else{
            if(currentDegree * 6 >= targetDegree)
                return true;
            else
                return false;
        }
    }

    static public void resetEncode(DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForRefresh();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    static public void waitForRefresh(){
        for(int i = 0; i<= 20000;i++){

        }
    }
}
