package org.firstinspires.ftc.teamcode;

public class CenterStageEnums {

    public enum AprilTag{
        BlueLeft(1),
        BlueCenter(2),
        BlueRight(3),
        RedLeft(4),
        RedCenter(5),
        RedRight(6);
        private int value;
        AprilTag(int value){
            this.value=value;
        }
        public int getValue() {
            return value;
        }
    }

    public enum ArmDirection {
        Down,
        None,
        Up
    }

    public enum FollowDirection {
        Rotate,
        Strafe,
        Straight
    }


    public enum Position {
        Down,
        None,
        Up
    }

    public enum StrafeDirection {
        Right,
        Left
    }

    public enum TapeColor {
        Blue,
        Red,
        Nothing
    }

    public enum TapeLocation{
        Left,
        Center,
        Right
    }
}
