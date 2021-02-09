package frc.robot.util;

public class Circle {
    double radius;
    Waypoint center;
    Waypoint end;
    Waypoint start;

    public Circle(double radius, Waypoint center){
        this.radius = radius;
        this.center = center;
    }

    public Circle(double radius, Waypoint start, Waypoint end){
        this.radius = radius;
        this.start = start;
        this.end = end;
    }

    public double tangentAngle(double botX, double botY, double botAngle){
        //(botY - center.y)*(botY - center.y)=radius*radius-((botX - center.x)*(botX - center.x));
        double radAngle = Math.toDegrees(Math.atan2(botY-center.y, botX-center.x));
        double delta = Util.angleDiff(radAngle+90, botAngle);
        if (delta>90){
            return radAngle -90;
        } else {
            return radAngle +90;
        }

    }
}
