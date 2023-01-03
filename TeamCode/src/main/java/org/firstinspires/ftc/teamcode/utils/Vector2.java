package org.firstinspires.ftc.teamcode.utils;

public class Vector2 {
    public double originX, originY, x, y;

    public Vector2(double originX, double originY, double x, double y) {
        this.originX = originX;
        this.originY = originY;
        this.x = x;
        this.y = y;
    }

    public Vector2(Vector2 v) {
        this.originX = v.originX;
        this.originY = v.originY;
        this.x = v.x;
        this.y = v.y;
    }

    public double dot(Vector2 v) {
        return this.x * v.x + this.y * v.y;
    }

    public Vector2 parallelProj(Vector2 v, double x, double y) {
        double dotV = this.dot(v);
        double dotThis = this.dot(this);
        double scale = dotV / dotThis;

        return new Vector2(x, y, this.x * scale, this.y * scale);
    }

    public Vector2 perpendicularProj(Vector2 v, double x, double y) {
        Vector2 parallel = this.parallelProj(v, x, y);
        return new Vector2(x, y, v.x - parallel.x, v.y - parallel.y);
    }

    public double len() {
        return Math.sqrt(x * x + y * y);
    }

    public void scale(double t) {
        this.x *= t;
        this.y *= t;
    }
}