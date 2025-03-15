// Pose.js
import { Point } from "./Point.js";

export class Pose extends Point {
  constructor(x, y, heading) {
    super(x, y);
    // Convert heading from degrees to radians.
    this.heading = heading * Math.PI / 180;
  }
}
