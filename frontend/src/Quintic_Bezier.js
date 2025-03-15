// QuinticBezier.js
import { Point, distance_formula, magnitude, determinant } from "./Point.js";

export class Quintic_Bezier {
  constructor(point0, point1, point2, point3, point4, point5) {
    this.point0 = point0;
    this.point1 = point1;
    this.point2 = point2;
    this.point3 = point3;
    this.point4 = point4;
    this.point5 = point5;
  }
  
  getControlPoints() {
    return [this.point0, this.point1, this.point2, this.point3, this.point4, this.point5];
  }
  
  calcFirstDerivative(t) {
    const term1 = this.point1.subtract(this.point0).multiply(5 * Math.pow(1 - t, 4));
    const term2 = this.point2.subtract(this.point1).multiply(20 * t * Math.pow(1 - t, 3));
    const term3 = this.point3.subtract(this.point2).multiply(30 * Math.pow(t, 2) * Math.pow(1 - t, 2));
    const term4 = this.point4.subtract(this.point3).multiply(20 * Math.pow(t, 3) * (1 - t));
    const term5 = this.point5.subtract(this.point4).multiply(5 * Math.pow(t, 4));
    return term1.add(term2).add(term3).add(term4).add(term5);
  }
  
  calcSecondDerivative(t) {
    const term1 = this.point2.subtract(this.point1.multiply(2)).add(this.point0)
      .multiply(20 * Math.pow(1 - t, 3));
    const term2 = this.point3.subtract(this.point2.multiply(2)).add(this.point1)
      .multiply(60 * t * Math.pow(1 - t, 2));
    const term3 = this.point4.subtract(this.point3.multiply(2)).add(this.point2)
      .multiply(60 * Math.pow(t, 2) * (1 - t));
    const term4 = this.point5.subtract(this.point4.multiply(2)).add(this.point3)
      .multiply(20 * Math.pow(t, 3));
    return term1.add(term2).add(term3).add(term4);
  }
  
  calcCurvature(t) {
    const firstDeriv = this.calcFirstDerivative(t);
    if (magnitude(firstDeriv) === 0) {
      return 0;
    }
    const secondDeriv = this.calcSecondDerivative(t);
    return determinant(firstDeriv, secondDeriv) / Math.pow(magnitude(firstDeriv), 3);
  }
  
  getPoint(t) {
    const term0 = this.point0.multiply(Math.pow(1 - t, 5));
    const term1 = this.point1.multiply(5 * Math.pow(1 - t, 4) * t);
    const term2 = this.point2.multiply(10 * Math.pow(1 - t, 3) * Math.pow(t, 2));
    const term3 = this.point3.multiply(10 * Math.pow(1 - t, 2) * Math.pow(t, 3));
    const term4 = this.point4.multiply(5 * (1 - t) * Math.pow(t, 4));
    const term5 = this.point5.multiply(Math.pow(t, 5));
    return term0.add(term1).add(term2).add(term3).add(term4).add(term5);
  }
  
  calcArcLength() {
    let distance = 0;
    let prevPoint = this.getPoint(0);
    // Loop from t=0 to t=1 in steps of 0.01
    for (let t = 0; t <= 1.0001; t += 0.01) {
      const currPoint = this.getPoint(t);
      distance += distance_formula(currPoint, prevPoint);
      prevPoint = currPoint;
    }
    return distance;
  }
  

}
