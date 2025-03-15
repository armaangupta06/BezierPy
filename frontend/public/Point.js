class Point {
    constructor(x, y) {
      this.x = x;
      this.y = y;
    }
    // Return a new Point that is the sum of this and another point.
    add(other) {
      return new Point(this.x + other.x, this.y + other.y);
    }
    // Return a new Point that is the difference of this and another point.
    subtract(other) {
      return new Point(this.x - other.x, this.y - other.y);
    }
    // Multiply by a scalar.
    multiply(scalar) {
      return new Point(this.x * scalar, this.y * scalar);
    }
    // Divide by a scalar.
    divide(scalar) {
      return new Point(this.x / scalar, this.y / scalar);
    }
    // Round the coordinates.
    round(ndigits = 0) {
      return new Point(Number(this.x.toFixed(ndigits)), Number(this.y.toFixed(ndigits)));
    }
    toString() {
      return `(${this.x}, ${this.y})`;
    }
  }
  
  function distance_formula(p1, p2) {
    return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
  }
  
  function magnitude(p) {
    return Math.sqrt(p.x * p.x + p.y * p.y);
  }
  
  function dot_product(v1, v2) {
    return v1.x * v2.x + v1.y * v2.y;
  }
  
  function determinant(v1, v2) {
    return v1.x * v2.y - v2.x * v1.y;
  }
  
  function slope(p1, p2) {
    return (p2.subtract(p1)).y / (p2.subtract(p1)).x;
  }
  
  /**
   * Returns a unit vector perpendicular to the angle bisector
   * of the angle formed by three points A, B, and C.
   */
  function getPerpendicularVector(A, B, C) {
    const AB = B.subtract(A);
    const BC = C.subtract(B);
    const v1 = AB.divide(magnitude(AB));
    const v2 = BC.divide(magnitude(BC));
    const sum = new Point(v1.x + v2.x, v1.y + v2.y);
    const magSum = magnitude(sum);
    if (magSum === 0) return new Point(1, 0);
    return sum.divide(magSum);
  }