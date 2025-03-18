/**
 * Utility functions for coordinate transformations between screen and world coordinates
 */

// VEX field size in inches
export const VEX_FIELD_SIZE = 144;

// Default canvas size (will be made responsive)
export const DEFAULT_CANVAS_SIZE = {
  width: 800,
  height: 800,
};

// Ratio of pixels to inches
export const getInchToPixelRatio = (canvasWidth: number) => canvasWidth / VEX_FIELD_SIZE;

/**
 * Convert world coordinates to screen coordinates
 * @param coords [x, y] in world coordinates (inches)
 * @param canvasSize Canvas dimensions in pixels
 * @returns [x, y] in screen coordinates (pixels)
 */
export const coordToPixels = (
  coords: [number, number], 
  canvasSize = DEFAULT_CANVAS_SIZE
): [number, number] => {
  const inchToPixel = getInchToPixelRatio(canvasSize.width);
  return [
    coords[0] * inchToPixel + canvasSize.width / 2,
    coords[1] * -inchToPixel + canvasSize.height / 2
  ];
};

/**
 * Convert screen coordinates to world coordinates
 * @param pixels [x, y] in screen coordinates (pixels)
 * @param canvasSize Canvas dimensions in pixels
 * @returns [x, y] in world coordinates (inches)
 */
export const pixelsToCoord = (
  pixels: [number, number], 
  canvasSize = DEFAULT_CANVAS_SIZE
): [number, number] => {
  const inchToPixel = getInchToPixelRatio(canvasSize.width);
  return [
    (pixels[0] - canvasSize.width / 2) / inchToPixel,
    (pixels[1] - canvasSize.height / 2) / -inchToPixel
  ];
};

/**
 * Calculate point radius in pixels based on canvas size and custom radius in inches
 * @param canvasSize Canvas dimensions in pixels
 * @param radiusInInches Optional custom radius in inches
 * @returns Point radius in pixels
 */
export const getPointRadius = (canvasSize = DEFAULT_CANVAS_SIZE, radiusInInches?: number): number => {
  const inchToPixel = getInchToPixelRatio(canvasSize.width);
  // Default is 14.5/2 inches (the diameter used in the original visualizer)
  const radius = radiusInInches !== undefined ? radiusInInches : (14.5 / 2);
  return radius * inchToPixel;
};

/**
 * Calculate pose circle radius in pixels based on canvas size and custom radius in inches
 * @param canvasSize Canvas dimensions in pixels
 * @param radiusInInches Optional custom radius in inches
 * @returns Pose circle radius in pixels
 */
export const getPoseRadius = (canvasSize = DEFAULT_CANVAS_SIZE, radiusInInches?: number): number => {
  const inchToPixel = getInchToPixelRatio(canvasSize.width);
  // Default is 10/2 inches
  const radius = radiusInInches !== undefined ? radiusInInches : (10 / 2);
  return radius * inchToPixel;
};
