import React, { useRef, useState, useEffect, useMemo } from 'react';
import { motion } from 'framer-motion';
import { coordToPixels, pixelsToCoord, getPointRadius } from '@/utils/coordinate-transforms';
import { generatePathPoints } from '@/utils/bezier-utils';
import { PoseModel, PointModel, BezierCurveModel } from '@/services/api';

interface PathCanvasProps {
  poses: PoseModel[];
  points?: PointModel[];
  controlPointsList?: BezierCurveModel[];
  pathPoints?: any[];
  trajectoryPoints?: any[];
  onAddPoint: (x: number, y: number) => void;
  onMovePoint: (index: number, x: number, y: number) => void;
  onSelectPoint: (index: number) => void;
  selectedPointIndex: number | null;
  showTrajectory: boolean;
  canvasSize: { width: number; height: number };
  pathCreationMethod?: 'poses' | 'points' | 'control-points';
  onControlPointsChange?: (controlPoints: BezierCurveModel[]) => void;
  onUpdatePoseHeading?: (index: number, heading: number) => void;
}

const PathCanvas: React.FC<PathCanvasProps> = ({
  poses = [],
  points = [],
  controlPointsList = [],
  pathPoints = [],
  trajectoryPoints = [],
  onAddPoint,
  onMovePoint,
  onSelectPoint,
  selectedPointIndex,
  showTrajectory,
  canvasSize,
  pathCreationMethod = 'poses',
  onControlPointsChange,
  onUpdatePoseHeading,
}) => {
  const canvasRef = useRef<HTMLDivElement>(null);
  const [isDragging, setIsDragging] = useState(false);
  const [dragPointIndex, setDragPointIndex] = useState<number | null>(null);
  const [mousePosition, setMousePosition] = useState<[number, number] | null>(null);
  const pointRadius = getPointRadius(canvasSize);
  
  // State for heading editing
  const [isEditingHeading, setIsEditingHeading] = useState(false);
  const [headingEditPointIndex, setHeadingEditPointIndex] = useState<number | null>(null);

  // Store the current dragging control point indices to avoid recalculating on every move
  const [dragControlPoint, setDragControlPoint] = useState<{ curveIndex: number, pointIndex: number } | null>(null);
  
  // Throttle mouse move updates to reduce lag
  const lastUpdateTimeRef = useRef<number>(0);
  const pendingUpdateRef = useRef<boolean>(false);
  
  // Separate function to handle the actual drag update logic
  const handleDragUpdate = (x: number, y: number) => {
    // Get the world coordinates
    const worldCoords = pixelsToCoord([x, y], canvasSize);
    
    console.log('Drag update at:', x, y, 'world:', worldCoords);
    console.log('Drag state:', { isDragging, dragControlPoint, dragPointIndex });
    
    // If we're dragging a specific control point, update it directly
    if (isDragging && dragControlPoint && controlPointsList && controlPointsList.length > 0 && onControlPointsChange) {
      const { curveIndex, pointIndex } = dragControlPoint;
      console.log('Dragging control point:', curveIndex, pointIndex);
      
      // Make sure the indices are valid
      if (curveIndex >= 0 && curveIndex < controlPointsList.length && 
          pointIndex >= 0 && pointIndex < controlPointsList[curveIndex].control_points.length) {
        
        // Create a new array with only the modified curve changed
        const updatedControlPoints = [...controlPointsList];
        const updatedCurve = { ...updatedControlPoints[curveIndex] };
        const updatedControlPointsArray = [...updatedCurve.control_points];
        
        // Get the original point for logging
        const originalPoint = updatedControlPointsArray[pointIndex];
        console.log('Moving control point from:', originalPoint, 'to:', { x: worldCoords[0], y: worldCoords[1] });
        
        // Update just the specific point that changed
        updatedControlPointsArray[pointIndex] = {
          ...updatedControlPointsArray[pointIndex],
          x: worldCoords[0],
          y: worldCoords[1]
        };
        
        updatedCurve.control_points = updatedControlPointsArray;
        updatedControlPoints[curveIndex] = updatedCurve;
        
        // Call the callback to update the control points
        onControlPointsChange(updatedControlPoints);
        return; // Exit early after handling control point drag
      }
    }
    
    // If we're not dragging a control point but have a dragPointIndex, handle regular point dragging
    if (isDragging && dragPointIndex !== null) {
      console.log('Dragging regular point:', dragPointIndex);
      onMovePoint(dragPointIndex, worldCoords[0], worldCoords[1]);
    }
  };
  
  // Handle mouse move on canvas with throttling
  const handleMouseMove = (e: React.MouseEvent) => {
    if (!canvasRef.current) return;
    
    const rect = canvasRef.current.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    setMousePosition([x, y]);
    
    // Handle heading editing
    if (isEditingHeading && headingEditPointIndex !== null && pathCreationMethod === 'poses') {
      // Calculate new heading based on mouse position
      const pose = poses[headingEditPointIndex];
      const [poseX, poseY] = coordToPixels([pose.x, pose.y], canvasSize);
      
      // Calculate angle between pose and mouse position
      const dx = x - poseX;
      const dy = poseY - y; // Invert y because SVG y-axis is inverted
      const angleRad = Math.atan2(dx, dy);
      let angleDeg = (angleRad * 180 / Math.PI);
      
      // Normalize angle to 0-360 range
      if (angleDeg < 0) angleDeg += 360;
      
      // Update pose heading if callback is provided
      if (onUpdatePoseHeading) {
        // Apply the update immediately
        onUpdatePoseHeading(headingEditPointIndex, angleDeg);
      }
      
      return;
    }
    
    // If not dragging, exit early
    if (!isDragging) return;
    
    // Check if we're dragging a control point
    if (dragControlPoint !== null) {
      console.log('Dragging control point in mouse move');
      // Throttle updates to 60fps (approximately 16ms between frames)
      const now = Date.now();
      if (now - lastUpdateTimeRef.current < 16) {
        if (!pendingUpdateRef.current) {
          pendingUpdateRef.current = true;
          requestAnimationFrame(() => {
            handleDragUpdate(x, y);
            pendingUpdateRef.current = false;
            lastUpdateTimeRef.current = Date.now();
          });
        }
        return;
      }
      
      // Update immediately if enough time has passed
      handleDragUpdate(x, y);
      lastUpdateTimeRef.current = now;
      return;
    }
    
    // Handle regular point dragging
    if (dragPointIndex === null) return;
    
    // Throttle updates to 60fps (approximately 16ms between frames)
    const now = Date.now();
    if (now - lastUpdateTimeRef.current < 16) {
      if (!pendingUpdateRef.current) {
        pendingUpdateRef.current = true;
        requestAnimationFrame(() => {
          handleDragUpdate(x, y);
          pendingUpdateRef.current = false;
          lastUpdateTimeRef.current = Date.now();
        });
      }
      return;
    }
    
    // Update immediately if enough time has passed
    handleDragUpdate(x, y);
    lastUpdateTimeRef.current = now;
  };

  // Handle mouse down on canvas
  const handleMouseDown = (e: React.MouseEvent) => {
    if (!canvasRef.current) return;
    
    const rect = canvasRef.current.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    
    console.log('Mouse down at:', x, y);
    
    // Check if clicking on a heading indicator for poses
    if (pathCreationMethod === 'poses' && selectedPointIndex !== null) {
      const selectedPose = poses[selectedPointIndex];
      const [poseX, poseY] = coordToPixels([selectedPose.x, selectedPose.y], canvasSize);
      
      // Calculate heading angle in radians
      const headingRad = selectedPose.heading * Math.PI / 180;
      
      // Calculate arrow endpoint
      const smallerPointRadius = pointRadius * 0.7;
      const arrowLength = smallerPointRadius * 2.2;
      const arrowX = poseX + Math.sin(headingRad) * arrowLength;
      const arrowY = poseY - Math.cos(headingRad) * arrowLength;
      
      // Check if click is near the heading indicator endpoint
      const distanceToHeadingIndicator = Math.sqrt(
        Math.pow(arrowX - x, 2) + Math.pow(arrowY - y, 2)
      );
      
      // If clicking near the heading indicator, start heading edit mode
      if (distanceToHeadingIndicator <= smallerPointRadius * 1.5) {
        setIsEditingHeading(true);
        setHeadingEditPointIndex(selectedPointIndex);
        return;
      }
    }
    
    // Check if clicking on an existing point based on path creation method
    let clickedPointIndex = -1;
    
    // Check for control points first regardless of mode if they're visible
    if (controlPointsList && controlPointsList.length > 0) {
      console.log('Checking control points, count:', controlPointsList.reduce((acc, curve) => acc + curve.control_points.length, 0));
      
      // First, check if we're clicking on a control point
      for (let i = 0; i < controlPointsList.length; i++) {
        const curve = controlPointsList[i];
        for (let j = 0; j < curve.control_points.length; j++) {
          const point = curve.control_points[j];
          const [px, py] = coordToPixels([point.x, point.y], canvasSize);
          const distance = Math.sqrt(Math.pow(px - x, 2) + Math.pow(py - y, 2));
          
          if (distance <= pointRadius * 0.8) {
            console.log('Found control point at curve', i, 'point', j);
            // Found a control point - set it for dragging
            setDragControlPoint({ curveIndex: i, pointIndex: j });
            setIsDragging(true);
            setDragPointIndex(null); // Make sure we're not trying to drag a regular point
            // Prevent adding a new point
            e.stopPropagation();
            return;
          }
        }
      }
      
      // If we have control points but didn't click on one, don't add a new point
      if (controlPointsList.length > 0) {
        console.log('Clicked on empty space with control points visible');
        return;
      }
    }
    // Handle other path creation methods
    else if (pathCreationMethod === 'poses') {
      clickedPointIndex = poses.findIndex((pose) => {
        const posePixels = coordToPixels([pose.x, pose.y], canvasSize);
        const distance = Math.sqrt(
          Math.pow(posePixels[0] - x, 2) + Math.pow(posePixels[1] - y, 2)
        );
        return distance <= pointRadius;
      });
    } else if (pathCreationMethod === 'points') {
      clickedPointIndex = points.findIndex((point) => {
        const pointPixels = coordToPixels([point.x, point.y], canvasSize);
        const distance = Math.sqrt(
          Math.pow(pointPixels[0] - x, 2) + Math.pow(pointPixels[1] - y, 2)
        );
        return distance <= pointRadius;
      });
    }
    
    if (clickedPointIndex !== -1) {
      // Clicked on an existing point
      onSelectPoint(clickedPointIndex);
      setIsDragging(true);
      // Store the index of the point being dragged
      setDragPointIndex(clickedPointIndex);
    } else if (pathCreationMethod !== 'control-points') {
      // Clicked on empty space, add a new point (only for poses and points methods)
      const worldCoords = pixelsToCoord([x, y], canvasSize);
      onAddPoint(worldCoords[0], worldCoords[1]);
    }
  };

  // Handle mouse up to stop dragging and heading editing
  const handleMouseUp = () => {
    setIsDragging(false);
    setDragPointIndex(null);
    setDragControlPoint(null);
    
    // Reset heading editing state
    setIsEditingHeading(false);
    setHeadingEditPointIndex(null);
  };

  // Draw path lines between points
  const renderPathLines = () => {
    if (!pathPoints || pathPoints.length < 2) return null;
    
    let pathD = '';
    pathPoints.forEach((point, index) => {
      const [x, y] = coordToPixels([point.x, point.y], canvasSize);
      if (index === 0) {
        pathD += `M ${x} ${y} `;
      } else {
        pathD += `L ${x} ${y} `;
      }
    });
    
    return (
      <path
        d={pathD}
        stroke="#EF4444"
        strokeWidth="3"
        fill="none"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    );
  };

  // Draw trajectory with velocity indicators
  const renderTrajectory = () => {
    if (!showTrajectory || !trajectoryPoints || trajectoryPoints.length < 2) return null;
    
    // Find max velocity for color scaling
    const maxVelocity = Math.max(...trajectoryPoints.map(p => p.velocity));
    
    return trajectoryPoints.map((point, index) => {
      if (index === trajectoryPoints.length - 1) return null;
      
      const [x1, y1] = coordToPixels([point.x, point.y], canvasSize);
      const [x2, y2] = coordToPixels([trajectoryPoints[index + 1].x, trajectoryPoints[index + 1].y], canvasSize);
      
      // Color based on velocity (green for slow, yellow for medium, red for fast)
      const velocityRatio = point.velocity / maxVelocity;
      const hue = 120 * (1 - velocityRatio); // 120 is green, 0 is red
      const color = `hsl(${hue}, 100%, 50%)`;
      
      return (
        <line
          key={`traj-${index}`}
          x1={x1}
          y1={y1}
          x2={x2}
          y2={y2}
          stroke={color}
          strokeWidth="2"
          strokeLinecap="round"
        />
      );
    });
  };

  // Render ultra-minimalist pose points with clean heading indicators
  const renderPosePoints = () => {
    if (pathCreationMethod !== 'poses') return null;
    
    return poses.map((pose, index) => {
      const [x, y] = coordToPixels([pose.x, pose.y], canvasSize);
      const isSelected = selectedPointIndex === index;
      const isEditingThisHeading = isEditingHeading && headingEditPointIndex === index;
      
      // Calculate heading angle in radians
      const headingRad = pose.heading * Math.PI / 180;
      
      // Make everything smaller
      const smallerPointRadius = pointRadius * 0.7;
      
      // Calculate arrow endpoint - make it proportional to smaller radius
      const arrowLength = smallerPointRadius * (isSelected ? 2.5 : 2.2);
      const arrowX = x + Math.sin(headingRad) * arrowLength;
      const arrowY = y - Math.cos(headingRad) * arrowLength;
      
      // Define colors for a cleaner look - more subtle
      const primaryColor = isSelected ? "#F59E0B" : "#3B82F6";
      const secondaryColor = isSelected ? "rgba(245, 158, 11, 0.5)" : "rgba(59, 130, 246, 0.5)";
      
      // Check if mouse is near the heading indicator (for hover effect)
      let isHeadingHovered = false;
      if (isSelected && mousePosition) {
        const [mx, my] = mousePosition;
        const distanceToHeadingIndicator = Math.sqrt(
          Math.pow(arrowX - mx, 2) + Math.pow(arrowY - my, 2)
        );
        isHeadingHovered = distanceToHeadingIndicator <= smallerPointRadius * 1.5;
      }
      
      return (
        <g key={`pose-${index}`}>
          {/* Heading indicator - thin, subtle line */}
          <line
            x1={x}
            y1={y}
            x2={arrowX}
            y2={arrowY}
            stroke={secondaryColor}
            strokeWidth={isSelected ? "1.2" : "1"}
            strokeLinecap="round"
            strokeDasharray={isEditingThisHeading ? "2,1" : "none"}
          />
          
          {/* Ultra-minimalist point circle */}
          <circle
            cx={x}
            cy={y}
            r={smallerPointRadius}
            fill={primaryColor}
            stroke="rgba(255, 255, 255, 0.7)"
            strokeWidth="1"
          />
          
          {/* Direction indicator with interactive styling */}
          {isSelected && (
            <circle
              cx={arrowX}
              cy={arrowY}
              r={smallerPointRadius * (isHeadingHovered || isEditingThisHeading ? 0.5 : 0.3)}
              fill={isHeadingHovered || isEditingThisHeading ? "#FFFFFF" : primaryColor}
              stroke={primaryColor}
              strokeWidth={isHeadingHovered || isEditingThisHeading ? "1" : "0"}
              style={{ cursor: isHeadingHovered ? 'grab' : 'default' }}
              opacity={isEditingThisHeading ? 0.8 : 1}
            />
          )}
          
          {/* Heading angle text - only show for selected point, smaller and more subtle */}
          {isSelected && (
            <text
              x={arrowX + smallerPointRadius * 0.8}
              y={arrowY}
              textAnchor="start"
              dominantBaseline="middle"
              fill={isEditingThisHeading ? "#FFFFFF" : "rgba(255, 255, 255, 0.9)"}
              fontSize="8"
              fontWeight={isEditingThisHeading ? "medium" : "normal"}
            >
              {Math.round(pose.heading)}°
            </text>
          )}
          
          {/* Point index - tiny and minimal */}
          <text
            x={x}
            y={y}
            textAnchor="middle"
            dominantBaseline="central"
            fill="#FFFFFF"
            fontSize="8"
            fontWeight="normal"
          >
            {index + 1}
          </text>
          
          {/* Add a hint tooltip for selected points */}
          {isSelected && !isEditingHeading && (
            <title>Drag the direction indicator to change heading</title>
          )}
        </g>
      );
    });
  };
  
  // Render simple points (without heading)
  const renderSimplePoints = () => {
    if (pathCreationMethod !== 'points') return null;
    
    return points.map((point, index) => {
      const [x, y] = coordToPixels([point.x, point.y], canvasSize);
      const isSelected = selectedPointIndex === index;
      
      return (
        <g key={`point-${index}`}>
          {/* Point outer glow (only for selected points) */}
          {isSelected && (
            <motion.circle
              cx={x}
              cy={y}
              r={pointRadius + 3}
              fill="rgba(249, 115, 22, 0.3)"
              initial={{ scale: 0.9, opacity: 0 }}
              animate={{ scale: 1, opacity: 1 }}
              transition={{ duration: 0.2 }}
            />
          )}
          
          {/* Point circle */}
          <motion.circle
            cx={x}
            cy={y}
            r={pointRadius}
            fill={isSelected ? "#F97316" : "#10B981"} // Orange when selected, Green otherwise
            stroke={isSelected ? "#FFFFFF" : "rgba(255, 255, 255, 0.8)"}
            strokeWidth={isSelected ? 1.5 : 1}
            initial={{ scale: 0.8, opacity: 0 }}
            animate={{ scale: 1, opacity: 1 }}
            whileHover={{ scale: 1.1 }}
            transition={{ duration: 0.2 }}
          />
          
          {/* Point index */}
          <text
            x={x}
            y={y}
            textAnchor="middle"
            dominantBaseline="central"
            fill="#FFFFFF"
            fontSize="10"
            fontWeight="medium"
          >
            {index + 1}
          </text>
        </g>
      );
    });
  };
  
  // Calculate bezier curve points locally to avoid API calls
  const localPathPoints = useMemo(() => {
    if (!controlPointsList || controlPointsList.length === 0) {
      return [];
    }
    return generatePathPoints(controlPointsList, 100);
  }, [controlPointsList]);

  // Render control points and bezier curves with a minimalist design
  const renderControlPoints = () => {
    if (!controlPointsList || controlPointsList.length === 0) {
      return null;
    }
    
    return (
      <g>
        {/* Render the calculated bezier curve path with subtle gradient effect */}
        <path
          d={localPathPoints.length > 0 ? `M${coordToPixels([localPathPoints[0].x, localPathPoints[0].y], canvasSize).join(',')} ${localPathPoints.slice(1).map(point => {
            const [x, y] = coordToPixels([point.x, point.y], canvasSize);
            return `L${x},${y}`;
          }).join(' ')}` : ''}
          stroke="url(#purpleGradient)"
          strokeWidth="1.5"
          strokeLinecap="round"
          fill="none"
        />
        
        {/* Define gradient for path */}
        <defs>
          <linearGradient id="purpleGradient" x1="0%" y1="0%" x2="100%" y2="0%">
            <stop offset="0%" stopColor="#9333EA" />
            <stop offset="100%" stopColor="#7C3AED" />
          </linearGradient>
        </defs>
        
        {controlPointsList.map((curve, curveIndex) => {
          const controlPoints = curve.control_points;
          
          // Draw ultra-thin lines connecting control points
          const lines = [];
          for (let i = 0; i < controlPoints.length - 1; i++) {
            const [x1, y1] = coordToPixels([controlPoints[i].x, controlPoints[i].y], canvasSize);
            const [x2, y2] = coordToPixels([controlPoints[i + 1].x, controlPoints[i + 1].y], canvasSize);
            
            lines.push(
              <line
                key={`control-line-${curveIndex}-${i}`}
                x1={x1}
                y1={y1}
                x2={x2}
                y2={y2}
                stroke="rgba(156, 163, 175, 0.4)"
                strokeWidth="0.7"
                strokeDasharray="3,2"
              />
            );
          }
          
          // Draw minimalist control points
          const points = controlPoints.map((point, pointIndex) => {
            const [x, y] = coordToPixels([point.x, point.y], canvasSize);
            const isEndpoint = pointIndex === 0 || pointIndex === controlPoints.length - 1;
            const isSelected = dragControlPoint?.curveIndex === curveIndex && dragControlPoint?.pointIndex === pointIndex;
            
            // Smaller, more subtle control points
            const radius = isEndpoint ? pointRadius * 0.45 : pointRadius * 0.3;
            
            // Subtle colors
            const fillColor = isEndpoint 
              ? isSelected ? "rgba(147, 51, 234, 0.9)" : "rgba(147, 51, 234, 0.7)" // Purple for endpoints
              : isSelected ? "rgba(107, 114, 128, 0.9)" : "rgba(107, 114, 128, 0.5)"; // Gray for control points
            
            return (
              <g key={`control-point-${curveIndex}-${pointIndex}`}>
                {/* Point */}
                <circle
                  cx={x}
                  cy={y}
                  r={radius}
                  fill={fillColor}
                  stroke={isSelected ? "#FFFFFF" : "rgba(255, 255, 255, 0.5)"}
                  strokeWidth={isSelected ? "0.8" : "0.5"}
                />
                
                {/* Subtle dot in the center for endpoints only */}
                {isEndpoint && (
                  <circle
                    cx={x}
                    cy={y}
                    r={radius * 0.3}
                    fill={isSelected ? "#FFFFFF" : "rgba(255, 255, 255, 0.7)"}
                  />
                )}
              </g>
            );
          });
          
          return (
            <g key={`control-curve-${curveIndex}`}>
              {lines}
              {points}
            </g>
          );
        })}
      </g>
    );
  };

  return (
    <div 
      ref={canvasRef}
      className="relative w-full h-full overflow-hidden bg-gray-900 cursor-crosshair"
      onMouseMove={handleMouseMove}
      onMouseDown={handleMouseDown}
      onMouseUp={handleMouseUp}
      onMouseLeave={handleMouseUp}
    >
      {/* Game field background */}
      <div 
        className="absolute inset-0 bg-center bg-no-repeat bg-contain"
        style={{ 
          backgroundImage: 'url(/images/high_stakes_field.png)',
          width: canvasSize.width,
          height: canvasSize.height
        }}
      />
      
      {/* SVG overlay for drawing paths and points */}
      <svg 
        width={canvasSize.width} 
        height={canvasSize.height} 
        className="absolute inset-0"
      >
        {/* Grid lines */}
        <defs>
          <pattern id="grid" width="50" height="50" patternUnits="userSpaceOnUse">
            <path d="M 50 0 L 0 0 0 50" fill="none" stroke="#2D3748" strokeWidth="0.5" />
          </pattern>
        </defs>
        <rect width="100%" height="100%" fill="url(#grid)" />
        
        {/* Origin marker */}
        <circle cx={canvasSize.width / 2} cy={canvasSize.height / 2} r="4" fill="#4B5563" />
        <line x1={canvasSize.width / 2 - 10} y1={canvasSize.height / 2} x2={canvasSize.width / 2 + 10} y2={canvasSize.height / 2} stroke="#4B5563" strokeWidth="1" />
        <line x1={canvasSize.width / 2} y1={canvasSize.height / 2 - 10} x2={canvasSize.width / 2} y2={canvasSize.height / 2 + 10} stroke="#4B5563" strokeWidth="1" />
        
        {/* Path lines */}
        {renderPathLines()}
        
        {/* Trajectory */}
        {renderTrajectory()}
        
        {/* Points based on creation method */}
        {controlPointsList.length === 0 && pathCreationMethod === 'poses' && renderPosePoints()}
        {controlPointsList.length === 0 && pathCreationMethod === 'points' && renderSimplePoints()}
        {controlPointsList.length > 0 && renderControlPoints()}
      </svg>
      
      {/* Mouse position indicator */}
      {mousePosition && !isDragging && (
        <div className="absolute bottom-4 right-4 bg-gray-800 bg-opacity-75 text-white text-xs px-2 py-1 rounded">
          {pixelsToCoord(mousePosition, canvasSize).map(coord => coord.toFixed(2)).join(', ')}
        </div>
      )}
    </div>
  );
};

export default PathCanvas;
