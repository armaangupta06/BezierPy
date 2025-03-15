"use client";

import React, { useState, useEffect, useRef } from 'react';
import { QueryClient, QueryClientProvider } from '@tanstack/react-query';
import MainLayout from '@/components/Layout/MainLayout';
import PathCanvas from '@/components/Canvas/PathCanvas';
import ControlPanel from '@/components/Controls/ControlPanel';
import HeadingInputModal from '@/components/Modals/HeadingInputModal';
import HeadingsInputModal from '@/components/Modals/HeadingsInputModal';
import ControlPointsEditor from '@/components/Controls/ControlPointsEditor';
import BezierScriptLoader from '@/components/BezierScriptLoader';
import TrajectoryTimeDisplay from '@/components/TrajectoryTimeDisplay';
import { PoseModel, PointModel, BezierCurveModel, PathResponse } from '@/services/api';
import usePathGeneration from '@/hooks/usePathGeneration';
import usePathFromPointsGeneration from '@/hooks/usePathFromPointsGeneration';
import usePathFromControlPointsGeneration from '@/hooks/usePathFromControlPointsGeneration';
import useTrajectoryGeneration from '@/hooks/useTrajectoryGeneration';

// Create a client
const queryClient = new QueryClient();

// Wrap the Home component with QueryClientProvider
function HomePage() {
  // State for canvas size
  const [canvasSize, setCanvasSize] = useState({ width: 800, height: 800 });
  
  // State for control panel
  const [isControlPanelOpen, setIsControlPanelOpen] = useState(true);
  
  // State for path creation method
  const [pathCreationMethod, setPathCreationMethod] = useState<'poses' | 'points' | 'control-points'>('poses');
  
  // State for poses (points with heading)
  const [poses, setPoses] = useState<PoseModel[]>([]);
  const [points, setPoints] = useState<PointModel[]>([]);
  const [controlPointsList, setControlPointsList] = useState<BezierCurveModel[]>([]);
  const [selectedPointIndex, setSelectedPointIndex] = useState<number | null>(null);
  
  // State for path and trajectory
  const [pathId, setPathId] = useState<string | null>(null);
  const [pathPoints, setPathPoints] = useState<any[]>([]);
  const [trajectoryId, setTrajectoryId] = useState<string | null>(null);
  const [trajectoryPoints, setTrajectoryPoints] = useState<any[]>([]);
  const [trajectoryTime, setTrajectoryTime] = useState<number | null>(null);
  const [showTrajectory, setShowTrajectory] = useState(false);
  
  // State for control point editing
  const [isEditingControlPoints, setIsEditingControlPoints] = useState(false);
  const [pathControlPoints, setPathControlPoints] = useState<BezierCurveModel[]>([]);
  
  // State for heading input modals
  const [isHeadingModalOpen, setIsHeadingModalOpen] = useState(false);
  const [isHeadingsModalOpen, setIsHeadingsModalOpen] = useState(false);
  const [newPointPosition, setNewPointPosition] = useState({ x: 0, y: 0 });
  
  // State for initial/final headings (for points method)
  const [initialHeading, setInitialHeading] = useState<number>(0);
  const [finalHeading, setFinalHeading] = useState<number | undefined>(undefined);
  
  // State for parameters
  const [pathParams, setPathParams] = useState({
    tangentMagnitude: 0.8,
  });
  
  const [trajectoryParams, setTrajectoryParams] = useState({
    initialVelocity: 0,
    finalVelocity: 0,
    maxVelocity: 100,
    acceleration: 100,
    deceleration: 100,
    maxJerk: 500,
    maxAngularVelocity: 180,
    useTrapezoidalProfile: true,
  });
  
  // State for control points edited flag
  const [areControlPointsEdited, setAreControlPointsEdited] = useState(false);
  
  // Resize handler
  useEffect(() => {
    const handleResize = () => {
      // Keep the canvas square and responsive
      const size = Math.min(window.innerWidth - (isControlPanelOpen ? 320 : 48), window.innerHeight - 100);
      setCanvasSize({ width: size, height: size });
    };
    
    handleResize();
    window.addEventListener('resize', handleResize);
    
    return () => {
      window.removeEventListener('resize', handleResize);
    };
  }, [isControlPanelOpen]);
  
  // Reset path and trajectory when changing creation method
  useEffect(() => {
    setPathId(null);
    setPathPoints([]);
    setTrajectoryId(null);
    setTrajectoryPoints([]);
  }, [pathCreationMethod]);
  
  // Handlers for point operations based on creation method
  const handleAddPointPosition = (x: number, y: number) => {
    setNewPointPosition({ x, y });
    
    if (pathCreationMethod === 'poses') {
      setIsHeadingModalOpen(true);
    } else if (pathCreationMethod === 'points') {
      // For points method, just add the point directly
      const newPoint: PointModel = { x, y };
      setPoints([...points, newPoint]);
      
      // If we have at least 2 points, show the headings modal
      if (points.length >= 1) {
        setIsHeadingsModalOpen(true);
      }
    }
    // For control-points method, points are added through the editor
  };
  
  const handleAddPointWithHeading = (heading: number) => {
    if (pathCreationMethod === 'poses') {
      const newPose: PoseModel = {
        x: newPointPosition.x,
        y: newPointPosition.y,
        heading: heading,
      };
      
      const updatedPoses = [...poses, newPose];
      setPoses(updatedPoses);
      setIsHeadingModalOpen(false);
      
      // Auto-generate path if we have at least 2 poses
      if (updatedPoses.length >= 2) {
        generatePathFromPoses(updatedPoses, pathParams.tangentMagnitude);
      }
    }
  };
  
  const handleSetPathHeadings = (initialHeading: number, finalHeading?: number) => {
    setInitialHeading(initialHeading);
    setFinalHeading(finalHeading);
    setIsHeadingsModalOpen(false);
    
    // Auto-generate path if we have at least 2 points
    if (pathCreationMethod === 'points' && points.length >= 2) {
      generatePathFromPoints(points, initialHeading, finalHeading, pathParams.tangentMagnitude);
    }
  };
  
  const handleMovePoint = (index: number, x: number, y: number) => {
    if (pathCreationMethod === 'poses') {
      const updatedPoses = [...poses];
      updatedPoses[index] = { ...updatedPoses[index], x, y };
      setPoses(updatedPoses);
      
      // Auto-generate path if we already have a path and at least 2 poses
      if (pathId && updatedPoses.length >= 2) {
        // Generate path immediately without debounce
        generatePathFromPoses(updatedPoses, pathParams.tangentMagnitude);
      }
    } else if (pathCreationMethod === 'points') {
      const updatedPoints = [...points];
      updatedPoints[index] = { ...updatedPoints[index], x, y };
      setPoints(updatedPoints);
      
      // Auto-generate path if we already have a path, at least 2 points, and headings are set
      if (pathId && updatedPoints.length >= 2 && initialHeading !== undefined) {
        // Generate path immediately without debounce
        generatePathFromPoints(updatedPoints, initialHeading, finalHeading, pathParams.tangentMagnitude);
      }
    }
    // For control-points method, points are moved through the editor
  };
  
  const handleUpdatePoseHeading = (index: number, heading: number) => {
    if (pathCreationMethod === 'poses' && index >= 0 && index < poses.length) {
      const updatedPoses = [...poses];
      updatedPoses[index] = { ...updatedPoses[index], heading };
      setPoses(updatedPoses);
      
      // Auto-generate path if we already have a path and at least 2 poses
      if (pathId && updatedPoses.length >= 2) {
        // Generate path immediately when heading changes
        generatePathFromPoses(updatedPoses, pathParams.tangentMagnitude);
      }
    }
  };
  
  const handleSelectPoint = (index: number) => {
    setSelectedPointIndex(index);
  };
  
  const handleDeleteSelectedPoint = () => {
    if (selectedPointIndex !== null) {
      if (pathCreationMethod === 'poses') {
        const updatedPoses = poses.filter((_, index) => index !== selectedPointIndex);
        setPoses(updatedPoses);
      } else if (pathCreationMethod === 'points') {
        const updatedPoints = points.filter((_, index) => index !== selectedPointIndex);
        setPoints(updatedPoints);
      }
      setSelectedPointIndex(null);
    }
  };
  
  const handleClearPoints = () => {
    setPoses([]);
    setPoints([]);
    setControlPointsList([]);
    setSelectedPointIndex(null);
    setPathId(null);
    setPathPoints([]);
    setTrajectoryId(null);
    setTrajectoryPoints([]);
    setInitialHeading(0);
    setFinalHeading(undefined);
    setIsEditingControlPoints(false);
    setPathControlPoints([]);
    setAreControlPointsEdited(false);
  };
  
  const handleControlPointsChange = (newControlPointsList: BezierCurveModel[]) => {
    console.log('handleControlPointsChange called with:', newControlPointsList);
    // Always update the control points list immediately for responsive UI
    if (isEditingControlPoints) {
      setPathControlPoints(newControlPointsList);
    } else {
      setControlPointsList(newControlPointsList);
    }
    
    // Update the actual points if the first or last control points were modified
    if (newControlPointsList.length > 0) {
      // If we're using poses, update the poses
      if (pathCreationMethod === 'poses' && poses.length >= 2) {
        const updatedPoses = [...poses];
        
        // Update first pose position from first control point
        if (newControlPointsList[0] && newControlPointsList[0].control_points[0]) {
          const firstPoint = newControlPointsList[0].control_points[0];
          updatedPoses[0] = {
            ...updatedPoses[0],
            x: firstPoint.x,
            y: firstPoint.y
          };
        }
        
        // Update last pose position from last control point
        const lastCurve = newControlPointsList[newControlPointsList.length - 1];
        if (lastCurve && lastCurve.control_points.length > 0) {
          const lastPoint = lastCurve.control_points[lastCurve.control_points.length - 1];
          updatedPoses[updatedPoses.length - 1] = {
            ...updatedPoses[updatedPoses.length - 1],
            x: lastPoint.x,
            y: lastPoint.y
          };
        }
        
        setPoses(updatedPoses);
      }
      // If we're using points, update the points
      else if (pathCreationMethod === 'points' && points.length >= 2) {
        const updatedPoints = [...points];
        
        // Update first point position from first control point
        if (newControlPointsList[0] && newControlPointsList[0].control_points[0]) {
          const firstPoint = newControlPointsList[0].control_points[0];
          updatedPoints[0] = {
            ...updatedPoints[0],
            x: firstPoint.x,
            y: firstPoint.y
          };
        }
        
        // Update last point position from last control point
        const lastCurve = newControlPointsList[newControlPointsList.length - 1];
        if (lastCurve && lastCurve.control_points.length > 0) {
          const lastPoint = lastCurve.control_points[lastCurve.control_points.length - 1];
          updatedPoints[updatedPoints.length - 1] = {
            ...updatedPoints[updatedPoints.length - 1],
            x: lastPoint.x,
            y: lastPoint.y
          };
        }
        
        setPoints(updatedPoints);
      }
    }
    
    // Calculate path points locally to avoid API calls during dragging
    // This will make the UI much more responsive
    try {
      // Import the generatePathPoints function from bezier-utils
      const { generatePathPoints } = require('@/utils/bezier-utils');
      
      // Generate path points locally
      const localPathPoints = generatePathPoints(newControlPointsList, 100);
      
      // Update path points state
      setPathPoints(localPathPoints);
      
      // Only make the API call when dragging stops to keep the backend in sync
      // We can add this later with a debounce function if needed
    } catch (error) {
      console.error('Error generating local path points:', error);
      
      // Fallback to API call if local calculation fails
      if (isEditingControlPoints) {
        generatePathFromControlPoints(newControlPointsList);
      }
    }
  };
  
  const handleToggleControlPointsEdit = () => {
    if (!pathId && !isEditingControlPoints) {
      alert('Please generate a path first before editing control points');
      return;
    }
    
    // Toggle editing mode
    const newEditingState = !isEditingControlPoints;
    setIsEditingControlPoints(newEditingState);
    
    if (newEditingState) {
      // Entering edit mode - initialize control points if needed
      if (pathControlPoints.length === 0 && controlPointsList.length > 0) {
        setPathControlPoints([...controlPointsList]);
      }
      
      // Clear any selected points when entering control points edit mode
      // This ensures no points are orange/in editing mode when editing control points
      setSelectedPointIndex(null);
    } else {
      // Exiting edit mode - apply the changes
      if (pathControlPoints.length > 0) {
        // Update the main control points list with the edited points
        setControlPointsList([...pathControlPoints]);
        
        // Regenerate the path with the updated control points
        generatePathFromControlPoints(pathControlPoints);
      }
    }
  };
  
  // Path generation hooks
  const { generatePath: generatePathFromPoses } = usePathGeneration({
    queryClient,
    onSuccess: async (pathId: string, points: any[], curves?: any[]) => {
      setPathId(pathId);
      setPathPoints(points);
      
      // If showTrajectory is true, automatically regenerate the trajectory
      if (showTrajectory) {
        // Keep the trajectory visible while updating
        await generateTrajectoryIfNeeded(pathId);
      } else {
        // If no trajectory is showing, just clear the trajectory data
        setTrajectoryId(null);
        setTrajectoryPoints([]);
      }
      
      // Reset control points edited flag when a new path is generated from poses
      setAreControlPointsEdited(false);
      console.log('Path generated from poses, areControlPointsEdited set to false');
      
      // Log the curves data to see its structure
      console.log('Curves data from API:', curves);
      
      // Extract control points from the curves with defensive checks
      if (curves && curves.length > 0) {
        try {
          // Check if curves have the expected structure
          const extractedControlPoints = curves.map(curve => {
            // Verify curve has the expected properties
            if (!curve || !curve.p0 || !curve.p1 || !curve.p2 || !curve.p3) {
              console.warn('Curve is missing expected properties:', curve);
              // If curve doesn't have the expected structure, try to handle it gracefully
              if (curve.control_points && Array.isArray(curve.control_points)) {
                // If it already has control_points, use them directly
                return curve;
              } else {
                // Create a placeholder with default values
                return {
                  control_points: [
                    { x: 0, y: 0 },
                    { x: 0.33, y: 0 },
                    { x: 0.66, y: 0 },
                    { x: 1, y: 0 }
                  ]
                };
              }
            }
            
            // Normal case - convert p0, p1, p2, p3 to control_points
            return {
              control_points: [
                { x: curve.p0.x, y: curve.p0.y },
                { x: curve.p1.x, y: curve.p1.y },
                { x: curve.p2.x, y: curve.p2.y },
                { x: curve.p3.x, y: curve.p3.y }
              ]
            };
          });
          
          console.log('Extracted control points:', extractedControlPoints);
          setPathControlPoints(extractedControlPoints);
          // Also update the control points list for consistency
          setControlPointsList(extractedControlPoints);
        } catch (error) {
          console.error('Error extracting control points:', error);
        }
      }
    },
    onError: (error: Error) => {
      console.error('Error generating path from poses:', error);
      alert('Failed to generate path. Please try again.');
    }
  });
  
  const { generatePath: generatePathFromPoints } = usePathFromPointsGeneration({
    queryClient,
    onSuccess: async (pathId: string, points: any[], curves?: any[]) => {
      setPathId(pathId);
      setPathPoints(points);
      
      // If showTrajectory is true, automatically regenerate the trajectory
      if (showTrajectory) {
        // Keep the trajectory visible while updating
        await generateTrajectoryIfNeeded(pathId);
      } else {
        // If no trajectory is showing, just clear the trajectory data
        setTrajectoryId(null);
        setTrajectoryPoints([]);
      }
      
      // Reset control points edited flag when a new path is generated from points
      setAreControlPointsEdited(false);
      console.log('Path generated from points, areControlPointsEdited set to false');
      
      // Log the curves data to see its structure
      console.log('Curves data from points API:', curves);
      
      // Extract control points from the curves with defensive checks
      if (curves && curves.length > 0) {
        try {
          // Check if curves have the expected structure
          const extractedControlPoints = curves.map(curve => {
            // Verify curve has the expected properties
            if (!curve || !curve.p0 || !curve.p1 || !curve.p2 || !curve.p3) {
              console.warn('Curve from points is missing expected properties:', curve);
              // If curve doesn't have the expected structure, try to handle it gracefully
              if (curve.control_points && Array.isArray(curve.control_points)) {
                // If it already has control_points, use them directly
                return curve;
              } else {
                // Create a placeholder with default values
                return {
                  control_points: [
                    { x: 0, y: 0 },
                    { x: 0.33, y: 0 },
                    { x: 0.66, y: 0 },
                    { x: 1, y: 0 }
                  ]
                };
              }
            }
            
            // Normal case - convert p0, p1, p2, p3 to control_points
            return {
              control_points: [
                { x: curve.p0.x, y: curve.p0.y },
                { x: curve.p1.x, y: curve.p1.y },
                { x: curve.p2.x, y: curve.p2.y },
                { x: curve.p3.x, y: curve.p3.y }
              ]
            };
          });
          
          console.log('Extracted control points from points:', extractedControlPoints);
          setPathControlPoints(extractedControlPoints);
          // Also update the control points list for consistency
          setControlPointsList(extractedControlPoints);
        } catch (error) {
          console.error('Error extracting control points from points:', error);
        }
      }
    },
    onError: (error: Error) => {
      console.error('Error generating path from points:', error);
      alert('Failed to generate path. Please try again.');
    }
  });
  
  const { generatePath: generatePathFromControlPoints } = usePathFromControlPointsGeneration({
    queryClient,
    onSuccess: async (pathId: string, points: any[], curves?: any[]) => {
      setPathId(pathId);
      setPathPoints(points);
      
      // If showTrajectory is true, automatically regenerate the trajectory
      if (showTrajectory) {
        // Keep the trajectory visible while updating
        await generateTrajectoryIfNeeded(pathId);
      } else {
        // If no trajectory is showing, just clear the trajectory data
        setTrajectoryId(null);
        setTrajectoryPoints([]);
      }
      
      // Note: We don't reset areControlPointsEdited here because this is called when control points are edited
      
      // Log the curves data to see its structure
      console.log('Curves data from control points API:', curves);
      
      // Store the control points if they're provided
      if (curves && curves.length > 0) {
        try {
          // Verify the structure of the curves
          const validCurves = curves.map(curve => {
            if (!curve.control_points || !Array.isArray(curve.control_points)) {
              console.warn('Curve from control points has invalid structure:', curve);
              // Try to convert if it has p0, p1, p2, p3 format
              if (curve.p0 && curve.p1 && curve.p2 && curve.p3) {
                return {
                  control_points: [
                    { x: curve.p0.x, y: curve.p0.y },
                    { x: curve.p1.x, y: curve.p1.y },
                    { x: curve.p2.x, y: curve.p2.y },
                    { x: curve.p3.x, y: curve.p3.y }
                  ]
                };
              } else {
                // Create a placeholder with default values
                return {
                  control_points: [
                    { x: 0, y: 0 },
                    { x: 0.33, y: 0 },
                    { x: 0.66, y: 0 },
                    { x: 1, y: 0 }
                  ]
                };
              }
            }
            return curve;
          });
          
          console.log('Valid control points:', validCurves);
          
          // If we're in edit mode, update the path control points
          if (isEditingControlPoints) {
            setPathControlPoints(validCurves);
          } else {
            // Otherwise update the regular control points list
            setControlPointsList(validCurves);
            // Also store in pathControlPoints for potential editing later
            setPathControlPoints(validCurves);
          }
        } catch (error) {
          console.error('Error processing control points:', error);
        }
      }
    },
    onError: (error: Error) => {
      console.error('Error generating path from control points:', error);
      alert('Failed to generate path. Please try again.');
    }
  });
  
  // Trajectory generation
  const { generateTrajectory } = useTrajectoryGeneration({
    queryClient,
    onSuccess: (trajectoryId: string, points: any[], totalTime?: number) => {
      setTrajectoryId(trajectoryId);
      setTrajectoryPoints(points);
      setTrajectoryTime(totalTime || null);
      setShowTrajectory(true);
    },
    onError: (error: Error) => {
      console.error('Error generating trajectory:', error);
      alert('Failed to generate trajectory. Please try again.');
    }
  });
  
  // Handler for path generation based on creation method
  const handleGeneratePath = async () => {
    try {
      if (pathCreationMethod === 'poses') {
        if (poses.length < 2) {
          alert('Please add at least 2 poses to generate a path');
          return;
        }
        
        await generatePathFromPoses(poses, pathParams.tangentMagnitude);
      } 
      else if (pathCreationMethod === 'points') {
        if (points.length < 2) {
          alert('Please add at least 2 points to generate a path');
          return;
        }
        
        if (initialHeading === undefined) {
          alert('Please specify an initial heading');
          setIsHeadingsModalOpen(true);
          return;
        }
        
        await generatePathFromPoints(points, initialHeading, finalHeading, pathParams.tangentMagnitude);
      } 
      else if (pathCreationMethod === 'control-points') {
        if (controlPointsList.length < 1) {
          alert('Please add at least one Bezier curve to generate a path');
          return;
        }
        
        await generatePathFromControlPoints(controlPointsList);
      }
    } catch (error: any) {
      console.error('Error generating path:', error);
      alert('Failed to generate path. Please try again.');
    }
  };
  
  // Helper function to generate trajectory if a path exists and showTrajectory is true
  const generateTrajectoryIfNeeded = async (currentPathId: string) => {
    // Only generate trajectory if showTrajectory is true (meaning a trajectory was previously generated)
    if (showTrajectory && currentPathId) {
      try {
        // Convert parameters to API format
        const apiParams = {
          initial_velocity: trajectoryParams.initialVelocity,
          final_velocity: trajectoryParams.finalVelocity,
          max_velocity: trajectoryParams.maxVelocity,
          acceleration: trajectoryParams.acceleration,
          // Ensure deceleration is negative as required by the backend
          deceleration: -Math.abs(trajectoryParams.deceleration),
          max_jerk: trajectoryParams.maxJerk,
          max_angular_velocity: trajectoryParams.maxAngularVelocity,
          use_trapezoidal: trajectoryParams.useTrapezoidalProfile,
        };
        
        // Use the hook to generate trajectory
        await generateTrajectory(currentPathId, apiParams);
      } catch (error: any) {
        console.error('Error auto-generating trajectory:', error);
        // Don't show alert for automatic regeneration to avoid disrupting the user
      }
    }
  };
  
  const handleGenerateTrajectory = async () => {
    if (!pathId) {
      alert('Please generate a path first');
      return;
    }
    
    try {
      // Convert parameters to API format
      const apiParams = {
        initial_velocity: trajectoryParams.initialVelocity,
        final_velocity: trajectoryParams.finalVelocity,
        max_velocity: trajectoryParams.maxVelocity,
        acceleration: trajectoryParams.acceleration,
        // Ensure deceleration is negative as required by the backend
        deceleration: -Math.abs(trajectoryParams.deceleration),
        max_jerk: trajectoryParams.maxJerk,
        max_angular_velocity: trajectoryParams.maxAngularVelocity,
        use_trapezoidal: trajectoryParams.useTrapezoidalProfile,
      };
      
      console.log('Sending trajectory params:', apiParams);
      
      // Use the hook to generate trajectory
      await generateTrajectory(pathId, apiParams);
    } catch (error: any) {
      console.error('Error generating trajectory:', error);
      alert('Failed to generate trajectory. Please try again.');
    }
  };
  
  // Handle closing the trajectory display
  const handleCloseTrajectory = () => {
    setShowTrajectory(false);
  };
  
  return (
    <QueryClientProvider client={queryClient}>
      <MainLayout
        statusInfo={{
          pathId: pathId || undefined,
          pointCount: pathCreationMethod === 'poses' ? poses.length : 
                     pathCreationMethod === 'points' ? points.length : 
                     controlPointsList.reduce((acc, curve) => acc + curve.control_points.length, 0),
          trajectoryId: trajectoryId || undefined,
        }}
      >
        <div className="flex flex-1 overflow-hidden">
          {/* Left sidebar for trajectory info */}
          {showTrajectory && trajectoryTime && (
            <div className="w-64 bg-gray-800 border-r border-gray-700 p-4 pt-10 flex flex-col gap-4 overflow-y-auto relative">
              {/* Close button - positioned at the top right with higher z-index */}
              <button 
                onClick={handleCloseTrajectory}
                className="absolute top-2 right-2 w-7 h-7 rounded-full bg-gray-800/80 hover:bg-gray-700 flex items-center justify-center text-gray-300 hover:text-white transition-colors z-20"
                aria-label="Close trajectory view"
              >
                <svg xmlns="http://www.w3.org/2000/svg" className="h-5 w-5" viewBox="0 0 20 20" fill="currentColor">
                  <path fillRule="evenodd" d="M4.293 4.293a1 1 0 011.414 0L10 8.586l4.293-4.293a1 1 0 111.414 1.414L11.414 10l4.293 4.293a1 1 0 01-1.414 1.414L10 11.414l-4.293 4.293a1 1 0 01-1.414-1.414L8.586 10 4.293 5.707a1 1 0 010-1.414z" clipRule="evenodd" />
                </svg>
              </button>
              {/* Time Card */}
              <div className="bg-gray-900/80 backdrop-blur-sm text-white px-4 py-3 rounded-lg shadow-md border border-gray-700">
                <div className="flex items-center justify-between mb-2">
                  <div className="text-xs uppercase tracking-wider text-gray-400">Trajectory Time</div>
                  <div className="w-2 h-2 rounded-full bg-green-400 animate-pulse"></div>
                </div>
                <div className="flex items-baseline">
                  <span className="text-2xl font-bold text-white">{trajectoryTime.toFixed(2)}</span>
                  <span className="ml-1 text-gray-300 text-sm">seconds</span>
                </div>
              </div>
              
              {/* Velocity Card */}
              <div className="bg-gray-900/80 backdrop-blur-sm text-white px-4 py-3 rounded-lg shadow-md border border-gray-700">
                <div className="text-xs uppercase tracking-wider text-gray-400 mb-2">Velocity</div>
                <div className="grid grid-cols-2 gap-2">
                  <div>
                    <div className="text-xs text-gray-400">Initial</div>
                    <div className="text-sm font-medium">{trajectoryParams.initialVelocity} m/s</div>
                  </div>
                  <div>
                    <div className="text-xs text-gray-400">Final</div>
                    <div className="text-sm font-medium">{trajectoryParams.finalVelocity} m/s</div>
                  </div>
                  <div>
                    <div className="text-xs text-gray-400">Maximum</div>
                    <div className="text-sm font-medium">{trajectoryParams.maxVelocity} m/s</div>
                  </div>
                </div>
              </div>
              
              {/* Acceleration Card */}
              <div className="bg-gray-900/80 backdrop-blur-sm text-white px-4 py-3 rounded-lg shadow-md border border-gray-700">
                <div className="text-xs uppercase tracking-wider text-gray-400 mb-2">Acceleration</div>
                <div className="grid grid-cols-2 gap-2">
                  <div>
                    <div className="text-xs text-gray-400">Acceleration</div>
                    <div className="text-sm font-medium">{trajectoryParams.acceleration} m/s²</div>
                  </div>
                  <div>
                    <div className="text-xs text-gray-400">Deceleration</div>
                    <div className="text-sm font-medium">{trajectoryParams.deceleration} m/s²</div>
                  </div>
                </div>
              </div>
              
              {/* Points Card */}
              <div className="bg-gray-900/80 backdrop-blur-sm text-white px-4 py-3 rounded-lg shadow-md border border-gray-700">
                <div className="text-xs uppercase tracking-wider text-gray-400 mb-2">Data Points</div>
                <div className="text-lg font-medium">{trajectoryPoints.length} points</div>
              </div>
              
              {/* Profile Type */}
              <div className="bg-gray-900/80 backdrop-blur-sm text-white px-4 py-3 rounded-lg shadow-md border border-gray-700">
                <div className="text-xs uppercase tracking-wider text-gray-400 mb-2">Profile Type</div>
                <div className="text-md font-medium">
                  {trajectoryParams.useTrapezoidalProfile ? 'Trapezoidal' : 'S-Curve'}
                </div>
              </div>
            </div>
          )}
          
          {/* Main canvas area */}
          <div className="flex-1 flex items-center justify-center bg-gray-900 p-4 relative overflow-hidden">
            <PathCanvas
              poses={pathCreationMethod === 'poses' ? poses : []}
              points={pathCreationMethod === 'points' ? points : []}
              controlPointsList={isEditingControlPoints ? pathControlPoints : (pathCreationMethod === 'control-points' ? controlPointsList : [])}
              pathPoints={pathPoints}
              trajectoryPoints={trajectoryPoints}
              onAddPoint={handleAddPointPosition}
              onMovePoint={handleMovePoint}
              onSelectPoint={handleSelectPoint}
              onUpdatePoseHeading={handleUpdatePoseHeading}
              selectedPointIndex={selectedPointIndex}
              showTrajectory={showTrajectory}
              canvasSize={canvasSize}
              pathCreationMethod={pathCreationMethod}
              onControlPointsChange={handleControlPointsChange}
            />
          </div>
          
          {/* Control panel */}
          <ControlPanel
            isOpen={isControlPanelOpen}
            onToggle={() => setIsControlPanelOpen(!isControlPanelOpen)}
            pathParams={pathParams}
            trajectoryParams={trajectoryParams}
            onPathParamsChange={(newParams) => {
              setPathParams(newParams);
              console.log("made false");
              // When waypoints or related parameters change, reset control points edited flag
              setAreControlPointsEdited(false);
              
              // Automatically regenerate path when tangent magnitude changes
              if (newParams.tangentMagnitude !== pathParams.tangentMagnitude && pathId) {
                // Use setTimeout to avoid state update conflicts
                setTimeout(() => {
                  if (pathCreationMethod === 'poses' && poses.length >= 2) {
                    generatePathFromPoses(poses, newParams.tangentMagnitude);
                  } else if (pathCreationMethod === 'points' && points.length >= 2 && initialHeading !== undefined) {
                    generatePathFromPoints(points, initialHeading, finalHeading, newParams.tangentMagnitude);
                  }
                }, 0);
              }
            }}
            onTrajectoryParamsChange={setTrajectoryParams}
            onGeneratePath={handleGeneratePath}
            onGenerateTrajectory={handleGenerateTrajectory}
            onClearPoints={handleClearPoints}
            onDeleteSelectedPoint={handleDeleteSelectedPoint}
            hasSelectedPoint={selectedPointIndex !== null}
            showTrajectory={showTrajectory}
            onToggleTrajectory={() => setShowTrajectory(!showTrajectory)}
            pathCreationMethod={pathCreationMethod}
            onPathCreationMethodChange={(method) => {
              setPathCreationMethod(method);
              setIsEditingControlPoints(false);
              console.log("made false");
              // Reset control points flag when switching mode
              setAreControlPointsEdited(false);
            }}
            isEditingControlPoints={isEditingControlPoints}
            onToggleControlPointsEdit={() => {
              console.log('Before toggle, areControlPointsEdited:', areControlPointsEdited);
              handleToggleControlPointsEdit();
              setAreControlPointsEdited(true);
              console.log('After toggle, areControlPointsEdited set to true');
            }}
            hasGeneratedPath={pathId !== null}
            // Pass data for code generation
            poses={poses}
            points={points}
            controlPointsList={controlPointsList}
            initialHeading={initialHeading}
            finalHeading={finalHeading}
            areControlPointsEdited={areControlPointsEdited}
          />
        </div>
        
        {/* Heading input modal for poses */}
        <HeadingInputModal
          isOpen={isHeadingModalOpen}
          onClose={() => setIsHeadingModalOpen(false)}
          onConfirm={handleAddPointWithHeading}
          position={newPointPosition}
        />
        
        {/* Headings input modal for points */}
        <HeadingsInputModal
          isOpen={isHeadingsModalOpen}
          onClose={() => setIsHeadingsModalOpen(false)}
          onConfirm={handleSetPathHeadings}
          initialHeading={initialHeading}
          finalHeading={finalHeading}
        />
        
        {/* Control points editor (rendered conditionally inside the control panel) */}
        {pathCreationMethod === 'control-points' && isControlPanelOpen && (
          <div className="fixed right-0 top-0 bottom-0 w-80 bg-gray-800 border-l border-gray-700 z-10 overflow-auto p-4">
            <ControlPointsEditor
              controlPointsList={controlPointsList}
              onChange={handleControlPointsChange}
              onGeneratePath={handleGeneratePath}
            />
          </div>
        )}
      </MainLayout>
    </QueryClientProvider>
  );
}

// Export the wrapped component
export default function Home() {
  return (
    <QueryClientProvider client={queryClient}>
      <BezierScriptLoader
        onLoad={() => console.log('Bezier scripts loaded successfully')}
        onError={(error) => console.error('Error loading Bezier scripts:', error)}
      >
        <HomePage />
      </BezierScriptLoader>
    </QueryClientProvider>
  );
}
