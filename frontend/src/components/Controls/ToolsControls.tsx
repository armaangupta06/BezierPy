import React from 'react';
import { FiTrash2, FiEye, FiEyeOff, FiX } from 'react-icons/fi';
import GetCodeButton from './GetCodeButton';
import CppCodeImport from '@/components/CppCodeImport';
import { PoseModel, PointModel, BezierCurveModel } from '@/services/api';

interface TrajectoryParams {
  initialVelocity: number;
  finalVelocity: number;
  maxVelocity: number;
  acceleration: number;
  deceleration: number;
  maxAngularVelocity: number;
  tangentMagnitude?: number;
  finalAngle?: number;
  reversed?: boolean;
}

interface ToolsControlsProps {
  onClearPoints: () => void;
  onDeleteSelectedPoint: () => void;
  hasSelectedPoint: boolean;
  showTrajectory: boolean;
  onToggleTrajectory: () => void;
  pathCreationMethod: 'poses' | 'points' | 'control-points';
  poses: PoseModel[];
  points: PointModel[];
  controlPointsList: BezierCurveModel[];
  initialHeading?: number;
  finalHeading?: number;
  tangentMagnitude: number;
  trajectoryParams: {
    initialVelocity: number;
    finalVelocity: number;
    maxVelocity: number;
    acceleration: number;
    deceleration: number;
    maxJerk: number;
    maxAngularVelocity: number;
    useTrapezoidalProfile: boolean;
  };
  areControlPointsEdited: boolean;
  onPointsFromCppCode?: (points: { x: number, y: number }[], params?: TrajectoryParams) => void;
  // New props for point and pose size control
  pointRadiusInInches?: number;
  poseRadiusInInches?: number;
  onPointRadiusChange?: (radius: number) => void;
  onPoseRadiusChange?: (radius: number) => void;
}

/**
 * Controls for tools and visualization options
 */
const ToolsControls: React.FC<ToolsControlsProps> = ({
  onClearPoints,
  onDeleteSelectedPoint,
  hasSelectedPoint,
  showTrajectory,
  onToggleTrajectory,
  pathCreationMethod,
  poses,
  points,
  controlPointsList,
  initialHeading,
  finalHeading,
  tangentMagnitude,
  trajectoryParams,
  areControlPointsEdited,
  onPointsFromCppCode = () => {},
  // New props for point and pose size control
  pointRadiusInInches = 14.5 / 2, // Default from coordinate-transforms.ts
  poseRadiusInInches = 10 / 2, // Default value
  onPointRadiusChange = () => {},
  onPoseRadiusChange = () => {}
}) => {
  return (
    <div className="space-y-6">
      <div className="bg-gray-900 rounded-lg p-4">
        <h3 className="text-lg font-medium text-white mb-4">Tools</h3>
        
        <div className="space-y-3">
          <button
            onClick={onClearPoints}
            className="w-full bg-gray-700 hover:bg-gray-600 text-white font-medium py-2 px-4 rounded-lg flex items-center space-x-2 transition-colors"
          >
            <FiTrash2 className="w-4 h-4" />
            <span>Clear All Points</span>
          </button>
          
          <button
            onClick={onDeleteSelectedPoint}
            disabled={!hasSelectedPoint}
            className={`w-full font-medium py-2 px-4 rounded-lg flex items-center space-x-2 transition-colors ${
              hasSelectedPoint 
                ? 'bg-red-700 hover:bg-red-600 text-white' 
                : 'bg-gray-800 text-gray-500 cursor-not-allowed'
            }`}
          >
            <FiX className="w-4 h-4" />
            <span>Delete Selected Point</span>
          </button>
        </div>
      </div>
      
      <div className="bg-gray-900 rounded-lg p-4">
        <h3 className="text-lg font-medium text-white mb-4">Visualization</h3>
        
        <div className="space-y-3">
          <button
            onClick={onToggleTrajectory}
            className="w-full bg-gray-700 hover:bg-gray-600 text-white font-medium py-2 px-4 rounded-lg flex items-center space-x-2 transition-colors"
          >
            {showTrajectory ? (
              <>
                <FiEyeOff className="w-4 h-4" />
                <span>Hide Trajectory</span>
              </>
            ) : (
              <>
                <FiEye className="w-4 h-4" />
                <span>Show Trajectory</span>
              </>
            )}
          </button>
        </div>
      </div>
      
      <div className="bg-gray-900 rounded-lg p-4">
        <h3 className="text-lg font-medium text-white mb-4">Code Generation</h3>
        
        <div className="space-y-3">
          <div className="flex gap-2">
            <div className="flex-1">
              <GetCodeButton
                pathCreationMethod={pathCreationMethod}
                poses={poses}
                points={points}
                controlPointsList={controlPointsList}
                initialHeading={initialHeading}
                finalHeading={finalHeading}
                tangentMagnitude={tangentMagnitude}
                trajectoryParams={trajectoryParams}
                areControlPointsEdited={areControlPointsEdited}
              />
            </div>
            <div className="flex-1">
              <CppCodeImport onPointsExtracted={onPointsFromCppCode} />
            </div>
          </div>
        </div>
      </div>
      
      <div className="bg-gray-900 rounded-lg p-4">
        <h3 className="text-lg font-medium text-white mb-4">Instructions</h3>
        
        <div className="space-y-2 text-sm text-gray-400">
          <p><span className="text-blue-400">Click</span> on the canvas to add a new point</p>
          <p><span className="text-blue-400">Click and drag</span> a point to move it</p>
          <p><span className="text-blue-400">Select</span> a point to edit or delete it</p>
          <p>Generate a path after adding at least 2 points</p>
          <p>Generate a trajectory after creating a path</p>
          <p>Use <span className="text-blue-400">Get Code</span> to generate C++ code</p>
        </div>
      </div>
      
      {/* Size Controls */}
      <div className="bg-gray-900 rounded-lg p-4">
        <h3 className="text-lg font-medium text-white mb-4">Size Controls</h3>
        
        <div className="space-y-4">
          {/* Point Size Slider */}
          <div>
            <div className="flex justify-between items-center mb-2">
              <label className="text-sm font-medium text-gray-400">Point Size</label>
              <span className="text-xs text-gray-500">{(pointRadiusInInches * 2).toFixed(1)}" diameter</span>
            </div>
            <input
              type="range"
              min="2"
              max="20"
              step="1"
              value={pointRadiusInInches * 2}
              onChange={(e) => onPointRadiusChange(parseFloat(e.target.value) / 2)}
              className="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer"
            />
          </div>
          
          {/* Pose Size Slider */}
          {pathCreationMethod === 'poses' && (
            <div>
              <div className="flex justify-between items-center mb-2">
                <label className="text-sm font-medium text-gray-400">Pose Size</label>
                <span className="text-xs text-gray-500">{(poseRadiusInInches * 2).toFixed(1)}" diameter</span>
              </div>
              <input
                type="range"
                min="2"
                max="20"
                step="1"
                value={poseRadiusInInches * 2}
                onChange={(e) => onPoseRadiusChange(parseFloat(e.target.value) / 2)}
                className="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer"
              />
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default ToolsControls;
