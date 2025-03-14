import React from 'react';
import { FiTrash2, FiEye, FiEyeOff, FiX } from 'react-icons/fi';
import GetCodeButton from './GetCodeButton';
import { PoseModel, PointModel, BezierCurveModel } from '@/services/api';

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
  areControlPointsEdited
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
    </div>
  );
};

export default ToolsControls;
