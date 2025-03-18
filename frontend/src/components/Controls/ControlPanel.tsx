import React, { useState } from 'react';
import { motion } from 'framer-motion';
import { FiChevronLeft, FiChevronRight } from 'react-icons/fi';
import PathControls from '@/components/Controls/PathControls';
import TrajectoryControls from '@/components/Controls/TrajectoryControls';
import ToolsControls from '@/components/Controls/ToolsControls';
import { PoseModel, PointModel, BezierCurveModel } from '@/services/api';

interface ControlPanelProps {
  isOpen: boolean;
  onToggle: () => void;
  pathParams: {
    tangentMagnitude: number;
  };
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
  onPathParamsChange: (params: any) => void;
  onTrajectoryParamsChange: (params: any) => void;
  onGeneratePath: () => void;
  onGenerateTrajectory: () => void;
  onClearPoints: () => void;
  onDeleteSelectedPoint: () => void;
  hasSelectedPoint: boolean;
  showTrajectory: boolean;
  onToggleTrajectory: () => void;
  pathCreationMethod?: 'poses' | 'points' | 'control-points';
  onPathCreationMethodChange?: (method: 'poses' | 'points' | 'control-points') => void;
  isEditingControlPoints?: boolean;
  onToggleControlPointsEdit?: () => void;
  hasGeneratedPath?: boolean;
  areControlPointsEdited: boolean;
  // Additional props for code generation
  poses: PoseModel[];
  points: PointModel[];
  controlPointsList: BezierCurveModel[];
  initialHeading?: number;
  finalHeading?: number;
  onPointsFromCppCode?: (points: { x: number, y: number }[]) => void;
  // Props for point and pose size control
  pointRadiusInInches?: number;
  poseRadiusInInches?: number;
  onPointRadiusChange?: (radius: number) => void;
  onPoseRadiusChange?: (radius: number) => void;
}

/**
 * Collapsible control panel with tabs for different parameter sets
 */
const ControlPanel: React.FC<ControlPanelProps> = ({
  isOpen,
  onToggle,
  pathParams,
  trajectoryParams,
  onPathParamsChange,
  onTrajectoryParamsChange,
  onGeneratePath,
  onGenerateTrajectory,
  onClearPoints,
  onDeleteSelectedPoint,
  hasSelectedPoint,
  showTrajectory,
  onToggleTrajectory,
  pathCreationMethod = 'poses',
  onPathCreationMethodChange = () => {},
  isEditingControlPoints = false,
  onToggleControlPointsEdit = () => {},
  hasGeneratedPath = false,
  areControlPointsEdited,
  poses = [],
  points = [],
  controlPointsList = [],
  initialHeading,
  finalHeading,
  onPointsFromCppCode,
  pointRadiusInInches,
  poseRadiusInInches,
  onPointRadiusChange = () => {},
  onPoseRadiusChange = () => {}
}) => {
  const [activeTab, setActiveTab] = useState<'tools' | 'path' | 'trajectory'>('tools');
  const [pathSubTab, setPathSubTab] = useState<'params' | 'creation-method'>('params');

  return (
    <motion.div 
      className="h-full bg-gray-800 border-l border-gray-700 flex flex-col"
      initial={{ width: isOpen ? 320 : 48 }}
      animate={{ width: isOpen ? 320 : 48 }}
      transition={{ duration: 0.3, ease: 'easeInOut' }}
    >
      {/* Toggle button */}
      <button 
        onClick={onToggle}
        className="absolute top-4 -left-4 bg-gray-800 border border-gray-700 rounded-l-md p-1 text-gray-400 hover:text-white z-10"
      >
        {isOpen ? <FiChevronRight /> : <FiChevronLeft />}
      </button>

      {isOpen && (
        <>
          {/* Tabs */}
          <div className="flex border-b border-gray-700">
            <button
              className={`flex-1 py-3 text-sm font-medium ${
                activeTab === 'tools' 
                  ? 'text-blue-400 border-b-2 border-blue-400' 
                  : 'text-gray-400 hover:text-white'
              }`}
              onClick={() => setActiveTab('tools')}
            >
              Tools
            </button>
            <button
              className={`flex-1 py-3 text-sm font-medium ${
                activeTab === 'path' 
                  ? 'text-blue-400 border-b-2 border-blue-400' 
                  : 'text-gray-400 hover:text-white'
              }`}
              onClick={() => setActiveTab('path')}
            >
              Path
            </button>
            <button
              className={`flex-1 py-3 text-sm font-medium ${
                activeTab === 'trajectory' 
                  ? 'text-blue-400 border-b-2 border-blue-400' 
                  : 'text-gray-400 hover:text-white'
              }`}
              onClick={() => setActiveTab('trajectory')}
            >
              Trajectory
            </button>
          </div>

          {/* Tab content */}
          <div className="flex-1 overflow-y-auto">
            {activeTab === 'tools' && (
              <div className="p-4">
                <ToolsControls 
                  onClearPoints={onClearPoints}
                  onDeleteSelectedPoint={onDeleteSelectedPoint}
                  hasSelectedPoint={hasSelectedPoint}
                  showTrajectory={showTrajectory}
                  onToggleTrajectory={onToggleTrajectory}
                  pathCreationMethod={pathCreationMethod}
                  poses={poses}
                  points={points}
                  controlPointsList={controlPointsList}
                  initialHeading={initialHeading}
                  finalHeading={finalHeading}
                  tangentMagnitude={pathParams.tangentMagnitude}
                  trajectoryParams={trajectoryParams}
                  areControlPointsEdited={areControlPointsEdited}
                  onPointsFromCppCode={onPointsFromCppCode}
                  pointRadiusInInches={pointRadiusInInches}
                  poseRadiusInInches={poseRadiusInInches}
                  onPointRadiusChange={onPointRadiusChange}
                  onPoseRadiusChange={onPoseRadiusChange}
                />
              </div>
            )}
            
            {activeTab === 'path' && (
              <div>
                {/* Path sub-tabs */}
                <div className="flex border-b border-gray-700">
                  <button
                    className={`flex-1 py-2 text-xs font-medium ${
                      pathSubTab === 'params' 
                        ? 'text-blue-400 border-b-2 border-blue-400' 
                        : 'text-gray-400 hover:text-white'
                    }`}
                    onClick={() => setPathSubTab('params')}
                  >
                    Parameters
                  </button>
                  <button
                    className={`flex-1 py-2 text-xs font-medium ${
                      pathSubTab === 'creation-method' 
                        ? 'text-blue-400 border-b-2 border-blue-400' 
                        : 'text-gray-400 hover:text-white'
                    }`}
                    onClick={() => setPathSubTab('creation-method')}
                  >
                    Creation Method
                  </button>
                </div>
                
                {/* Path sub-tab content */}
                <div className="p-4">
                  {pathSubTab === 'params' && (
                    <PathControls 
                      params={pathParams}
                      onChange={onPathParamsChange}
                      onGeneratePath={onGeneratePath}
                    />
                  )}
                  
                  {pathSubTab === 'creation-method' && (
                    <div className="space-y-4">
                      <h3 className="text-sm font-semibold mb-2">Path Creation Method</h3>
                      
                      <div className="space-y-3">
                        <div className="flex items-center space-x-2">
                          <input
                            type="radio"
                            id="poses-method"
                            name="path-method"
                            checked={pathCreationMethod === 'poses'}
                            onChange={() => onPathCreationMethodChange?.('poses')}
                            className="text-blue-500 focus:ring-blue-500"
                          />
                          <label htmlFor="poses-method" className="text-sm">
                            From Poses (Points with Heading)
                          </label>
                        </div>
                        
                        <div className="flex items-center space-x-2">
                          <input
                            type="radio"
                            id="points-method"
                            name="path-method"
                            checked={pathCreationMethod === 'points'}
                            onChange={() => onPathCreationMethodChange?.('points')}
                            className="text-blue-500 focus:ring-blue-500"
                          />
                          <label htmlFor="points-method" className="text-sm">
                            From Points (with Initial/Final Heading)
                          </label>
                        </div>
                        
                        <div className="flex items-center space-x-2">
                          <input
                            type="radio"
                            id="control-points-method"
                            name="path-method"
                            checked={pathCreationMethod === 'control-points'}
                            onChange={() => onPathCreationMethodChange?.('control-points')}
                            className="text-blue-500 focus:ring-blue-500"
                          />
                          <label htmlFor="control-points-method" className="text-sm">
                            From Control Points (Direct Editing)
                          </label>
                        </div>
                      </div>
                      
                      <div className="mt-4 p-3 bg-gray-700 bg-opacity-30 rounded-md">
                        <p className="text-xs text-gray-300">
                          {pathCreationMethod === 'poses' && 'Add points with heading to create a smooth path through them.'}
                          {pathCreationMethod === 'points' && 'Add points and specify initial/final heading to create a path.'}
                          {pathCreationMethod === 'control-points' && 'Directly edit the control points of Bezier curves.'}
                        </p>
                      </div>
                      
                      {hasGeneratedPath && (
                        <div className="mt-4">
                          <button
                            className={`px-3 py-2 rounded w-full ${isEditingControlPoints ? 'bg-purple-500 text-white' : 'bg-gray-700 text-gray-300'}`}
                            onClick={onToggleControlPointsEdit}
                            disabled={pathCreationMethod === 'control-points'}
                          >
                            {isEditingControlPoints ? 'Finish Editing Control Points' : 'Edit Control Points'}
                          </button>
                          {isEditingControlPoints && (
                            <p className="mt-2 text-xs text-gray-500">
                              Drag the control points to adjust the curve shape.
                            </p>
                          )}
                        </div>
                      )}
                    </div>
                  )}
                </div>
              </div>
            )}
            
            {activeTab === 'trajectory' && (
              <div className="p-4">
                <TrajectoryControls 
                  params={trajectoryParams}
                  onChange={onTrajectoryParamsChange}
                  onGenerateTrajectory={onGenerateTrajectory}
                />
              </div>
            )}
          </div>
        </>
      )}
    </motion.div>
  );
};

export default ControlPanel;
