import React from 'react';
import { FiPlay } from 'react-icons/fi';

interface TrajectoryControlsProps {
  params: {
    initialVelocity: number;
    finalVelocity: number;
    maxVelocity: number;
    acceleration: number;
    deceleration: number;
    maxJerk: number;
    maxAngularVelocity: number;
    useTrapezoidalProfile: boolean;
  };
  onChange: (params: any) => void;
  onGenerateTrajectory: () => void;
}

/**
 * Controls for trajectory generation parameters
 */
const TrajectoryControls: React.FC<TrajectoryControlsProps> = ({
  params,
  onChange,
  onGenerateTrajectory
}) => {
  // Handler for numeric input changes
  const handleNumberChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    onChange({ ...params, [name]: parseFloat(value) });
  };

  // Handler for checkbox changes
  const handleCheckboxChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, checked } = e.target;
    onChange({ ...params, [name]: checked });
  };

  return (
    <div className="space-y-6">
      <div className="bg-gray-900 rounded-lg p-4">
        <h3 className="text-lg font-medium text-white mb-4">Trajectory Parameters</h3>
        
        <div className="space-y-4">
          {/* Velocity Parameters */}
          <div>
            <h4 className="text-sm font-medium text-gray-300 mb-2">Velocity</h4>
            
            <div className="grid grid-cols-2 gap-3">
              <div>
                <label className="block text-xs font-medium text-gray-400 mb-1">
                  Initial (in/s)
                </label>
                <input
                  type="number"
                  name="initialVelocity"
                  value={params.initialVelocity}
                  onChange={handleNumberChange}
                  min="0"
                  step="1"
                  className="w-full bg-gray-700 border border-gray-600 rounded-md px-3 py-1.5 text-sm text-white"
                />
              </div>
              
              <div>
                <label className="block text-xs font-medium text-gray-400 mb-1">
                  Final (in/s)
                </label>
                <input
                  type="number"
                  name="finalVelocity"
                  value={params.finalVelocity}
                  onChange={handleNumberChange}
                  min="0"
                  step="1"
                  className="w-full bg-gray-700 border border-gray-600 rounded-md px-3 py-1.5 text-sm text-white"
                />
              </div>
              
              <div>
                <label className="block text-xs font-medium text-gray-400 mb-1">
                  Maximum (in/s)
                </label>
                <input
                  type="number"
                  name="maxVelocity"
                  value={params.maxVelocity}
                  onChange={handleNumberChange}
                  min="0"
                  step="10"
                  className="w-full bg-gray-700 border border-gray-600 rounded-md px-3 py-1.5 text-sm text-white"
                />
              </div>
              
              <div>
                <label className="block text-xs font-medium text-gray-400 mb-1">
                  Max Angular (deg/s)
                </label>
                <input
                  type="number"
                  name="maxAngularVelocity"
                  value={params.maxAngularVelocity}
                  onChange={handleNumberChange}
                  min="0"
                  step="10"
                  className="w-full bg-gray-700 border border-gray-600 rounded-md px-3 py-1.5 text-sm text-white"
                />
              </div>
            </div>
          </div>
          
          {/* Acceleration Parameters */}
          <div>
            <h4 className="text-sm font-medium text-gray-300 mb-2">Acceleration</h4>
            
            <div className="grid grid-cols-2 gap-3">
              <div>
                <label className="block text-xs font-medium text-gray-400 mb-1">
                  Acceleration (in/s²)
                </label>
                <input
                  type="number"
                  name="acceleration"
                  value={params.acceleration}
                  onChange={handleNumberChange}
                  min="0"
                  step="10"
                  className="w-full bg-gray-700 border border-gray-600 rounded-md px-3 py-1.5 text-sm text-white"
                />
              </div>
              
              <div>
                <label className="block text-xs font-medium text-gray-400 mb-1">
                  Deceleration (in/s²)
                </label>
                <input
                  type="number"
                  name="deceleration"
                  value={params.deceleration}
                  onChange={handleNumberChange}
                  min="0"
                  step="10"
                  className="w-full bg-gray-700 border border-gray-600 rounded-md px-3 py-1.5 text-sm text-white"
                />
                <p className="mt-1 text-xs text-gray-500">
                  Enter as positive value
                </p>
              </div>
              
              <div>
                <label className="block text-xs font-medium text-gray-400 mb-1">
                  Max Jerk (in/s³)
                </label>
                <input
                  type="number"
                  name="maxJerk"
                  value={params.maxJerk}
                  onChange={handleNumberChange}
                  min="0"
                  step="100"
                  className="w-full bg-gray-700 border border-gray-600 rounded-md px-3 py-1.5 text-sm text-white"
                />
              </div>
            </div>
          </div>
          
          {/* Profile Type */}
          <div className="flex items-center">
            <input
              type="checkbox"
              id="useTrapezoidalProfile"
              name="useTrapezoidalProfile"
              checked={params.useTrapezoidalProfile}
              onChange={handleCheckboxChange}
              className="h-4 w-4 text-blue-600 rounded border-gray-600 bg-gray-700 focus:ring-blue-500"
            />
            <label htmlFor="useTrapezoidalProfile" className="ml-2 text-sm text-gray-300">
              Use Trapezoidal Velocity Profile
            </label>
          </div>
        </div>
      </div>
      
      <button
        onClick={onGenerateTrajectory}
        className="w-full bg-blue-600 hover:bg-blue-700 text-white font-medium py-2 px-4 rounded-lg flex items-center justify-center space-x-2 transition-colors"
      >
        <FiPlay className="w-4 h-4" />
        <span>Generate Trajectory</span>
      </button>
      
      <div className="bg-blue-900 bg-opacity-20 border border-blue-800 rounded-lg p-3">
        <p className="text-xs text-blue-300">
          Generate a path first, then adjust the trajectory parameters to control the robot's motion along the path.
        </p>
      </div>
    </div>
  );
};

export default TrajectoryControls;
