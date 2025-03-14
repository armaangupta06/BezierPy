import React from 'react';
import { FiPlay } from 'react-icons/fi';

interface PathControlsProps {
  params: {
    tangentMagnitude: number;
  };
  onChange: (params: any) => void;
  onGeneratePath: () => void;
}

/**
 * Controls for path generation parameters
 */
const PathControls: React.FC<PathControlsProps> = ({
  params,
  onChange,
  onGeneratePath
}) => {
  const handleTangentMagnitudeChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = parseFloat(e.target.value);
    onChange({ ...params, tangentMagnitude: value });
  };

  return (
    <div className="space-y-6">
      <div className="bg-gray-900 rounded-lg p-4">
        <h3 className="text-lg font-medium text-white mb-4">Path Parameters</h3>
        
        <div className="space-y-4">
          <div>
            <label className="block text-sm font-medium text-gray-400 mb-1">
              Tangent Magnitude
            </label>
            <div className="flex items-center space-x-3">
              <input
                type="range"
                min="0.1"
                max="2"
                step="0.1"
                value={params.tangentMagnitude}
                onChange={handleTangentMagnitudeChange}
                className="flex-1 h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer"
              />
              <span className="text-sm text-gray-300 w-12 text-right">
                {params.tangentMagnitude.toFixed(1)}
              </span>
            </div>
            <p className="mt-1 text-xs text-gray-500">
              Controls the curvature of the path. Higher values create smoother curves.
            </p>
          </div>
        </div>
      </div>
      
      <button
        onClick={onGeneratePath}
        className="w-full bg-blue-600 hover:bg-blue-700 text-white font-medium py-2 px-4 rounded-lg flex items-center justify-center space-x-2 transition-colors"
      >
        <FiPlay className="w-4 h-4" />
        <span>Generate Path</span>
      </button>
      
      <div className="bg-blue-900 bg-opacity-20 border border-blue-800 rounded-lg p-3">
        <p className="text-xs text-blue-300">
          Add at least two points on the canvas to generate a path. Adjust the tangent magnitude to control the curvature of the path.
        </p>
      </div>
    </div>
  );
};

export default PathControls;
