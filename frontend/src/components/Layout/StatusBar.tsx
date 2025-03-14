import React from 'react';

interface StatusBarProps {
  pathId?: string;
  pointCount?: number;
  trajectoryId?: string;
}

/**
 * Status bar component displaying current path and trajectory information
 */
const StatusBar: React.FC<StatusBarProps> = ({ pathId, pointCount, trajectoryId }) => {
  return (
    <div className="bg-gray-800 border-t border-gray-700 px-4 py-2 text-sm text-gray-400 flex items-center justify-between">
      <div className="flex items-center space-x-4">
        {pathId && (
          <div>
            <span className="text-gray-500">Path ID:</span>{' '}
            <span className="text-gray-300 font-mono">{pathId.substring(0, 8)}...</span>
          </div>
        )}
        
        {pointCount !== undefined && (
          <div>
            <span className="text-gray-500">Points:</span>{' '}
            <span className="text-gray-300">{pointCount}</span>
          </div>
        )}
        
        {trajectoryId && (
          <div>
            <span className="text-gray-500">Trajectory ID:</span>{' '}
            <span className="text-gray-300 font-mono">{trajectoryId.substring(0, 8)}...</span>
          </div>
        )}
      </div>
      
      <div className="text-gray-500">
        <span>Ready</span>
      </div>
    </div>
  );
};

export default StatusBar;
