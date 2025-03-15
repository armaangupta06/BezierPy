import React from 'react';

interface TrajectoryTimeDisplayProps {
  totalTime: number | null;
  trajectoryPoints?: any[];
}

const TrajectoryTimeDisplay: React.FC<TrajectoryTimeDisplayProps> = ({ 
  totalTime, 
  trajectoryPoints = [] 
}) => {
  if (totalTime === null) return null;
  
  // Calculate metadata from trajectory points
  const pointCount = trajectoryPoints.length;
  const avgVelocity = trajectoryPoints.length > 0 
    ? trajectoryPoints.reduce((sum, point) => sum + (point.velocity || 0), 0) / pointCount
    : 0;
  const maxVelocity = trajectoryPoints.length > 0
    ? Math.max(...trajectoryPoints.map(point => point.velocity || 0))
    : 0;
  const distance = totalTime * avgVelocity; // Approximate distance
  
  // Format values nicely
  const formattedTime = totalTime.toFixed(2);
  const formattedAvgVelocity = avgVelocity.toFixed(2);
  const formattedMaxVelocity = maxVelocity.toFixed(2);
  const formattedDistance = distance.toFixed(2);
  
  return (
    <div className="absolute right-4 top-1/2 -translate-y-1/2 z-10 flex flex-col gap-2">
      {/* Time Card */}
      <div className="bg-gray-800/80 backdrop-blur-sm text-white px-4 py-3 rounded-lg shadow-md border border-gray-700 w-48">
        <div className="flex items-center justify-between mb-2">
          <div className="text-xs uppercase tracking-wider text-gray-400">Trajectory Time</div>
          <div className="w-2 h-2 rounded-full bg-green-400 animate-pulse"></div>
        </div>
        <div className="flex items-baseline">
          <span className="text-2xl font-bold text-white">{formattedTime}</span>
          <span className="ml-1 text-gray-300 text-sm">seconds</span>
        </div>
      </div>
      
      {/* Velocity Card */}
      <div className="bg-gray-800/80 backdrop-blur-sm text-white px-4 py-3 rounded-lg shadow-md border border-gray-700 w-48">
        <div className="text-xs uppercase tracking-wider text-gray-400 mb-2">Velocity</div>
        <div className="grid grid-cols-2 gap-2">
          <div>
            <div className="text-xs text-gray-400">Average</div>
            <div className="text-sm font-medium">{formattedAvgVelocity} m/s</div>
          </div>
          <div>
            <div className="text-xs text-gray-400">Maximum</div>
            <div className="text-sm font-medium">{formattedMaxVelocity} m/s</div>
          </div>
        </div>
      </div>
      
      {/* Distance Card */}
      <div className="bg-gray-800/80 backdrop-blur-sm text-white px-4 py-3 rounded-lg shadow-md border border-gray-700 w-48">
        <div className="text-xs uppercase tracking-wider text-gray-400 mb-2">Distance</div>
        <div className="text-lg font-medium">{formattedDistance} meters</div>
      </div>
      
      {/* Points Card */}
      <div className="bg-gray-800/80 backdrop-blur-sm text-white px-4 py-3 rounded-lg shadow-md border border-gray-700 w-48">
        <div className="text-xs uppercase tracking-wider text-gray-400 mb-2">Data Points</div>
        <div className="text-lg font-medium">{pointCount} points</div>
      </div>
    </div>
  );
};

export default TrajectoryTimeDisplay;
