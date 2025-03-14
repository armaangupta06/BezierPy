import React, { useState, useEffect } from 'react';
import { BezierCurveModel, ControlPointModel } from '@/services/api';

interface ControlPointsEditorProps {
  controlPointsList: BezierCurveModel[];
  onChange: (controlPointsList: BezierCurveModel[]) => void;
  onGeneratePath: () => void;
}

/**
 * Component for directly editing Bezier curve control points
 */
const ControlPointsEditor: React.FC<ControlPointsEditorProps> = ({
  controlPointsList,
  onChange,
  onGeneratePath,
}) => {
  const [curves, setCurves] = useState<BezierCurveModel[]>(
    controlPointsList.length > 0 
      ? controlPointsList 
      : [createDefaultCurve()]
  );

  // Update parent component when curves change
  useEffect(() => {
    onChange(curves);
  }, [curves, onChange]);

  // Create a default curve with 6 control points
  function createDefaultCurve(): BezierCurveModel {
    return {
      control_points: [
        { x: 100, y: 100 },
        { x: 150, y: 100 },
        { x: 200, y: 100 },
        { x: 250, y: 100 },
        { x: 300, y: 100 },
        { x: 350, y: 100 },
      ]
    };
  }

  // Handle control point value change
  const handleControlPointChange = (
    curveIndex: number,
    pointIndex: number,
    field: 'x' | 'y',
    value: number
  ) => {
    const newCurves = [...curves];
    newCurves[curveIndex].control_points[pointIndex][field] = value;
    setCurves(newCurves);
  };

  // Add a new curve
  const handleAddCurve = () => {
    // If there are existing curves, create a new curve that continues from the last point
    if (curves.length > 0) {
      const lastCurve = curves[curves.length - 1];
      const lastPoint = lastCurve.control_points[5];
      
      // Create a new curve starting from the last point of the previous curve
      const newCurve: BezierCurveModel = {
        control_points: [
          { x: lastPoint.x, y: lastPoint.y },
          { x: lastPoint.x + 50, y: lastPoint.y },
          { x: lastPoint.x + 100, y: lastPoint.y },
          { x: lastPoint.x + 150, y: lastPoint.y },
          { x: lastPoint.x + 200, y: lastPoint.y },
          { x: lastPoint.x + 250, y: lastPoint.y },
        ]
      };
      
      setCurves([...curves, newCurve]);
    } else {
      setCurves([createDefaultCurve()]);
    }
  };

  // Remove a curve
  const handleRemoveCurve = (index: number) => {
    if (curves.length > 1) {
      const newCurves = [...curves];
      newCurves.splice(index, 1);
      setCurves(newCurves);
    }
  };

  return (
    <div className="space-y-4">
      <div className="flex justify-between items-center">
        <h3 className="text-sm font-semibold">Control Points Editor</h3>
        <button
          onClick={onGeneratePath}
          className="px-3 py-1 bg-blue-600 hover:bg-blue-500 rounded text-xs"
        >
          Generate Path
        </button>
      </div>
      
      <div className="space-y-6 max-h-[400px] overflow-y-auto pr-2">
        {curves.map((curve, curveIndex) => (
          <div key={curveIndex} className="p-3 bg-gray-700 bg-opacity-40 rounded-md">
            <div className="flex justify-between items-center mb-2">
              <h4 className="text-xs font-medium">Curve {curveIndex + 1}</h4>
              {curves.length > 1 && (
                <button
                  onClick={() => handleRemoveCurve(curveIndex)}
                  className="text-xs text-red-400 hover:text-red-300"
                >
                  Remove
                </button>
              )}
            </div>
            
            <div className="space-y-2">
              {curve.control_points.map((point, pointIndex) => (
                <div key={pointIndex} className="grid grid-cols-2 gap-2">
                  <div>
                    <label className="block text-xs text-gray-400 mb-1">
                      P{pointIndex} X
                    </label>
                    <input
                      type="number"
                      value={point.x}
                      onChange={(e) => 
                        handleControlPointChange(
                          curveIndex, 
                          pointIndex, 
                          'x', 
                          parseFloat(e.target.value) || 0
                        )
                      }
                      className="w-full bg-gray-800 border border-gray-600 rounded px-2 py-1 text-xs"
                    />
                  </div>
                  <div>
                    <label className="block text-xs text-gray-400 mb-1">
                      P{pointIndex} Y
                    </label>
                    <input
                      type="number"
                      value={point.y}
                      onChange={(e) => 
                        handleControlPointChange(
                          curveIndex, 
                          pointIndex, 
                          'y', 
                          parseFloat(e.target.value) || 0
                        )
                      }
                      className="w-full bg-gray-800 border border-gray-600 rounded px-2 py-1 text-xs"
                    />
                  </div>
                </div>
              ))}
            </div>
          </div>
        ))}
      </div>
      
      <button
        onClick={handleAddCurve}
        className="w-full py-2 bg-gray-700 hover:bg-gray-600 rounded text-xs flex items-center justify-center"
      >
        + Add Curve
      </button>
    </div>
  );
};

export default ControlPointsEditor;
