import { useState } from 'react';
import { FiCode, FiCheck, FiCopy } from 'react-icons/fi';
import { generateMotionProfileCode } from '@/utils/code-generator';
import { PoseModel, PointModel, BezierCurveModel } from '@/services/api';

interface GetCodeButtonProps {
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
 * Button to generate and copy C++ code for motion profiling
 */
const GetCodeButton: React.FC<GetCodeButtonProps> = ({
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
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [copied, setCopied] = useState(false);
  
  const handleGetCode = () => {
    setIsModalOpen(true);
    setCopied(false);
  };
  
  const handleCopyCode = () => {
    console.log("EDITED");
    console.log(areControlPointsEdited);
    const code = generateMotionProfileCode(
      pathCreationMethod,
      poses,
      points,
      controlPointsList,
      initialHeading,
      finalHeading,
      tangentMagnitude,
      trajectoryParams,
      false, // reversed
      areControlPointsEdited
    );
    
    navigator.clipboard.writeText(code)
      .then(() => {
        setCopied(true);
        setTimeout(() => setCopied(false), 2000);
      })
      .catch(err => {
        console.error('Failed to copy code: ', err);
      });
  };
  
  return (
    <>
      <button
        onClick={handleGetCode}
        className="flex items-center justify-center gap-2 bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-md transition-colors"
      >
        <FiCode className="text-lg" />
        <span>Get Code</span>
      </button>
      
      {isModalOpen && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
          <div className="bg-gray-800 rounded-lg shadow-xl w-full max-w-3xl max-h-[80vh] flex flex-col">
            <div className="flex justify-between items-center p-4 border-b border-gray-700">
              <h2 className="text-xl font-semibold text-white">Generated C++ Code</h2>
              <button 
                onClick={() => setIsModalOpen(false)}
                className="text-gray-400 hover:text-white"
              >
                &times;
              </button>
            </div>
            
            <div className="flex-1 overflow-auto p-4">
              <pre className="bg-gray-900 p-4 rounded-md text-green-400 text-sm overflow-x-auto whitespace-pre">
                {generateMotionProfileCode(
                  pathCreationMethod,
                  poses,
                  points,
                  controlPointsList,
                  initialHeading,
                  finalHeading,
                  tangentMagnitude,
                  trajectoryParams,
                  false, // reversed
                  areControlPointsEdited
                )}
              </pre>
            </div>
            
            <div className="p-4 border-t border-gray-700 flex justify-end">
              <button
                onClick={handleCopyCode}
                className={`flex items-center gap-2 px-4 py-2 rounded-md transition-colors ${
                  copied ? 'bg-green-600' : 'bg-blue-600 hover:bg-blue-700'
                } text-white`}
              >
                {copied ? (
                  <>
                    <FiCheck className="text-lg" />
                    <span>Copied!</span>
                  </>
                ) : (
                  <>
                    <FiCopy className="text-lg" />
                    <span>Copy to Clipboard</span>
                  </>
                )}
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default GetCodeButton;
