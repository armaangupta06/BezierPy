import React, { useState, useEffect } from 'react';
import { motion } from 'framer-motion';
import { FiX } from 'react-icons/fi';
import { Point } from '@/Point';

interface TrajectoryParams {
  initialVelocity: number;
  finalVelocity: number;
  maxVelocity: number;
  acceleration: number;
  deceleration: number;
  maxAngularVelocity: number;
  tangentMagnitude?: number;
  finalAngle?: number;
  initialHeading?: number;
  reversed?: boolean;
  // Flags to indicate what type of path we found
  hasPoses?: boolean;
  hasControlPoints?: boolean;
  // Store all headings for poses
  poseHeadings?: number[];
}

interface CppCodeImportProps {
  onPointsExtracted: (points: { x: number, y: number }[], params?: TrajectoryParams) => void;
}

const CppCodeImport: React.FC<CppCodeImportProps> = ({ onPointsExtracted }) => {
  const [cppCode, setCppCode] = useState('');
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [error, setError] = useState<string>('');

  // Reset error when modal opens
  useEffect(() => {
    if (isModalOpen) {
      setError('');
    }
  }, [isModalOpen]);

  // Handle keyboard shortcuts
  useEffect(() => {
    if (!isModalOpen) return;
    
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        setIsModalOpen(false);
      } else if (e.key === 'Enter' && e.ctrlKey) {
        parsePoints();
      }
    };
    
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isModalOpen, cppCode]);

  // Parse C++ code to extract points and trajectory parameters
  const parsePoints = () => {
    try {
      setError('');
      
      // Preprocess the code to handle multi-line input
      const normalizedCode = cppCode
        .replace(/\/\/.*$/gm, '') // Remove single-line comments
        .replace(/\/\*[\s\S]*?\*\//g, '') // Remove multi-line comments
        .replace(/\s+/g, ' ') // Normalize whitespace
        .trim();
      
      console.log('Normalized code:', normalizedCode);
      
      // Regular expressions to match different C++ point formats
      const pointRegex = /\{\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\}/g;
      const pointConstructorRegex = /Point\s*\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)/gi; // Case insensitive
      const poseConstructorRegex = /Pose\s*\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)/gi; // Case insensitive
      const pointVarRegex = /\.\s*x\s*=\s*(-?\d+\.?\d*)\s*;\s*\w+\.\s*y\s*=\s*(-?\d+\.?\d*)/g;
      
      // Regex for Quintic Bezier control points - handles both direct coordinates and Point objects
      const bezierRegex = /Quintic_Bezier\s*\(\s*(?:Point\s*\(\s*)?(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)(?:\s*\))?\s*,\s*(?:Point\s*\(\s*)?(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)(?:\s*\))?\s*,\s*(?:Point\s*\(\s*)?(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)(?:\s*\))?\s*,\s*(?:Point\s*\(\s*)?(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)(?:\s*\))?\s*,\s*(?:Point\s*\(\s*)?(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)(?:\s*\))?\s*,\s*(?:Point\s*\(\s*)?(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)(?:\s*\))?\s*\)/gi;
      
      const points: { x: number, y: number }[] = [];
      const headings: number[] = [];
      const controlPoints: { x: number, y: number }[][] = [];
      let foundPoses = false;
      let foundControlPoints = false;
      
      // Match {x, y} format
      let match;
      while ((match = pointRegex.exec(cppCode)) !== null) {
        points.push({
          x: parseFloat(match[1]),
          y: parseFloat(match[2])
        });
      }
      
      // Match Point(x, y) format - case insensitive to catch "point" as well
      while ((match = pointConstructorRegex.exec(cppCode)) !== null) {
        points.push({
          x: parseFloat(match[1]),
          y: parseFloat(match[2])
        });
      }
      
      // Match Pose(x, y, theta) format - extract both point and heading
      while ((match = poseConstructorRegex.exec(cppCode)) !== null) {
        points.push({
          x: parseFloat(match[1]),
          y: parseFloat(match[2])
        });
        headings.push(parseFloat(match[3])); // Store the heading angle
        foundPoses = true; // Mark that we found poses
      }
      
      // Match Quintic_Bezier control points
      while ((match = bezierRegex.exec(cppCode)) !== null) {
        // Each match contains 6 control points (x1,y1,x2,y2,...,x6,y6)
        const curvePoints = [];
        
        // Process all 6 control points
        // The regex captures x,y pairs, so we need to step through the matches
        let validPoints = true;
        for (let i = 0; i < 6; i++) {
          const xIndex = i*2+1;
          const yIndex = i*2+2;
          
          // Ensure both x and y values are valid numbers
          if (match[xIndex] && match[yIndex] && 
              !isNaN(parseFloat(match[xIndex])) && 
              !isNaN(parseFloat(match[yIndex]))) {
            
            curvePoints.push({
              x: parseFloat(match[xIndex]),
              y: parseFloat(match[yIndex])
            });
          } else {
            validPoints = false;
            break;
          }
        }
        
        // Only add the curve if we have all 6 valid control points
        if (validPoints && curvePoints.length === 6) {
          controlPoints.push(curvePoints);
          foundControlPoints = true; // Mark that we found control points
          console.log('Found Quintic Bezier curve with control points:', curvePoints);
        }
      }
      
      // Match point.x = x; point.y = y format
      while ((match = pointVarRegex.exec(cppCode)) !== null) {
        points.push({
          x: parseFloat(match[1]),
          y: parseFloat(match[2])
        });
      }
      
      // Extract trajectory parameters
      const params: TrajectoryParams = {
        initialVelocity: 0,
        finalVelocity: 0,
        maxVelocity: 4.0,
        acceleration: 2.0,
        deceleration: -2.0,
        maxAngularVelocity: 4.0
      };
      
      // Function to extract motion_profiling parameters using a more robust approach
      const extractMotionProfilingParams = (code: string) => {
        console.log('Extracting motion profiling parameters...');
        
        // Check if the code contains a motion_profiling call
        if (!code.includes('motion_profiling')) {
          console.log('No motion_profiling call found');
          return;
        }
        
        // Detect if we're dealing with Poses
        const hasPoses = code.includes('Pose(') && code.includes('motion_profiling');
        console.log('Has poses:', hasPoses);
        
        // Detect if we're dealing with Quintic_Bezier
        const hasBezier = code.includes('Quintic_Bezier') && code.includes('motion_profiling');
        console.log('Has Bezier:', hasBezier);
        
        // More robust regex to extract the entire motion_profiling call
        // This handles multi-line function calls and complex first parameters
        const fullCallRegex = /motion_profiling\s*\(([\s\S]*?)\)\s*;/;
        const fullCallMatch = fullCallRegex.exec(code);
        
        if (!fullCallMatch) {
          console.log('Could not extract full motion_profiling call');
          return;
        }
        
        // Extract the full parameter list
        const fullParams = fullCallMatch[1];
        console.log('Full parameters:', fullParams);
        
        // Split the parameters by comma, but be careful with nested structures
        // This is a simplified approach - a full parser would be more robust
        const paramList: string[] = [];
        let currentParam = '';
        let nestedLevel = 0;
        
        for (let i = 0; i < fullParams.length; i++) {
          const char = fullParams[i];
          
          if (char === '(' || char === '{' || char === '[') {
            nestedLevel++;
            currentParam += char;
          } else if (char === ')' || char === '}' || char === ']') {
            nestedLevel--;
            currentParam += char;
          } else if (char === ',' && nestedLevel === 0) {
            // Only split on commas at the top level
            paramList.push(currentParam.trim());
            currentParam = '';
          } else {
            currentParam += char;
          }
        }
        
        // Add the last parameter
        if (currentParam.trim()) {
          paramList.push(currentParam.trim());
        }
        
        console.log('Extracted parameters:', paramList);
        
        // Now assign parameters based on the detected type
        if (hasPoses) {
          // motion_profiling(std::vector<Pose> path, double tangent_magnitude, double v1, double v_max, double a_accel, double a_decel, double w, bool reversed)
          if (paramList.length >= 2) params.tangentMagnitude = parseFloat(paramList[1]);
          if (paramList.length >= 3) params.initialVelocity = parseFloat(paramList[2]);
          if (paramList.length >= 4) params.maxVelocity = parseFloat(paramList[3]);
          if (paramList.length >= 5) params.acceleration = parseFloat(paramList[4]);
          if (paramList.length >= 6) params.deceleration = parseFloat(paramList[5]); // Keep as positive
          if (paramList.length >= 7) {
            // Convert radians to degrees for angular velocity
            const angularVelRad = parseFloat(paramList[6]);
            params.maxAngularVelocity = angularVelRad * (180 / Math.PI);
          }
          if (paramList.length >= 8) params.reversed = paramList[7].trim() === 'true';
        } else if (hasBezier) {
          // motion_profiling(std::vector<Quintic_Bezier> path, double v1, double v_max, double a_accel, double a_decel, double w, bool reversed)
          if (paramList.length >= 2) params.initialVelocity = parseFloat(paramList[1]);
          if (paramList.length >= 3) params.maxVelocity = parseFloat(paramList[2]);
          if (paramList.length >= 4) params.acceleration = parseFloat(paramList[3]);
          if (paramList.length >= 5) params.deceleration = parseFloat(paramList[4]); // Keep as positive
          if (paramList.length >= 6) {
            // Convert radians to degrees for angular velocity
            const angularVelRad = parseFloat(paramList[5]);
            params.maxAngularVelocity = angularVelRad * (180 / Math.PI);
          }
          if (paramList.length >= 7) params.reversed = paramList[6].trim() === 'true';
        } else {
          // motion_profiling(std::vector<Point> path, double final_angle, double tangent_magnitude, double v1, double v_max, double a_accel, double a_decel, double w, bool reversed)
          if (paramList.length >= 2) params.finalAngle = parseFloat(paramList[1]);
          if (paramList.length >= 3) params.tangentMagnitude = parseFloat(paramList[2]);
          if (paramList.length >= 4) params.initialVelocity = parseFloat(paramList[3]);
          if (paramList.length >= 5) params.maxVelocity = parseFloat(paramList[4]);
          if (paramList.length >= 6) params.acceleration = parseFloat(paramList[5]);
          if (paramList.length >= 7) params.deceleration = parseFloat(paramList[6]); // Keep as positive
          if (paramList.length >= 8) {
            // Convert radians to degrees for angular velocity
            const angularVelRad = parseFloat(paramList[7]);
            params.maxAngularVelocity = angularVelRad * (180 / Math.PI);
          }
          if (paramList.length >= 9) params.reversed = paramList[8].trim() === 'true';
        }
      };
      
      // Extract motion profiling parameters using the more robust approach
      extractMotionProfilingParams(normalizedCode);
      
      // Check for reversed parameter (always the last one)
      const reversedMatch = /motion_profiling\s*\([^,]+(?:,[^,]+)*,\s*(true|false)\s*\)/;
      const reversed = reversedMatch.exec(normalizedCode);
      if (reversed) {
        params.reversed = reversed[1] === 'true';
      }
      
      if (points.length > 0) {
        console.log('Extracted points from C++ code:', points);
        console.log('Extracted trajectory parameters:', params);
        
        // If we extracted headings and have at least one, set the initial heading
        if (headings.length > 0) {
          params.initialHeading = headings[0];
          
          // If we have multiple headings, set the final heading to the last one
          if (headings.length > 1) {
            params.finalAngle = headings[headings.length - 1];
          }
          
          // Store all headings for each pose
          params.poseHeadings = headings;
        }
        
        // Set flags to indicate what type of path we found
        params.hasPoses = foundPoses;
        params.hasControlPoints = foundControlPoints;
        
        // Log what we found
        console.log('Found poses:', foundPoses);
        console.log('Found control points:', foundControlPoints);
        
        // If we found control points, use those instead of points
        if (foundControlPoints && controlPoints.length > 0) {
          // Flatten the control points into a single array for the UI
          // The page component will handle reconstructing them
          const flattenedPoints: { x: number, y: number }[] = [];
          controlPoints.forEach(curve => {
            curve.forEach(point => {
              flattenedPoints.push(point);
            });
          });
          
          // Use the flattened control points
          onPointsExtracted(flattenedPoints, { ...params, hasControlPoints: true });
        } else {
          // Use regular points
          onPointsExtracted(points, params);
        }
        setIsModalOpen(false);
      } else {
        setError('No valid points found in the C++ code. Please check the format and try again.');
      }
    } catch (error) {
      console.error('Error parsing C++ code:', error);
      setError('Error parsing C++ code. Please check the format and try again.');
    }
  };

  return (
    <>
      <button
        onClick={() => setIsModalOpen(true)}
        className="w-full bg-blue-600 hover:bg-blue-700 text-white font-medium py-2 px-4 rounded-lg transition-colors"
      >
        Import from Code
      </button>
      
      {isModalOpen && (
        <motion.div
          className="fixed inset-0 z-50 flex items-center justify-center bg-black bg-opacity-50"
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          exit={{ opacity: 0 }}
        >
          <motion.div
            className="bg-gray-800 rounded-lg shadow-xl w-full max-w-2xl"
            initial={{ scale: 0.9, y: 20 }}
            animate={{ scale: 1, y: 0 }}
            exit={{ scale: 0.9, y: 20 }}
          >
            <div className="flex items-center justify-between p-4 border-b border-gray-700">
              <h3 className="text-lg font-medium text-white">
                Import C++ Trajectory Code
              </h3>
              <button
                onClick={() => setIsModalOpen(false)}
                className="text-gray-400 hover:text-white transition-colors"
              >
                <FiX className="w-5 h-5" />
              </button>
            </div>
            
            <div className="p-4">
              <div className="mb-4">
                <p className="text-gray-300 mb-2">
                  Paste C++ code containing points, poses, or Bezier curves with trajectory parameters. The code will be analyzed to extract:
                </p>
                <ul className="list-disc ml-5 mb-4 text-gray-300 text-sm">
                  <li>Points in any format: <code className="bg-gray-700 px-1 rounded">&#123;x, y&#125;</code>, <code className="bg-gray-700 px-1 rounded">Point(x, y)</code>, etc.</li>
                  <li>Poses with heading: <code className="bg-gray-700 px-1 rounded">Pose(x, y, theta)</code> - headings will be preserved</li>
                  <li>Quintic Bezier curves: <code className="bg-gray-700 px-1 rounded">Quintic_Bezier(x1, y1, x2, y2, x3, y3)</code> or <code className="bg-gray-700 px-1 rounded">Quintic_Bezier(p1, p2, p3, p4, p5, p6)</code></li>
                  <li>Trajectory parameters from <code className="bg-gray-700 px-1 rounded">motion_profiling()</code> function calls with various signatures</li>
                  <li>Initial and final headings, velocities, accelerations, and other motion parameters</li>
                </ul>
                
                <textarea
                  value={cppCode}
                  onChange={(e) => setCppCode(e.target.value)}
                  className="w-full h-64 bg-gray-700 border border-gray-600 rounded-md p-3 font-mono text-sm text-white"
                  placeholder="Paste your C++ code here..."
                  autoFocus
                />
                
                {error && (
                  <div className="mt-2 text-sm text-red-500">
                    {error}
                  </div>
                )}
              </div>
              
              <div className="flex space-x-3">
                <button
                  type="button"
                  onClick={() => setIsModalOpen(false)}
                  className="flex-1 bg-gray-700 hover:bg-gray-600 text-white font-medium py-2 px-4 rounded-lg transition-colors flex items-center justify-center"
                >
                  <span>Cancel</span>
                  <span className="ml-2 text-xs text-gray-400">(Esc)</span>
                </button>
                <button
                  type="button"
                  onClick={parsePoints}
                  className="flex-1 bg-blue-600 hover:bg-blue-700 text-white font-medium py-2 px-4 rounded-lg transition-colors flex items-center justify-center shadow-lg"
                >
                  <span>Import Points</span>
                  <span className="ml-2 text-xs text-blue-300">(Ctrl+Enter)</span>
                </button>
              </div>
            </div>
          </motion.div>
        </motion.div>
      )}
    </>
  );
};

export default CppCodeImport;
