import React, { useState, useEffect } from 'react';
import { motion } from 'framer-motion';

interface HeadingsInputModalProps {
  isOpen: boolean;
  onClose: () => void;
  onConfirm: (initialHeading: number, finalHeading?: number) => void;
  initialHeading?: number;
  finalHeading?: number;
}

/**
 * Modal for inputting initial and final heading for path generation from points
 */
const HeadingsInputModal: React.FC<HeadingsInputModalProps> = ({
  isOpen,
  onClose,
  onConfirm,
  initialHeading: defaultInitialHeading = 0,
  finalHeading: defaultFinalHeading,
}) => {
  const [initialInputValue, setInitialInputValue] = useState<string>(defaultInitialHeading.toString());
  const [finalInputValue, setFinalInputValue] = useState<string>(
    defaultFinalHeading !== undefined ? defaultFinalHeading.toString() : '0'
  );
  const [useFinalHeading, setUseFinalHeading] = useState<boolean>(defaultFinalHeading !== undefined);
  const [initialError, setInitialError] = useState<string>('');
  const [finalError, setFinalError] = useState<string>('');
  
  // Reset values when modal opens
  useEffect(() => {
    if (isOpen) {
      setInitialInputValue(defaultInitialHeading.toString());
      setFinalInputValue(defaultFinalHeading !== undefined ? defaultFinalHeading.toString() : '0');
      setUseFinalHeading(defaultFinalHeading !== undefined);
      setInitialError('');
      setFinalError('');
    }
  }, [isOpen, defaultInitialHeading, defaultFinalHeading]);

  const handleInitialInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setInitialInputValue(e.target.value);
    setInitialError('');
  };

  const handleFinalInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setFinalInputValue(e.target.value);
    setFinalError('');
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    
    // Validate initial heading
    const initialValue = parseFloat(initialInputValue);
    if (isNaN(initialValue)) {
      setInitialError('Please enter a valid number');
      return;
    }
    
    if (initialValue < -180 || initialValue > 180) {
      setInitialError('Value must be between -180 and 180');
      return;
    }
    
    // Validate final heading if enabled
    if (useFinalHeading) {
      const finalValue = parseFloat(finalInputValue);
      if (isNaN(finalValue)) {
        setFinalError('Please enter a valid number');
        return;
      }
      
      if (finalValue < -180 || finalValue > 180) {
        setFinalError('Value must be between -180 and 180');
        return;
      }
      
      onConfirm(initialValue, finalValue);
    } else {
      onConfirm(initialValue, undefined);
    }
  };

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-black bg-opacity-60 flex items-center justify-center z-50">
      <motion.div
        initial={{ opacity: 0, scale: 0.95 }}
        animate={{ opacity: 1, scale: 1 }}
        exit={{ opacity: 0, scale: 0.95 }}
        transition={{ duration: 0.15 }}
        className="bg-gray-800 rounded-lg shadow-xl p-6 w-full max-w-sm mx-4"
      >
        <h2 className="text-lg font-medium text-white mb-4">Set Path Headings</h2>
        
        <form onSubmit={handleSubmit}>
          <div className="space-y-4">
            {/* Initial Heading Input */}
            <div>
              <label htmlFor="initialHeading" className="block text-sm font-medium text-gray-300 mb-2">
                Initial Heading (degrees)
              </label>
              <div className="relative">
                <input
                  type="text"
                  id="initialHeading"
                  value={initialInputValue}
                  onChange={handleInitialInputChange}
                  className={`w-full bg-gray-700 border ${initialError ? 'border-red-500' : 'border-gray-600'} rounded-md px-3 py-2 text-white pr-8`}
                  autoFocus
                />
                <div className="absolute inset-y-0 right-0 flex items-center pr-3 pointer-events-none">
                  <span className="text-gray-400">°</span>
                </div>
              </div>
              {initialError && (
                <div className="mt-1 text-sm text-red-500">
                  {initialError}
                </div>
              )}
            </div>
            
            {/* Use Final Heading Checkbox */}
            <div className="flex items-center">
              <input
                type="checkbox"
                id="useFinalHeading"
                checked={useFinalHeading}
                onChange={(e) => setUseFinalHeading(e.target.checked)}
                className="h-4 w-4 text-blue-500 rounded border-gray-600 bg-gray-700 focus:ring-blue-500 focus:ring-offset-gray-800"
              />
              <label htmlFor="useFinalHeading" className="ml-2 text-sm font-medium text-gray-300">
                Specify final heading
              </label>
            </div>
            
            {/* Final Heading Input (conditional) */}
            {useFinalHeading && (
              <div>
                <label htmlFor="finalHeading" className="block text-sm font-medium text-gray-300 mb-2">
                  Final Heading (degrees)
                </label>
                <div className="relative">
                  <input
                    type="text"
                    id="finalHeading"
                    value={finalInputValue}
                    onChange={handleFinalInputChange}
                    className={`w-full bg-gray-700 border ${finalError ? 'border-red-500' : 'border-gray-600'} rounded-md px-3 py-2 text-white pr-8`}
                  />
                  <div className="absolute inset-y-0 right-0 flex items-center pr-3 pointer-events-none">
                    <span className="text-gray-400">°</span>
                  </div>
                </div>
                {finalError && (
                  <div className="mt-1 text-sm text-red-500">
                    {finalError}
                  </div>
                )}
              </div>
            )}
          </div>
          
          <div className="mt-6 flex justify-end space-x-3">
            <button
              type="button"
              onClick={onClose}
              className="px-4 py-2 bg-gray-700 hover:bg-gray-600 rounded-md text-sm font-medium transition-colors"
            >
              Cancel
            </button>
            <button
              type="submit"
              className="px-4 py-2 bg-blue-600 hover:bg-blue-500 rounded-md text-sm font-medium transition-colors"
            >
              Add Points
            </button>
          </div>
        </form>
      </motion.div>
    </div>
  );
};

export default HeadingsInputModal;
