import React, { useState, useEffect } from 'react';
import { motion } from 'framer-motion';
import { FiX } from 'react-icons/fi';

interface PointInputModalProps {
  isOpen: boolean;
  onClose: () => void;
  onConfirm: (x: number, y: number) => void;
  position: { x: number; y: number };
  isEditMode?: boolean;
}

/**
 * Modal for inputting/editing point coordinates
 */
const PointInputModal: React.FC<PointInputModalProps> = ({
  isOpen,
  onClose,
  onConfirm,
  position,
  isEditMode = false
}) => {
  const [xValue, setXValue] = useState<string>('');
  const [yValue, setYValue] = useState<string>('');
  const [coordError, setCoordError] = useState<string>('');
  
  // Reset values when modal opens
  useEffect(() => {
    if (isOpen) {
      setXValue(position.x.toFixed(2));
      setYValue(position.y.toFixed(2));
      setCoordError('');
    }
  }, [isOpen, position]);

  const handleXChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setXValue(e.target.value);
    setCoordError('');
  };

  const handleYChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setYValue(e.target.value);
    setCoordError('');
  };

  // Handle keyboard shortcuts - only for Escape key
  useEffect(() => {
    if (!isOpen) return;
    
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onClose();
      }
      // Enter key is now handled by the form's onSubmit
    };
    
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, onClose]);
  
  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    
    // Validate coordinates
    const parsedX = parseFloat(xValue);
    const parsedY = parseFloat(yValue);
    
    if (isNaN(parsedX) || isNaN(parsedY)) {
      setCoordError('Please enter valid coordinates');
      return;
    }
    
    // Prevent any potential double-submission
    if (isOpen) {
      onConfirm(parsedX, parsedY);
    }
  };

  if (!isOpen) return null;

  return (
    <motion.div
      className="fixed inset-0 z-50 flex items-center justify-center bg-black bg-opacity-50"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
    >
      <motion.div
        className="bg-gray-800 rounded-lg shadow-xl w-80"
        initial={{ scale: 0.9, y: 20 }}
        animate={{ scale: 1, y: 0 }}
        exit={{ scale: 0.9, y: 20 }}
      >
        <div className="flex items-center justify-between p-4 border-b border-gray-700">
          <h3 className="text-lg font-medium text-white">
            {isEditMode ? 'Edit Point' : 'Add Point'}
          </h3>
          <button
            onClick={onClose}
            className="text-gray-400 hover:text-white transition-colors"
          >
            <FiX className="w-5 h-5" />
          </button>
        </div>
        
        <form onSubmit={handleSubmit} className="p-4">
          <div className="mb-4">
            <label className="block text-sm font-medium text-gray-400 mb-1">
              Coordinates
            </label>
            <div className="flex space-x-2 mb-2">
              <div className="relative flex-1">
                <label className="text-xs text-gray-500 mb-1 block">X</label>
                <input
                  type="text"
                  value={xValue}
                  onChange={handleXChange}
                  className={`w-full bg-gray-700 border ${coordError ? 'border-red-500' : 'border-gray-600'} rounded-md px-3 py-2 text-white`}
                  autoFocus
                />
              </div>
              <div className="relative flex-1">
                <label className="text-xs text-gray-500 mb-1 block">Y</label>
                <input
                  type="text"
                  value={yValue}
                  onChange={handleYChange}
                  className={`w-full bg-gray-700 border ${coordError ? 'border-red-500' : 'border-gray-600'} rounded-md px-3 py-2 text-white`}
                />
              </div>
            </div>
            {coordError && (
              <div className="mt-1 text-sm text-red-500 mb-4">
                {coordError}
              </div>
            )}
          </div>
          
          <div className="flex space-x-3">
            <button
              type="button"
              onClick={onClose}
              className="flex-1 bg-gray-700 hover:bg-gray-600 text-white font-medium py-2 px-4 rounded-lg transition-colors flex items-center justify-center"
            >
              <span>Cancel</span>
            </button>
            <button
              type="submit"
              className="flex-1 bg-blue-600 hover:bg-blue-700 text-white font-medium py-2 px-4 rounded-lg transition-colors flex items-center justify-center shadow-lg"
            >
              <span>{isEditMode ? 'Update Point' : 'Add Point'}</span>
            </button>
          </div>
        </form>
      </motion.div>
    </motion.div>
  );
};

export default PointInputModal;
