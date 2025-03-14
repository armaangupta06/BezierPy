import React, { useState, useEffect } from 'react';
import { motion } from 'framer-motion';
import { FiX } from 'react-icons/fi';

interface HeadingInputModalProps {
  isOpen: boolean;
  onClose: () => void;
  onConfirm: (heading: number) => void;
  position: { x: number; y: number };
}

/**
 * Modal for inputting heading when adding a new point
 */
const HeadingInputModal: React.FC<HeadingInputModalProps> = ({
  isOpen,
  onClose,
  onConfirm,
  position
}) => {
  const [inputValue, setInputValue] = useState<string>('0');
  const [error, setError] = useState<string>('');
  
  // Reset heading when modal opens
  useEffect(() => {
    if (isOpen) {
      setInputValue('0'); // Start with upward direction (90 degrees)
      setError('');
    }
  }, [isOpen]);

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setInputValue(e.target.value);
    setError('');
  };

  // Handle keyboard shortcuts
  useEffect(() => {
    if (!isOpen) return;
    
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onClose();
      } else if (e.key === 'Enter' && !e.shiftKey) {
        handleSubmit(new Event('submit') as any);
      }
    };
    
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, onClose]);
  
  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    
    // Validate input
    const parsedValue = parseFloat(inputValue);
    if (isNaN(parsedValue)) {
      setError('Please enter a valid number');
      return;
    }
    
    if (parsedValue < -180 || parsedValue > 180) {
      setError('Value must be between -180 and 180');
      return;
    }
    
    onConfirm(parsedValue);
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
            Add Point at ({position.x.toFixed(2)}, {position.y.toFixed(2)})
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
              Heading (degrees)
            </label>
            <div className="relative">
              <input
                type="text"
                value={inputValue}
                onChange={handleInputChange}
                className={`w-full bg-gray-700 border ${error ? 'border-red-500' : 'border-gray-600'} rounded-md px-3 py-2 text-white pr-8`}
                autoFocus
              />
              <div className="absolute inset-y-0 right-0 flex items-center pr-3 pointer-events-none">
                <span className="text-gray-400">°</span>
              </div>
            </div>
            {error && (
              <div className="mt-1 text-sm text-red-500">
                {error}
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
              <span className="ml-2 text-xs text-gray-400">(Esc)</span>
            </button>
            <button
              type="submit"
              className="flex-1 bg-blue-600 hover:bg-blue-700 text-white font-medium py-2 px-4 rounded-lg transition-colors flex items-center justify-center shadow-lg"
            >
              <span>Add Point</span>
              <span className="ml-2 text-xs text-blue-300">(Enter)</span>
            </button>
          </div>
          

        </form>
      </motion.div>
    </motion.div>
  );
};

export default HeadingInputModal;
