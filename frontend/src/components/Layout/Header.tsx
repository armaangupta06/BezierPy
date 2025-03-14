import React from 'react';
import { FiSettings, FiSave, FiFolder } from 'react-icons/fi';

/**
 * Header component with app title and global actions
 */
const Header: React.FC = () => {
  return (
    <header className="bg-gray-800 border-b border-gray-700 px-4 py-3 flex items-center justify-between">
      <div className="flex items-center space-x-2">
        <h1 className="text-xl font-bold text-blue-400">BezierPy Visualizer</h1>
        <span className="text-xs bg-blue-600 text-white px-2 py-0.5 rounded-full">Beta</span>
      </div>
      
    </header>
  );
};

export default Header;
