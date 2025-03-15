'use client';

interface BezierScriptLoaderProps {
  children: React.ReactNode;
  onLoad?: () => void;
  onError?: (error: Error) => void;
}

/**
 * Component that serves as a wrapper for Bezier functionality
 * Since we're now using ES modules, we don't need to dynamically load scripts
 */
const BezierScriptLoader: React.FC<BezierScriptLoaderProps> = ({ 
  children,
  onLoad,
  onError 
}) => {
  // With ES modules, we don't need to load scripts dynamically anymore
  // But we'll call onLoad to maintain compatibility with existing code
  if (onLoad) {
    // Call onLoad on the next tick to simulate async loading
    setTimeout(onLoad, 0);
  }
  
  return <>{children}</>;
};

export default BezierScriptLoader;
