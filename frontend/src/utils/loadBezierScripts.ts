/**
 * Utility to dynamically load Bezier JavaScript files
 */

// Track loading status
let scriptsLoaded = false;

// List of scripts to load
const scriptPaths = [
  '/Point.js',
  '/Pose.js',
  '/Path_Point.js',
  '/Quintic_Bezier.js',
  '/curve.js'
];

/**
 * Loads a script and returns a promise that resolves when the script is loaded
 */
const loadScript = (src: string): Promise<void> => {
  return new Promise((resolve, reject) => {
    // Check if script already exists
    const existingScript = document.querySelector(`script[src="${src}"]`);
    if (existingScript) {
      resolve();
      return;
    }

    const script = document.createElement('script');
    script.src = src;
    script.async = true;
    
    script.onload = () => resolve();
    script.onerror = (e) => reject(new Error(`Failed to load script: ${src}`));
    
    document.body.appendChild(script);
  });
};

/**
 * Loads all Bezier scripts
 */
export const loadBezierScripts = async (): Promise<boolean> => {
  if (scriptsLoaded) {
    return true;
  }
  
  try {
    // Load scripts in sequence to respect dependencies
    for (const path of scriptPaths) {
      await loadScript(path);
    }
    
    scriptsLoaded = true;
    console.log('All Bezier scripts loaded successfully');
    return true;
  } catch (error) {
    console.error('Error loading Bezier scripts:', error);
    return false;
  }
};

/**
 * Checks if all Bezier scripts are loaded
 */
export const areBezierScriptsLoaded = (): boolean => {
  if (typeof window === 'undefined') {
    return false;
  }
  
  return (
    typeof (window as any).Point !== 'undefined' &&
    typeof (window as any).Pose !== 'undefined' &&
    typeof (window as any).Path_Point !== 'undefined' &&
    typeof (window as any).Quintic_Bezier !== 'undefined' &&
    typeof (window as any).pathWithPoses !== 'undefined'
  );
};
