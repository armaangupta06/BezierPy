import React, { ReactNode } from 'react';
import Header from '@/components/Layout/Header';
import StatusBar from '@/components/Layout/StatusBar';

interface MainLayoutProps {
  children: ReactNode;
  statusInfo?: {
    pathId?: string;
    pointCount?: number;
    trajectoryId?: string;
  };
}

/**
 * Main layout component that wraps the entire application
 */
const MainLayout: React.FC<MainLayoutProps> = ({ children, statusInfo }) => {
  return (
    <div className="flex flex-col h-screen bg-gray-900 text-white">
      <Header />
      <main className="flex-1 flex overflow-hidden">
        {children}
      </main>
      <StatusBar 
        pathId={statusInfo?.pathId} 
        pointCount={statusInfo?.pointCount} 
        trajectoryId={statusInfo?.trajectoryId} 
      />
    </div>
  );
};

export default MainLayout;
