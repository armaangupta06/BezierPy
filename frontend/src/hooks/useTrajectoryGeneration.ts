import { useState } from 'react';
import { useMutation, useQuery, QueryClient } from '@tanstack/react-query';
import apiService, { 
  TrajectoryParamsModel, 
  TrajectoryResponse 
} from '@/services/api';
import bezierService from '@/services/bezierService';

interface UseTrajectoryGenerationProps {
  queryClient: QueryClient;
  onSuccess?: (trajectoryId: string, points: any[]) => void;
  onError?: (error: Error) => void;
}

/**
 * Hook for trajectory generation operations
 */
export default function useTrajectoryGeneration({
  queryClient,
  onSuccess,
  onError,
}: UseTrajectoryGenerationProps) {
  const [isLoading, setIsLoading] = useState(false);
  // Generate trajectory from path
  const generateTrajectoryMutation = useMutation<
    TrajectoryResponse, 
    Error, 
    { pathId: string, params: TrajectoryParamsModel }
  >({
    mutationFn: async ({ pathId, params }) => {
      try {
        // Use local bezierService instead of API
        const response = bezierService.generateTrajectory(pathId, params);
        return Promise.resolve(response);
      } catch (error) {
        console.error('Error in local trajectory generation:', error);
        throw error;
      }
    },
    onSuccess: (data) => {
      if (onSuccess) {
        onSuccess(data.trajectory_id, data.points || []);
      }
      // Invalidate queries that might be affected
      queryClient.invalidateQueries({ queryKey: ['trajectory', data.trajectory_id] });
    },
    onError: (error: Error) => {
      console.error('Error generating trajectory:', error);
      if (onError) {
        onError(error);
      }
    },
    onSettled: () => {
      setIsLoading(false);
    },
  });

  // Get trajectory by ID
  const getTrajectory = (trajectoryId: string | null) => {
    return useQuery({
      queryKey: ['trajectory', trajectoryId],
      queryFn: () => apiService.getTrajectory(trajectoryId!),
      enabled: !!trajectoryId, // Only run query if trajectoryId is provided
    });
  };

  // Delete trajectory
  const deleteTrajectory = useMutation({
    mutationFn: (trajectoryId: string) => 
      apiService.deleteTrajectory(trajectoryId),
  });

  // Generate trajectory wrapper function
  const generateTrajectory = async (pathId: string, params: TrajectoryParamsModel) => {
    if (!pathId) {
      throw new Error('Path ID is required to generate a trajectory');
    }

    setIsLoading(true);
    return generateTrajectoryMutation.mutate({ pathId, params });
  };

  return {
    generateTrajectory,
    getTrajectory,
    deleteTrajectory,
    isLoading,
  };
}


