import { useState } from 'react';
import { useMutation, useQuery, QueryClient } from '@tanstack/react-query';
import apiService, { 
  TrajectoryParamsModel, 
  TrajectoryResponse 
} from '@/services/api';

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
  const generateTrajectoryMutation = useMutation({
    mutationFn: ({ pathId, params }: { pathId: string, params: TrajectoryParamsModel }) => 
      apiService.generateTrajectory(pathId, params),
    onSuccess: async (data) => {
      try {
        const trajectoryResponse = await apiService.getTrajectory(data.trajectory_id);
        if (onSuccess) {
          onSuccess(data.trajectory_id, trajectoryResponse.points || []);
        }
        // Invalidate queries that might be affected
        queryClient.invalidateQueries({ queryKey: ['trajectory', data.trajectory_id] });
      } catch (error) {
        console.error('Error fetching trajectory details:', error);
        if (onError) {
          onError(error instanceof Error ? error : new Error('Unknown error'));
        }
      }
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


